#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"  
#include "esp_err.h"
#include "esp_camera.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "mqtt_client.h"

#define WIFI_SSID      "Your_SSID"
#define WIFI_PASS      "Your_Password"
#define SERVER_IP      "Your_Server_IP"

#define HX711_DOUT_PIN      GPIO_NUM_1
#define HX711_SCK_PIN       GPIO_NUM_2
#define I2S_WS_PIN          GPIO_NUM_39
#define I2S_SCK_PIN         GPIO_NUM_40
#define I2S_SD_PIN          GPIO_NUM_41
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_SCL_IO   GPIO_NUM_38
#define DS18B20_PIN         GPIO_NUM_14

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_FREQ_HZ  400000
#define SHT31_ADDR          0x44
#define OLED_ADDR           0x3C

struct wav_header {
    char riff[4];
    uint32_t size;
    char wave[4];
    char fmt[4];
    uint32_t fmt_len;
    uint16_t format;
    uint16_t channels;
    uint32_t samplerate;
    uint32_t byterate;
    uint16_t align;
    uint16_t bps;
    char data_id[4];
    uint32_t data_len;
} __attribute__((packed));

static camera_config_t camera_config = {
    .pin_pwdn = -1,
    .pin_reset = -1,
    .pin_xclk = 15,
    .pin_sccb_sda = 4,
    .pin_sccb_scl = 5,
    .pin_d7 = 16,
    .pin_d6 = 17,
    .pin_d5 = 18,
    .pin_d4 = 12,
    .pin_d3 = 10,
    .pin_d2 = 8,
    .pin_d1 = 9,
    .pin_d0 = 11,
    .pin_vsync = 6,
    .pin_href = 7,
    .pin_pclk = 13,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_DRAM
};

float scale_factor = 427.5f; 
int32_t offset = 0;
i2s_chan_handle_t rx_chan;
esp_mqtt_client_handle_t mqtt_client = NULL;

volatile float current_weight = 0.0f;
volatile float current_temp = 0.0f;       
volatile float current_hum = 0.0f;        
volatile float current_probe_temp = 0.0f; 
volatile uint32_t current_activity = 0;
volatile bool wifi_connected = false;
volatile bool mqtt_connected = false;

portMUX_TYPE ds_mux = portMUX_INITIALIZER_UNLOCKED;
i2c_master_bus_handle_t i2c_bus_handle;
i2c_master_dev_handle_t oled_dev_handle;
i2c_master_dev_handle_t sht31_dev_handle;

int32_t hx711_read_average(int times);
int32_t hx711_read_raw(void);
void upload_photo(void);
void upload_audio(void);
void mqtt_app_start(void);

void upload_file_to_server(const char *path, const uint8_t *buffer, size_t len, const char *content_type) {
    char url[128];
    snprintf(url, sizeof(url), "http://%s:1880%s", SERVER_IP, path);

    esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", content_type);
    esp_http_client_set_post_field(client, (const char *)buffer, len);

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        printf("Upload successfully completed: %s (%zu bytes)\n", path, len);
    } else {
        printf("Upload failed for %s: %s\n", path, esp_err_to_name(err));
    }
    esp_http_client_cleanup(client);
}

void i2c_init(void) {
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &i2c_bus_handle));

    i2c_device_config_t oled_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = OLED_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &oled_dev_config, &oled_dev_handle));

    i2c_device_config_t sht31_dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT31_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &sht31_dev_config, &sht31_dev_handle));
}

const uint8_t min_font[][5] = {
    {0x3e, 0x51, 0x49, 0x45, 0x3e}, // 0
    {0x00, 0x42, 0x7f, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4b, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7f, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3c, 0x4a, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1e}, // 9
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 10: Space
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 11: .
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 12: -
    {0x3e, 0x41, 0x41, 0x41, 0x22}, // 13: C
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 14: %
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // 15: k
    {0x30, 0x4a, 0x4a, 0x4a, 0x7c}, // 16: g
    {0x01, 0x01, 0x7f, 0x01, 0x01}, // 17: T
    {0x7f, 0x20, 0x18, 0x20, 0x7f}, // 18: W
    {0x7f, 0x08, 0x08, 0x08, 0x7f}, // 19: H
    {0x06, 0x09, 0x09, 0x06, 0x00}, // 20: o (Degree symbol)
    {0x7e, 0x11, 0x11, 0x11, 0x7e}, // 21: A
    {0x3e, 0x41, 0x41, 0x41, 0x3e}, // 22: O
    {0x7f, 0x08, 0x14, 0x22, 0x41}, // 23: K
    {0x7f, 0x02, 0x0c, 0x02, 0x7f}, // 24: M
    {0x3e, 0x41, 0x51, 0x21, 0x5e}, // 25: Q
    {0x7f, 0x09, 0x09, 0x09, 0x01}, // 26: F
    {0x00, 0x44, 0x7d, 0x40, 0x00}  // 27: i
};

int get_font_idx(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    switch (c) {
        case ' ':  return 10;
        case '.':  return 11;
        case '-':  return 12;
        case 'C':  return 13;
        case '%':  return 14;
        case 'k':  return 15;
        case 'g':  return 16;
        case 'T':  return 17;
        case 'W':  return 18;
        case 'H':  return 19;
        case 'o':  return 20; 
        case 'A':  return 21;
        case 'O':  return 22;
        case 'K':  return 23;
        case 'M':  return 24;
        case 'Q':  return 25;
        case 'F':  return 26;
        case 'i':  return 27;
        default:   return 10;
    }
}

void oled_cmd(uint8_t c) {
    uint8_t d[2] = {0x00, c};
    i2c_master_transmit(oled_dev_handle, d, 2, 100);
}

void oled_init(void) {
    uint8_t cmds[] = {
        0xAE, 0x20, 0x02, 0x21, 0, 127, 0x22, 0, 7, 
        0xC8, 0x40, 0x81, 0xCF, 0xA1, 0xA6, 0xA8, 0x3F, 
        0xDA, 0x12, 0xD3, 0x00, 0xD5, 0x80, 0xD9, 0xF1, 
        0xDB, 0x40, 0x8D, 0x14, 0xAF
    };
    for (size_t i = 0; i < sizeof(cmds); i++) {
        oled_cmd(cmds[i]);
    }
}

void oled_clear(void) {
    for (int page = 0; page < 8; page++) {
        oled_cmd(0xB0 + page);
        oled_cmd(0x00);
        oled_cmd(0x10);
        for (int col = 0; col < 128; col++) {
            uint8_t d[2] = {0x40, 0x00};
            i2c_master_transmit(oled_dev_handle, d, 2, 100);
        }
    }
}

void oled_print(uint8_t page, uint8_t col, const char* str) {
    oled_cmd(0xB0 + page);
    oled_cmd(0x00 + (col & 0x0F));
    oled_cmd(0x10 + ((col >> 4) & 0x0F));
    
    while (*str) {
        int idx = get_font_idx(*str++);
        for (int i = 0; i < 5; i++) {
            uint8_t d[2] = {0x40, min_font[idx][i]};
            i2c_master_transmit(oled_dev_handle, d, 2, 100);
        }
        uint8_t space[2] = {0x40, 0x00};
        i2c_master_transmit(oled_dev_handle, space, 2, 100);
    }
}

void ds18b20_init(void) {
    gpio_set_direction(DS18B20_PIN, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_pull_mode(DS18B20_PIN, GPIO_PULLUP_ONLY);
    gpio_set_level(DS18B20_PIN, 1);
}

bool ds18b20_reset(void) {
    gpio_set_level(DS18B20_PIN, 0);
    esp_rom_delay_us(500);
    
    portENTER_CRITICAL(&ds_mux);
    gpio_set_level(DS18B20_PIN, 1);
    esp_rom_delay_us(55);
    bool present = !gpio_get_level(DS18B20_PIN);
    portEXIT_CRITICAL(&ds_mux);
    
    esp_rom_delay_us(400);
    return present;
}

void ds18b20_write_bit(uint8_t bit) {
    portENTER_CRITICAL(&ds_mux);
    gpio_set_level(DS18B20_PIN, 0);
    esp_rom_delay_us(2);
    if (bit) {
        gpio_set_level(DS18B20_PIN, 1);
    }
    esp_rom_delay_us(60);
    gpio_set_level(DS18B20_PIN, 1);
    esp_rom_delay_us(2);
    portEXIT_CRITICAL(&ds_mux);
}

uint8_t ds18b20_read_bit(void) {
    uint8_t bit = 0;
    portENTER_CRITICAL(&ds_mux);
    gpio_set_level(DS18B20_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(DS18B20_PIN, 1);
    esp_rom_delay_us(10);
    bit = gpio_get_level(DS18B20_PIN);
    esp_rom_delay_us(50);
    portEXIT_CRITICAL(&ds_mux);
    return bit;
}

void ds18b20_write_byte(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(data & 0x01);
        data >>= 1;
    }
}

uint8_t ds18b20_read_byte(void) {
    uint8_t data = 0;
    for (int i = 0; i < 8; i++) {
        if (ds18b20_read_bit()) {
            data |= (1 << i);
        }
    }
    return data;
}

void hx711_init(void) {
    gpio_reset_pin(HX711_DOUT_PIN);
    gpio_set_direction(HX711_DOUT_PIN, GPIO_MODE_INPUT);
    gpio_reset_pin(HX711_SCK_PIN);
    gpio_set_direction(HX711_SCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(HX711_SCK_PIN, 0);
}

int32_t hx711_read_raw(void) {
    int wait_counter = 0;
    while (gpio_get_level(HX711_DOUT_PIN) == 1) {
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_counter++;
        if (wait_counter > 100) {
            return 0;
        }
    }
    
    int32_t count = 0;
    portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
    portENTER_CRITICAL(&mux);
    
    for (int i = 0; i < 24; i++) {
        gpio_set_level(HX711_SCK_PIN, 1);
        esp_rom_delay_us(1);
        count = count << 1;
        gpio_set_level(HX711_SCK_PIN, 0);
        esp_rom_delay_us(1);
        if (gpio_get_level(HX711_DOUT_PIN)) {
            count++;
        }
    }
    
    gpio_set_level(HX711_SCK_PIN, 1);
    esp_rom_delay_us(1);
    gpio_set_level(HX711_SCK_PIN, 0);
    esp_rom_delay_us(1);
    portEXIT_CRITICAL(&mux);
    
    if (count & 0x800000) {
        count |= 0xFF000000;
    }
    return count;
}

int32_t hx711_read_average(int times) {
    int64_t sum = 0;
    for (int i = 0; i < times; i++) {
        sum += hx711_read_raw();
    }
    return (int32_t)(sum / times);
}

void hx711_tare(void) {
    printf("Scale: Taring...\n");
    offset = hx711_read_average(10);
    printf("Scale: Done.\n");
}

void init_microphone(void) {
    // Uses I2S_NUM_1 to prevent resources conflicts with the camera module
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));
    
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000), 
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO), 
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din = I2S_SD_PIN,
            .invert_flags = {0}
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_connected = false;
        mqtt_connected = false;
        esp_wifi_connect();
        printf("WiFi: Connection lost. Reconnecting...\n");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("WiFi: Connected. IP: " IPSTR "\n", IP2STR(&event->ip_info.ip));
        wifi_connected = true;
        mqtt_app_start();
    }
}

void wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("MQTT: Connected.\n");
            mqtt_connected = true;
            esp_mqtt_client_subscribe(mqtt_client, "bienen/befehl", 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            printf("MQTT: Connection lost.\n");
            mqtt_connected = false;
            break;
        case MQTT_EVENT_DATA:
            printf("MQTT: Command received: %.*s\n", event->data_len, event->data);
            if (strncmp(event->data, "take_photo", event->data_len) == 0) {
                upload_photo();
            } else if (strncmp(event->data, "record_audio", event->data_len) == 0) {
                upload_audio();
            }
            break;
        default:
            break;
    }
}

void mqtt_app_start(void) {
    char url[64];
    snprintf(url, sizeof(url), "mqtt://%s", SERVER_IP);
    
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = url
    };
    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void upload_photo(void) {
    printf("Camera: Capturing photo...\n");
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        printf("Camera: Capture failed!\n");
        return;
    }
    upload_file_to_server("/upload/image", fb->buf, fb->len, "image/jpeg");
    esp_camera_fb_return(fb);
}

void upload_audio(void) {
    printf("Audio: Capturing 3 seconds (DSP Noise-Filter enabled)...\n");
    
    uint32_t sample_rate = 16000;
    uint32_t duration_sec = 3;
    uint32_t num_samples = sample_rate * duration_sec;
    uint32_t data_size = num_samples * sizeof(int16_t);
    uint32_t total_size = data_size + sizeof(struct wav_header);

    uint8_t *wav_buf = malloc(total_size);
    if (!wav_buf) {
        printf("Audio: Allocation failed (insufficient RAM).\n");
        return;
    }

    struct wav_header header = {
        .riff = {'R', 'I', 'F', 'F'},
        .size = total_size - 8,
        .wave = {'W', 'A', 'V', 'E'},
        .fmt = {'f', 'm', 't', ' '},
        .fmt_len = 16,
        .format = 1,
        .channels = 1,
        .samplerate = sample_rate,
        .byterate = sample_rate * sizeof(int16_t),
        .align = sizeof(int16_t),
        .bps = 16,
        .data_id = {'d', 'a', 't', 'a'},
        .data_len = data_size
    };
    memcpy(wav_buf, &header, sizeof(struct wav_header));

    const uint32_t chunk_frames = 256;
    int32_t *raw_buf = malloc(chunk_frames * 2 * sizeof(int32_t));
    if (!raw_buf) {
        free(wav_buf);
        printf("Audio: Temp-buffer allocation failed.\n");
        return;
    }

    uint32_t samples_collected = 0;
    int16_t *pcm_ptr = (int16_t *)(wav_buf + sizeof(struct wav_header));
    
    // DSP filter parameters
    float prev_x = 0; 
    float prev_y = 0; 
    float alpha = 0.99f; // High-pass filter (DC removal)
    float lpf_y = 0; 
    float beta = 0.35f;  // Low-pass filter (noise smoothing)
    float gain = 6.5f;

    while (samples_collected < num_samples) {
        size_t bytes_read = 0;
        uint32_t chunk = (num_samples - samples_collected > chunk_frames) ? 
                          chunk_frames : (num_samples - samples_collected);
                          
        esp_err_t res = i2s_channel_read(rx_chan, raw_buf, chunk * 2 * sizeof(int32_t), &bytes_read, portMAX_DELAY);
        if (res == ESP_OK) {
            uint32_t read_samples = bytes_read / sizeof(int32_t);
            uint32_t read_frames = read_samples / 2;
            
            for (uint32_t i = 0; i < read_frames; i++) {
                int32_t raw_sample = raw_buf[i * 2] >> 14;
                
                // DC offset removal (High-pass)
                float x = (float)raw_sample;
                float y = x - prev_x + alpha * prev_y;
                prev_x = x; 
                prev_y = y;
                
                // High frequency filter (Low-pass)
                lpf_y = beta * y + (1.0f - beta) * lpf_y;
                
                // Apply digital gain
                int32_t corrected = (int32_t)(lpf_y * gain);
                
                // Noise gate for silent intervals
                if (corrected < 180 && corrected > -180) {
                    corrected /= 5; 
                }
                
                // Hard clipping protection
                if (corrected > 32767) {
                    corrected = 32767;
                }
                if (corrected < -32768) {
                    corrected = -32768;
                }
                
                *pcm_ptr++ = (int16_t)corrected;
            }
            samples_collected += read_frames;
        } else {
            break;
        }
    }
    free(raw_buf);

    printf("Audio: Capturing complete (%lu bytes). Uploading...\n", total_size);
    upload_file_to_server("/upload/audio", wav_buf, total_size, "audio/wav");
    free(wav_buf);
}

void scale_task(void *pvParameters) {
    while (1) {
        int32_t current_raw = hx711_read_average(3);
        int32_t raw_without_offset = current_raw - offset;
        current_weight = (float)raw_without_offset / scale_factor;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sht31_task(void *pvParameters) {
    uint8_t cmd[2] = {0x24, 0x00}; 
    uint8_t data[6];
    while (1) {
        if (i2c_master_transmit(sht31_dev_handle, cmd, 2, 1000) == ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(20));
            if (i2c_master_receive(sht31_dev_handle, data, 6, 1000) == ESP_OK) {
                uint16_t t_raw = (data[0] << 8) | data[1]; 
                uint16_t h_raw = (data[3] << 8) | data[4];
                current_temp = -45.0f + 175.0f * ((float)t_raw / 65535.0f);
                current_hum = 100.0f * ((float)h_raw / 65535.0f);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void ds18b20_task(void *pvParameters) {
    ds18b20_init();
    while (1) {
        if (ds18b20_reset()) {
            ds18b20_write_byte(0xCC); 
            ds18b20_write_byte(0x44); 
            vTaskDelay(pdMS_TO_TICKS(800));
            
            if (ds18b20_reset()) {
                ds18b20_write_byte(0xCC); 
                ds18b20_write_byte(0xBE);
                uint8_t temp_lsb = ds18b20_read_byte(); 
                uint8_t temp_msb = ds18b20_read_byte();
                int16_t raw_temp = (temp_msb << 8) | temp_lsb; 
                current_probe_temp = raw_temp / 16.0f;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void display_task(void *pvParameters) {
    oled_init(); 
    oled_clear();
    char buf[32];
    
    while (1) {
        snprintf(buf, sizeof(buf), "W   %.2f kg", current_weight); 
        oled_print(0, 0, buf); 
        
        snprintf(buf, sizeof(buf), "T1  %.1f oC", current_temp); 
        oled_print(1, 0, buf); 
        
        snprintf(buf, sizeof(buf), "T2  %.1f oC", current_probe_temp); 
        oled_print(2, 0, buf); 
        
        snprintf(buf, sizeof(buf), "H   %.1f %%", current_hum); 
        oled_print(3, 0, buf);

        snprintf(buf, sizeof(buf), "A   %lu %%", current_activity); 
        oled_print(4, 0, buf);

        oled_print(5, 0, "--------------");

        snprintf(buf, sizeof(buf), "WiFi %s", wifi_connected ? "OK" : "--"); 
        oled_print(6, 0, buf);

        snprintf(buf, sizeof(buf), "MQTT %s", mqtt_connected ? "OK" : "--"); 
        oled_print(7, 0, buf);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void telemetry_task(void *pvParameters) {
    char payload[256];
    const int sample_cnt = 512;
    int32_t *raw_buf = malloc(sample_cnt * 2 * sizeof(int32_t));
    
    while (1) {
        uint32_t avg_volume = 0;
        uint32_t activity_index = 0;

        if (raw_buf) {
            size_t bytes_read = 0;
            // Short 32ms microphone acquisition for telemetry calculations
            esp_err_t res = i2s_channel_read(rx_chan, raw_buf, sample_cnt * 2 * sizeof(int32_t), &bytes_read, pdMS_TO_TICKS(100));
            if (res == ESP_OK) {
                uint32_t read_samples = bytes_read / sizeof(int32_t);
                uint32_t read_frames = read_samples / 2;
                int64_t sum_amp = 0;
                
                for (uint32_t i = 0; i < read_frames; i++) {
                    int32_t sample = raw_buf[i * 2] >> 16;
                    sum_amp += (sample < 0) ? -sample : sample;
                }
                if (read_frames > 0) {
                    avg_volume = sum_amp / read_frames;
                }
            }
        }

        activity_index = avg_volume / 80;
        if (activity_index > 100) {
            activity_index = 100;
        }
        current_activity = activity_index;

        if (mqtt_client) {
            snprintf(payload, sizeof(payload), 
                     "{\"weight\":%.2f,\"temp_stock\":%.1f,\"temp_brut\":%.1f,\"humidity\":%.1f,\"activity\":%lu}",
                     current_weight, current_temp, current_probe_temp, current_hum, activity_index);
            
            esp_mqtt_client_publish(mqtt_client, "bienen/sensoren", payload, 0, 1, 0);
            printf("MQTT: Telemetry sent: %s\n", payload);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    free(raw_buf); 
}

void app_main(void) {
    printf("Starting SmartBeehive System...\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    hx711_init();
    i2c_init(); 
    vTaskDelay(pdMS_TO_TICKS(500));

    hx711_tare(); 

    // Note: Initialize the camera first before starting I2S microphone tasks 
    // to avoid bus conflicts on power-on reset.
    printf("Camera: Initializing OV2640...\n");
    ESP_ERROR_CHECK(esp_camera_init(&camera_config));
    printf("Camera: Initialized successfully.\n");

    printf("Microphone: Initializing INMP441...\n");
    init_microphone();
    printf("Microphone: Initialized successfully.\n");

    wifi_init_sta();

    xTaskCreatePinnedToCore(scale_task, "Scale Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(sht31_task, "SHT31 Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(ds18b20_task, "DS18B20 Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(display_task, "Display Task", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(telemetry_task, "Telemetry Task", 4096, NULL, 2, NULL, 0);
}