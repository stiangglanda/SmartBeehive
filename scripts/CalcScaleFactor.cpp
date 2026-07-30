#include "HX711.h"

const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

const long OFFSET = 543820;

const float KNOWN_WEIGHT_KG = 2.0f;

#define NUM_READINGS 500

HX711 scale;

void setup()
{
    Serial.begin(57600);

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    Serial.println("HX711 Scale Factor Calibration");
    Serial.println();

    Serial.println("Place the known weight on the scale.");
    Serial.println("Waiting 10 seconds...");
    
    delay(10000);

    Serial.println("Starting measurements...");

    long long sum = 0;

    for (int i = 0; i < NUM_READINGS; i++)
    {
        while (!scale.is_ready())
        {
            delay(10);
        }

        long reading = scale.read();
        sum += reading;

        if (i % 50 == 0)
        {
            Serial.print("Measurement ");
            Serial.print(i);
            Serial.print("/");
            Serial.println(NUM_READINGS);
        }

        delay(50);
    }

    long raw_average = sum / NUM_READINGS;

    long difference = raw_average - OFFSET;

    float scale_factor = difference / KNOWN_WEIGHT_KG;

    Serial.println();
    Serial.println("===============================");
    Serial.print("Raw average: ");
    Serial.println(raw_average);

    Serial.print("Difference: ");
    Serial.println(difference);

    Serial.print("Scale factor: ");
    Serial.println(scale_factor, 3);

    Serial.println("===============================");
}

void loop()
{
}