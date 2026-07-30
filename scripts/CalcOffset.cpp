#include "HX711.h"

const int LOADCELL_DOUT_PIN = 3;
const int LOADCELL_SCK_PIN = 2;

HX711 scale;

#define NUM_READINGS 500

void setup()
{
    Serial.begin(57600);

    scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

    Serial.println("HX711 Offset Calibration");
    Serial.println("Remove all weight from the scale.");
    Serial.println("Waiting 30 seconds...");

    delay(30000);

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

    long offset = sum / NUM_READINGS;

    Serial.println();
    Serial.println("================================");
    Serial.print("OFFSET = ");
    Serial.println(offset);
    Serial.println("================================");
}

void loop()
{
}