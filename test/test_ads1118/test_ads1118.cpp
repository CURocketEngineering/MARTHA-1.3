#include <math.h>

#include <ADS1118.h>
#include <unity.h>

#include "pins.h"

ADS1118 externalAdc;

void setupADS1118() {
    if (!externalAdc.begin_SPI(EXTERNAL_ADC_CS)) {
        TEST_FAIL_MESSAGE("Failed to initialize ADS1118. Check SPI wiring and chip select!");
    }

    externalAdc.setSamplingRate(externalAdc.RATE_860SPS);
    externalAdc.setInputSelected(externalAdc.AIN_0);
    externalAdc.setFullScaleRange(externalAdc.FSR_4096);
}

void test_ads1118_voltage_reading() {
    float voltage = 0.0f;

    for (int i = 0; i < 10; i++) {
        voltage = static_cast<float>(externalAdc.getMilliVolts() / 1000.0);
        Serial.print("ADS1118 AIN0 voltage: ");
        Serial.println(voltage, 6);
        delay(10);
    }

    TEST_ASSERT_TRUE_MESSAGE(isfinite(voltage), "ADS1118 voltage reading is not finite");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
        4.096f,
        0.0f,
        voltage,
        "ADS1118 single-ended AIN0 reading is outside the configured +/-4.096 V range"
    );
}

void test_ads1118_temperature_reading() {
    float temperature_c = 0.0f;

    for (int i = 0; i < 5; i++) {
        temperature_c = static_cast<float>(externalAdc.getTemperature());
        Serial.print("ADS1118 internal temperature C: ");
        Serial.println(temperature_c, 3);
        delay(10);
    }

    TEST_ASSERT_TRUE_MESSAGE(isfinite(temperature_c), "ADS1118 temperature reading is not finite");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
        85.0f,
        25.0f,
        temperature_c,
        "ADS1118 internal temperature is outside a broad bench-test range"
    );
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    UNITY_BEGIN();

    setupADS1118();

    RUN_TEST(test_ads1118_voltage_reading);
    RUN_TEST(test_ads1118_temperature_reading);

    UNITY_END();
}

void loop() {
    // Unity test framework doesn't use the loop function.
}
