#include <math.h>

#include <DallasTemperature.h>
#include <OneWire.h>
#include <unity.h>

#include "pins.h"

OneWire probeTemperatureOneWire(PROBE_TEMPERATURE_PIN);
DallasTemperature probeTemperatureSensor(&probeTemperatureOneWire);

void setupProbeTemperatureSensor() {
    probeTemperatureSensor.begin();

    if (probeTemperatureSensor.getDeviceCount() < 1) {
        TEST_FAIL_MESSAGE("No DS18B20 probe found. Check data pin, power, ground, and 4.7k pull-up.");
    }

    probeTemperatureSensor.setResolution(9);
}

void test_probe_temperature_reading() {
    float temperature_c = DEVICE_DISCONNECTED_C;

    for (int i = 0; i < 5; i++) {
        probeTemperatureSensor.requestTemperatures();
        temperature_c = probeTemperatureSensor.getTempCByIndex(0);

        Serial.print("Probe temperature C: ");
        Serial.println(temperature_c, 3);
        delay(100);
    }

    TEST_ASSERT_NOT_EQUAL_MESSAGE(
        DEVICE_DISCONNECTED_C,
        temperature_c,
        "DS18B20 probe disconnected or not responding"
    );
    TEST_ASSERT_TRUE_MESSAGE(isfinite(temperature_c), "Probe temperature reading is not finite");
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(
        85.0f,
        25.0f,
        temperature_c,
        "Probe temperature is outside a broad bench-test range"
    );
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100);
    }

    UNITY_BEGIN();

    setupProbeTemperatureSensor();
    RUN_TEST(test_probe_temperature_reading);

    UNITY_END();
}

void loop() {
    // Unity test framework doesn't use the loop function.
}
