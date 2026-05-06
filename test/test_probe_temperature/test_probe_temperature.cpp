#include <math.h>

#include <DallasTemperature.h>
#include <OneWire.h>
#include <unity.h>

#include "pins.h"

OneWire probeTemperatureOneWire(PROBE_TEMPERATURE_PIN);
DallasTemperature probeTemperatureSensor(&probeTemperatureOneWire);

void printDeviceAddress(const DeviceAddress address) {
    for (uint8_t i = 0; i < 8; i++) {
        if (address[i] < 16) {
            Serial.print("0");
        }
        Serial.print(address[i], HEX);
    }
    Serial.println();
}

void setupProbeTemperatureSensor() {
    DeviceAddress deviceAddress;
    bool parasitePower = false;

    probeTemperatureSensor.begin();
    probeTemperatureSensor.setWaitForConversion(true);

    Serial.print("Probe pin: ");
    Serial.println(PROBE_TEMPERATURE_PIN);
    Serial.print("Detected 1-Wire devices: ");
    Serial.println(probeTemperatureSensor.getDeviceCount());

    if (probeTemperatureSensor.getDeviceCount() < 1) {
        TEST_FAIL_MESSAGE("No DS18B20 probe found. Check data pin, power, ground, and 4.7k pull-up.");
    }

    if (!probeTemperatureSensor.getAddress(deviceAddress, 0)) {
        TEST_FAIL_MESSAGE("Found a 1-Wire device count, but could not read device address.");
    }

    Serial.print("Probe ROM address: ");
    printDeviceAddress(deviceAddress);

    parasitePower = probeTemperatureSensor.readPowerSupply(deviceAddress);
    Serial.print("Probe parasite power mode: ");
    Serial.println(parasitePower ? "YES" : "NO");

    TEST_ASSERT_FALSE_MESSAGE(
        parasitePower,
        "DS18B20 is reporting parasite power mode. Verify VDD is connected and not floating."
    );

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
