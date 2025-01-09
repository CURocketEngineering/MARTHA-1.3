// The MIT License (MIT)
// Copyright (c) 2019 Ha Thach for Adafruit Industries
#include <unity.h>

#include <SPI.h>
#include <SdFat.h>

#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

void test_flash_init() {
    TEST_ASSERT_EQUAL(true, flash.begin());
}

// Test JEDEC ID
void test_flash_correct_JEDEC_ID() {
    TEST_ASSERT_EQUAL(EXPECTED_JEDEC_ID, flash.getJEDECID());
}

// Test flash size
void test_flash_size_greater_than_zero() {
    TEST_ASSERT_GREATER_THAN(0, flash.size());
}

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100); // wait for native usb
    }

   

    


    UNITY_BEGIN();
    RUN_TEST(test_flash_init);
    RUN_TEST(test_flash_correct_JEDEC_ID);
    RUN_TEST(test_flash_size_greater_than_zero);


    // Using a flash device not already listed? Start the flash memory by passing
    // it the array of device settings defined above, and the number of elements
    // in the array.
    // flash.begin(my_flash_devices, flashDevices);
    Serial.println("Adafruit Serial Flash Info example");
    uint32_t jedec_id = flash.getJEDECID();
    Serial.print("JEDEC ID: 0x");
    Serial.println(jedec_id, HEX);
    Serial.print("Flash size (usable): ");
    Serial.print(flash.size() / 1024);
    Serial.println(" KB");

    UNITY_END();
}

void loop() {
  // nothing to do
}