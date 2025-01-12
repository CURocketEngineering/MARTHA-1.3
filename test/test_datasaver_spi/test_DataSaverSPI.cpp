#include <unity.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include "data_handling/DataSaverSPI.h"
#include "data_handling/DataPoint.h"

#include "flash_config.h"

#define TEST_ADDRESS 0x00010000 // Example address for testing
#define TEST_DATA 0xA5         // Example test data pattern
#define TEST_BYTE_SIZE 3288    // Size for time-based test case
#define TEST_TIME_LIMIT 1000   // Time limit in milliseconds

Adafruit_SPIFlash flash(&flashTransport);
DataSaverSPI dataSaver(10, &flash); // Save data every 10 ms

void test_flash_init() {
    unsigned long start_time = millis();
    TEST_ASSERT_EQUAL(true, flash.begin());
    TEST_ASSERT_GREATER_THAN(0, flash.size());
    Serial.printf("test_flash_init execution time: %lu ms\n", millis() - start_time);
}

void test_data_saver_begin() {
    unsigned long start_time = millis();
    TEST_ASSERT_EQUAL(true, dataSaver.begin());
    Serial.printf("test_data_saver_begin execution time: %lu ms\n", millis() - start_time);
}

void test_save_data_point() {
    DataPoint dp = {100, 50}; // Example DataPoint: timestamp = 100ms, value = 50
    unsigned long start_time = millis();
    TEST_ASSERT_EQUAL(0, dataSaver.saveDataPoint(dp, 1)); // 1 as name/ID
    TEST_ASSERT_EQUAL(100, dataSaver.getLastTimestamp());
    TEST_ASSERT_EQUAL(50, dataSaver.getLastDataPoint().data);
    Serial.printf("test_save_data_point execution time: %lu ms\n", millis() - start_time);
}

void test_time_based_write() {
    uint8_t testData[TEST_BYTE_SIZE];
    memset(testData, TEST_DATA, TEST_BYTE_SIZE); // Fill buffer with test data
    unsigned long start_time = millis();
    TEST_ASSERT_EQUAL(TEST_BYTE_SIZE, flash.writeBuffer(TEST_ADDRESS, testData, TEST_BYTE_SIZE));
    unsigned long duration = millis() - start_time;
    Serial.printf("test_time_based_write execution time: %lu ms\n", duration);
    TEST_ASSERT_LESS_THAN(TEST_TIME_LIMIT, duration);
}

void test_post_launch_mode() {
    unsigned long start_time = millis();
    dataSaver.eraseAllData();
    dataSaver.launchDetected(500); // Launch detected at 500ms
    TEST_ASSERT_EQUAL(true, dataSaver.isPostLaunchMode());
    DataPoint dp = {600, 80};
    TEST_ASSERT_EQUAL(0, dataSaver.saveDataPoint(dp, 1));
    // Check post-launch flag persistence
    dataSaver.clearPostLaunchMode();
    TEST_ASSERT_EQUAL(false, dataSaver.isPostLaunchMode());
    Serial.printf("test_post_launch_mode execution time: %lu ms\n", millis() - start_time);
}

void test_erase_all_data() {
    DataPoint dp = {100, 50};
    dataSaver.saveDataPoint(dp, 1);

    // Also write to flash to ensure it's erased
    uint8_t writeData = TEST_DATA;
    flash.writeBuffer(TEST_ADDRESS, &writeData, 1);

    unsigned long start_time = millis();
    dataSaver.eraseAllData();
    TEST_ASSERT_EQUAL(0, dataSaver.getLastTimestamp());

    // Read from flash to ensure it's erased
    uint8_t readData;
    flash.readBuffer(TEST_ADDRESS, &readData, 1);
    TEST_ASSERT_NOT_EQUAL(TEST_DATA, readData);

    // Ensure next write starts fresh
    dp.timestamp_ms = 200;
    TEST_ASSERT_EQUAL(0, dataSaver.saveDataPoint(dp, 1));
    TEST_ASSERT_EQUAL(200, dataSaver.getLastTimestamp());
    Serial.printf("test_erase_all_data execution time: %lu ms\n", millis() - start_time);
}

void test_post_launch_data_preservation() {
    // Ideally a launch detection should protect that previous 1 minute of data 
    // and prevent all newer data from being overwritten.

    // Reset data saver
    dataSaver.eraseAllData();

    int minute_5_ms = 5 * 60 * 1000;
    
    // Write 5 minutes of data
    Serial.println("Writing 5 minutes of data (pre-launch)");
    for (uint32_t i = 0; i <= minute_5_ms / 10; i++) {
        DataPoint dp = {i * 10, 12};  // pre-launch data has a value of 12
        dataSaver.saveDataPoint(dp, 1);
    }

    // Trigger a launch at the 5 minute mark
    dataSaver.launchDetected(minute_5_ms); // Launch detected at 5 minutes

    TEST_ASSERT_EQUAL(true, dataSaver.isPostLaunchMode());
    TEST_ASSERT_EQUAL(minute_5_ms, dataSaver.getLastTimestamp());

    // Print out the launch write address and next write address
    Serial.print("Launch write address: ");
    Serial.println(dataSaver.getLaunchWriteAddress());
    Serial.print("Next write address: ");
    Serial.println(dataSaver.getNextWriteAddress());
    
    // Ensure the launch write address is less than the getNextWriteAddress
    TEST_ASSERT_LESS_THAN(dataSaver.getNextWriteAddress(), dataSaver.getLaunchWriteAddress());

    

    // Write 5 more minutes of data
    Serial.println("Writing 5 minutes of data (post-launch)");
    for (uint32_t i = 0; i < minute_5_ms / 10; i++) {
        DataPoint dp = {i * 10 + minute_5_ms, 25}; // 5 minutes of post-launch data has a value of 25
        dataSaver.saveDataPoint(dp, 1);
    }

    // At this point if we write flash size number of bytes, we should loop back to the beginning
    // and overwrite the pre-launch data. But the post-launch data should be preserved.

    // saveDataPoint returns a 1 if the data was not saved due to post-launch data
    // if we keep saving we should get a 1 eventually

    // Write until we get a 1
    Serial.println("Writing until we loop back to the beginning");
    for (uint32_t i = 0; i < flash.size(); i++) {
        DataPoint dp = {i * 20 + 2 * minute_5_ms, 50};
        if (dataSaver.saveDataPoint(dp, 1) == 1) {
            break;
        }

        // Print progress
        if (i % 10000 == 0) {
            Serial.print("Progress (till wrap around): ");
            Serial.print(dataSaver.getNextWriteAddress() * 100 / flash.size());
            Serial.print("%");

            // print current write address
            Serial.print(" Current write address: ");
            Serial.println(dataSaver.getNextWriteAddress());
        }
    }

    // If we reached the end without getting a 1, something is wrong
    TEST_ASSERT_EQUAL(1, dataSaver.saveDataPoint({0, 0}, 1));

    Serial.print("Hit a post-launch data preservation limit at address: ");
    Serial.println(dataSaver.getNextWriteAddress());

    // Test that the nextwriteaddress is less than the launchwriteaddress
    TEST_ASSERT_LESS_THAN(dataSaver.getLaunchWriteAddress() + 100, dataSaver.getNextWriteAddress());
}

void test_data_point_byte_size() {
    // Write a few data points and ensure that the address changed by the correct amount
    dataSaver.resetTimestamp();
    DataPoint dp = {100, 50};

    uint32_t startAddress = dataSaver.getNextWriteAddress();
    dataSaver.saveDataPoint(dp, 1);

    // 5 (timestamp record) + 5 (data record)
    TEST_ASSERT_EQUAL(5 + 5, dataSaver.getNextWriteAddress() - startAddress);

    // Write another with a similar timestamp
    startAddress = dataSaver.getNextWriteAddress();
    dataSaver.saveDataPoint(dp, 1);

    // This won't have a timestamp record
    // 5 (data record)
    TEST_ASSERT_EQUAL(5, dataSaver.getNextWriteAddress() - startAddress);

    // Write 50 more
    startAddress = dataSaver.getNextWriteAddress();
    for (uint32_t i = 0; i < 50; i++) {
        dp = {200 + i * 11, 50};
        dataSaver.saveDataPoint(dp, 1);
    }

    TEST_ASSERT_EQUAL(10 * 50, dataSaver.getNextWriteAddress() - startAddress);

}

void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(100); // wait for native USB
    }

    UNITY_BEGIN();
    RUN_TEST(test_flash_init);
    RUN_TEST(test_data_saver_begin);
    RUN_TEST(test_save_data_point);
    RUN_TEST(test_data_point_byte_size);
    RUN_TEST(test_post_launch_data_preservation);
    RUN_TEST(test_time_based_write);
    RUN_TEST(test_post_launch_mode);
    RUN_TEST(test_erase_all_data);
    UNITY_END();
}

void loop() {
    // nothing to do here
}
