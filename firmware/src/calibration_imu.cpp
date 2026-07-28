/**
 * @file mpu6050_calibration.cpp
 * @brief Interactive calibration script for the MPU6050 IMU.
 *
 * This script provides a menu-driven interface to:
 * 1. Calibrate Gyroscope (Hardware Offsets)
 * 2. Calibrate Accelerometer (Software Bias)
 * 3. Run Full Calibration (Both)
 * 4. Verify live sensor readings
 *
 * @note Place the board flat and level (SMD components facing up) and keep it still
 *       for the whole duration of the calibration.
 */

#include <Arduino.h>
#include "mpu6050.hpp"

constexpr int BUFFER_SIZE = 1000;      // Samples averaged per pass (higher = more precise, slower)
constexpr int DISCARD_SAMPLES = 100;   // Leading samples discarded to let readings settle
constexpr int GYRO_DEADZONE = 2;       // Gyro convergence tolerance, raw ticks
constexpr int32_t ACCEL_ONE_G = 16384; // Ideal Z reading at rest for ±2g range

MPU6050 imu(&Wire1, MPU6050::DEFAULT_ADDRESS);

MPU6050::Axis3<int32_t> mean_accel{0, 0, 0};
MPU6050::Axis3<int32_t> mean_gyro{0, 0, 0};

void meanSensors();
void calibrateGyro();
void calibrateAccel();
void verifyReadings();
void clearSerialBuffer();

void setup()
{
    Wire1.begin();
    Wire1.setClock(400000); // 400kHz I2C clock

    Serial.begin(115200);
    while (!Serial)
    {
        delay(10);
    }

    Serial.printf("\n###############################\n");
    Serial.printf("MPU6050 Calibration & Test\n");
    Serial.printf("###############################\n");

    Serial.printf("Verifying MPU-6050 connection...\n");
    while (!imu.begin(MPU6050::AccelRange::G_2, MPU6050::GyroRange::DPS_250))
    {
        Serial.printf(">>> MPU-6050 connection FAILED. Retrying in 1 second...\n");
        delay(1000);
    }
    Serial.printf("MPU-6050 initialized successfully.\n");

    Serial.printf("Resetting hardware and software offsets to zero...\n");
    imu.setGyroOffset({0, 0, 0});
    imu.setAccelOffset({0, 0, 0});
    delay(500);
}

void loop()
{
    String input = "";
    static bool repeat = true;

    while (repeat)
    {
        Serial.printf("\n===============================\n");
        Serial.printf("Select an option:\n");
        Serial.printf("1. Calibrate Gyroscope (Hardware Offsets)\n");
        Serial.printf("2. Calibrate Accelerometer (Software Bias)\n");
        Serial.printf("3. Run Full Calibration (Gyro + Accel)\n");
        Serial.printf("4. Verify Sensor Readings (Live Output)\n");
        Serial.printf("Choice (1-4): ");

        clearSerialBuffer();
        while (!Serial.available())
        {
            delay(10);
        }

        input = Serial.readStringUntil('\n').trim();
        Serial.println(input); // Echo user input

        switch (input.charAt(0))
        {
        case '1':
            calibrateGyro();
            break;
        case '2':
            calibrateAccel();
            break;
        case '3':
            calibrateGyro();
            calibrateAccel();
            break;
        case '4':
            verifyReadings();
            break;
        default:
            Serial.printf("\nInvalid option. Please select a number between 1 and 4.\n");
            break;
        }

        Serial.printf("\nDo you want to perform another action? (y/n): ");
        clearSerialBuffer();
        while (!Serial.available())
        {
            delay(10);
        }

        input = Serial.readStringUntil('\n').trim();
        repeat = input.equalsIgnoreCase("y");
    }

    Serial.printf("\nExiting calibration loop. Restart the microcontroller to run again.\n");
    while (true)
    {
        delay(1000);
    }
}

///////////////////////////////////   HELPERS   ///////////////////////////////////

/**
 * @brief Reads the sensor rapidly to compute a stable average, discarding initial readings.
 */
void meanSensors()
{
    int64_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for (int i = 0; i < BUFFER_SIZE + DISCARD_SAMPLES; i++)
    {
        MPU6050::Axis3<int16_t> accel = imu.readRawAcceleration();
        MPU6050::Axis3<int16_t> gyro = imu.readRawGyroscope();

        if (i >= DISCARD_SAMPLES)
        {
            sum_ax += accel.x;
            sum_ay += accel.y;
            sum_az += accel.z;
            sum_gx += gyro.x;
            sum_gy += gyro.y;
            sum_gz += gyro.z;
        }

        delay(2); // avoid reading the same sample twice
    }

    mean_accel.x = static_cast<int32_t>(sum_ax / BUFFER_SIZE);
    mean_accel.y = static_cast<int32_t>(sum_ay / BUFFER_SIZE);
    mean_accel.z = static_cast<int32_t>(sum_az / BUFFER_SIZE);
    mean_gyro.x = static_cast<int32_t>(sum_gx / BUFFER_SIZE);
    mean_gyro.y = static_cast<int32_t>(sum_gy / BUFFER_SIZE);
    mean_gyro.z = static_cast<int32_t>(sum_gz / BUFFER_SIZE);
}

/**
 * @brief Iteratively computes and applies hardware offsets for the gyroscope.
 */
void calibrateGyro()
{
    Serial.printf("\n--- Gyroscope Calibration ---\n");
    Serial.printf("Converging hardware offsets");

    MPU6050::Axis3<int16_t> gyro_offset{0, 0, 0};
    imu.setGyroOffset(gyro_offset);
    delay(50);

    while (true)
    {
        meanSensors();
        Serial.print(".");

        bool ready = true;

        if (abs(mean_gyro.x) > GYRO_DEADZONE)
        {
            int16_t step = mean_gyro.x / 4;
            if (step == 0)
                step = (mean_gyro.x > 0) ? 1 : -1;
            gyro_offset.x -= step;
            ready = false;
        }
        if (abs(mean_gyro.y) > GYRO_DEADZONE)
        {
            int16_t step = mean_gyro.y / 4;
            if (step == 0)
                step = (mean_gyro.y > 0) ? 1 : -1;
            gyro_offset.y -= step;
            ready = false;
        }
        if (abs(mean_gyro.z) > GYRO_DEADZONE)
        {
            int16_t step = mean_gyro.z / 4;
            if (step == 0)
                step = (mean_gyro.z > 0) ? 1 : -1;
            gyro_offset.z -= step;
            ready = false;
        }

        imu.setGyroOffset(gyro_offset);

        if (ready)
            break;
    }

    Serial.printf("\nGyroscope calibration complete!\n");
    Serial.printf("Add this to your setup():\n");
    Serial.printf("\timu.setGyroOffset({%d, %d, %d});\n", gyro_offset.x, gyro_offset.y, gyro_offset.z);
}

/**
 * @brief Computes the software bias for the accelerometer in a single pass.
 */
void calibrateAccel()
{
    Serial.printf("\n--- Accelerometer Calibration ---\n");
    Serial.printf("Computing software offsets...\n");

    imu.setAccelOffset({0, 0, 0});
    delay(50);

    meanSensors();

    int32_t ax_offset = mean_accel.x;
    int32_t ay_offset = mean_accel.y;
    int32_t az_offset = mean_accel.z - ACCEL_ONE_G;

    imu.setAccelOffset({static_cast<int16_t>(ax_offset),
                        static_cast<int16_t>(ay_offset),
                        static_cast<int16_t>(az_offset)});

    Serial.printf("Accelerometer calibration complete!\n");
    Serial.printf("Add this to your setup():\n");
    Serial.printf("\timu.setAccelOffset({%d, %d, %d});\n", ax_offset, ay_offset, az_offset);
}

/**
 * @brief Streams live formatted data to the terminal to verify the calibration.
 */
void verifyReadings()
{
    Serial.printf("\n--- Verifying Live Sensor Data ---\n");
    Serial.printf("Send any character in the Serial Monitor to stop.\n\n");

    clearSerialBuffer();

    while (!Serial.available())
    {
        imu.update();

        MPU6050::Axis3<float> accel = imu.getAcceleration();
        MPU6050::Axis3<float> gyro = imu.getGyroscope();

        // Print formatted data aligned in columns
        Serial.printf("Accel (G):  X: %6.3f  Y: %6.3f  Z: %6.3f   |   Gyro (dps):  X: %6.2f  Y: %6.2f  Z: %6.2f\n",
                      accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);

        delay(100);
    }

    clearSerialBuffer();
    Serial.printf("Live data stream stopped.\n");
}

/**
 * @brief Safely clears the serial buffer of dangling newline characters.
 */
void clearSerialBuffer()
{
    while (Serial.available())
    {
        Serial.read();
        delay(2);
    }
}