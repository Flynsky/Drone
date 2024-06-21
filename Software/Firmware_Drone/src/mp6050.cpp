/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

/*https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/*/
#include "header.h"
#include <Wire.h>
#include <MPU6050_light.h>
#define TELEPLOT_ENABLE 1

MPU6050 mpu(Wire1);

char mpu_init = 0;

void mpu_setup()
{
    if (mpu_init)
    {
        debugln("mpu already initialised\n");
        return;
    }
    Wire1.setClock(400000);
    Wire1.setSCL(PIN_MPU_SCL);
    Wire1.setSDA(PIN_MPU_SDA);
    Wire1.begin();
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    if (status != 0) // could not connect to MPU6050
    {
        mpu_init = 0;
        Serial.println("Could not connect to MPU6050.");
        rp2040.reboot();
        return;
    }
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true, true); // gyro and accelero
    Serial.println("Done!\n");
    mpu_init++;
}

void mpu_print()
{
    if (!mpu_init)
    {
        mpu_setup();
    }
    // mpu.update();

#if TELEPLOT_ENABLE == 1
    // Serial.print(F(">temp: "));
    // Serial.println(mpu.getTemp());

    // Serial.print(F(">ACCELERO  X: "));
    // Serial.println(mpu.getAccX());
    // Serial.print(">ACCELERO Y: ");
    // Serial.println(mpu.getAccY());
    // Serial.print(">ACCELERO Z: ");
    // Serial.println(mpu.getAccZ());

    // Serial.print(F(">GYRO X: "));
    // Serial.println(mpu.getGyroX());
    // Serial.print(">GYRO Y: ");
    // Serial.println(mpu.getGyroY());
    // Serial.print(">GYRO Z: ");
    // Serial.println(mpu.getGyroZ());

    // Serial.print(F(">ACC ANGLE X: "));
    // Serial.println(mpu.getAccAngleX());
    // Serial.print(">ACC ANGLE Y: ");
    // Serial.println(mpu.getAccAngleY());

    Serial.print(F(">ANGLE X: "));
    Serial.println(mpu.getAngleX());
    Serial.print(">ANGLE Y: ");
    Serial.println(mpu.getAngleY());
    Serial.print(">ANGLE Z: ");
    Serial.println(mpu.getAngleZ());

#else

    Serial.print(F("TEMPERATURE: "));
    Serial.println(mpu.getTemp());
    Serial.print(F("ACCELERO  X: "));
    Serial.print(mpu.getAccX());
    Serial.print("\nY: ");
    Serial.print(mpu.getAccY());
    Serial.print("\nZ: ");
    Serial.println(mpu.getAccZ());

    Serial.print(F("GYRO      X: "));
    Serial.print(mpu.getGyroX());
    Serial.print("\nY: ");
    Serial.print(mpu.getGyroY());
    Serial.print("\nZ: ");
    Serial.println(mpu.getGyroZ());

    Serial.print(F("ACC ANGLE X: "));
    Serial.print(mpu.getAccAngleX());
    Serial.print("\nY: ");
    Serial.println(mpu.getAccAngleY());

    Serial.print(F("ANGLE     X: "));
    Serial.print(mpu.getAngleX());
    Serial.print("\nY: ");
    Serial.print(mpu.getAngleY());
    Serial.print("\nZ: ");
    Serial.println(mpu.getAngleZ());
    Serial.println(F("=====================================================\n"));

#endif
}