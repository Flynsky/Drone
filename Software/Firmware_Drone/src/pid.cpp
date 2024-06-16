#include "header.h"
#include <MPU6050_light.h>

const int ANALOG_WRITE_RANGE = 4096;
const float REGEL_MAX = 10000;
const float X_SETPOINT = 0;
const float Y_SETPOINT = 0;

static float pid_x_update();
static float pid_y_update();
static void printPid(float p, float i, float d, float regl);
void set_motor(uint8_t Pin, float duty);

char pid_init = 0;
void pid_setup()
{
    mpu_setup();
    pinMode(PIN_MOT0, OUTPUT);
    pinMode(PIN_MOT1, OUTPUT);
    pinMode(PIN_MOT2, OUTPUT);
    pinMode(PIN_MOT3, OUTPUT);
    analogWriteRange(ANALOG_WRITE_RANGE);
    pid_init++;
}

/**
 * Updates the pid controll of all achsis of the drone and changes the motor dutycycle accordingly
 */
void pid_update()
{
    if (!pid_init)
    {
        pid_setup();
    }
    float pidx = pid_x_update();
    pidx = (ANALOG_WRITE_RANGE * pidx / REGEL_MAX);
    float pidy = pid_y_update();
    pidy = ANALOG_WRITE_RANGE * pidy / REGEL_MAX;

    // set_motor(0, -pidx);
    set_motor(1, pidx);
    set_motor(2, pidx);
    // set_motor(3, -pidx);
}

float kp_x = 0; // /p 13 2 10
float ki_x = 0;
float kd_x = 0;
const float IBUF_X_MINMAX = 1000000;
/**
 * Updates the pid controll of the X achsis of the Drone
 *@return:float regl
 */
float pid_x_update()
{
    mpu.update();

    static unsigned int timebuff = millis();
    static float ibuf_x = 0;

    float error = X_SETPOINT - mpu.getAngleX();

    unsigned int Ti = millis() - timebuff; // Time Constant

    /*-------------p------------*/
    float p = kp_x * error * Ti;

    /*-------------i------------*/
    ibuf_x += error * Ti;
    float i = ki_x * ibuf_x;

    if (ibuf_x > IBUF_X_MINMAX)
    {
        ibuf_x = IBUF_X_MINMAX;
    }
    else if (ibuf_x < -1 * IBUF_X_MINMAX)
    {
        ibuf_x = -1 * IBUF_X_MINMAX;
    }

    /*-------------d------------*/
    float d = kd_x * -1 * mpu.getGyroX();

    /*------------calc----------*/

    float regl = p + i + d;
    printPid(p, i, d, regl);
    timebuff = millis();
    return regl;
}

float kp_y = 0; // /p 13 2 10
float ki_y = 0;
float kd_y = 0;
const float IBUF_Y_MINMAX = 1000000;
/**
 * Updates the pid controll of the X achsis of the Drone
 *@return:float regl
 */
float pid_y_update()
{
    mpu.update();

    static unsigned int timebuff = millis();
    static float ibuf_y = 0;

    float error = Y_SETPOINT - mpu.getAngleY();

    unsigned int Ti = millis() - timebuff; // Time Constant

    /*-------------p------------*/
    float p = kp_y * error * Ti;

    /*-------------i------------*/
    ibuf_y += error * Ti;
    float i = ki_y * ibuf_y;

    if (ibuf_y > IBUF_Y_MINMAX)
    {
        ibuf_y = IBUF_Y_MINMAX;
    }
    else if (ibuf_y < -1 * IBUF_Y_MINMAX)
    {
        ibuf_y = -1 * IBUF_Y_MINMAX;
    }

    /*-------------d------------*/
    float d = kd_y * -1 * mpu.getGyroY();

    /*------------calc----------*/

    float regl = p + i + d;
    printPid(p, i, d, regl);
    timebuff = millis();
    return regl;
}

void printPid(float p, float i, float d, float regl)
{
    if (mpu_print_enable)
    {
        debug(">p:");
        debug(p);
        debugln();
        debug(">i:");
        debug(i);
        debugln();
        debug(">d:");
        debug(d);
        debugln();
        debug(">regl:");
        debug(regl);
        debugln();
    }
}

#define MOTORMAX 0.4 // max motor power in percent/100
void set_motor(uint8_t Motor_nr, float duty)
{
    // selected
    switch (Motor_nr)
    {
    case 0:
        Motor_nr = PIN_MOT0;
        break;
    case 1:
        Motor_nr = PIN_MOT1;
        break;
    case 2:
        Motor_nr = PIN_MOT2;
        break;
    case 3:
        Motor_nr = PIN_MOT3;
        break;
    default:
        return;
    }

    if (duty < 0)
    {
        digitalWrite(Motor_nr, 0);
        return;
    }

    if (duty < MOTORMAX)
    {
        analogWrite(Motor_nr, (int)(duty * ANALOG_WRITE_RANGE));
        // Serial.printf("set motor pin %i at %i\n", Motor_nr, (int)(duty * ANALOG_WRITE_RANGE));
    }
    else
    {
        debugln("Motor speed over limit.");
    }
}