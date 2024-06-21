#include "header.h"
#include <MPU6050_light.h>

const float REGEL_MAX = ANALOG_WRITE_RANGE;
const unsigned int REGEL_FREQ = 1000;

const float X_SETPOINT = 0;
const float Y_SETPOINT = 0;

static float pid_x_update();
static float pid_y_update();
static void printPid(float p, float i, float d, float regl);
void set_motor(uint8_t Pin, unsigned int analog_write_value);

char pid_init = 0;
void pid_setup()
{
    mpu_setup();
    pinMode(PIN_MOT0, OUTPUT);
    pinMode(PIN_MOT1, OUTPUT);
    pinMode(PIN_MOT2, OUTPUT);
    pinMode(PIN_MOT3, OUTPUT);
    analogWriteFreq(ANALOG_WRITE_FREQ);
    analogWriteRange(ANALOG_WRITE_RANGE);
    pid_init++;
}

/**
 * Updates the pid controll of all achsis of the drone and changes the motor dutycycle accordingly
 */
void pid_update()
{
    static unsigned int last_regel = millis();
    if (millis() >= last_regel)
    {
        last_regel = millis() + (1000 / REGEL_FREQ);

        if (!pid_init)
        {
            pid_setup();
        }

        float pidx = pid_x_update();
        float pidy = pid_y_update();

        // set_motor(0, -pidx);
        set_motor(1, pidx);
        set_motor(2, pidx);
        // set_motor(3, -pidx);
    }
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
    static unsigned int time_last = millis();
    static float ibuf_x = 0;

    mpu.update();

    float error = X_SETPOINT - mpu.getAngleX();

    unsigned int Ti = millis() - time_last; // Time from last messurement to the current

    /*-------------p------------*/
    float p = kp_x * error;

    /*-------------i------------*/
    ibuf_x += error * Ti * 0.001;
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
    float d = kd_x * -mpu.getGyroX();

    /*------------calc----------*/

    float regl = p + i + d;
    if (regl > REGEL_MAX)
    {
        regl = REGEL_MAX;
    }
    printPid(p, i, d, regl);

    time_last = millis();
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

    static unsigned int time_last = millis();
    static float ibuf_y = 0;

    float error = Y_SETPOINT - mpu.getAngleY();

    unsigned int Ti = millis() - time_last; // Time Constant

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
    time_last = millis();
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

const unsigned int MOTORMAX = (ANALOG_WRITE_RANGE * 0.2); // max motor power in percent/100

void set_motor(uint8_t Motor_nr, unsigned int analog_write_value)
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

    if (analog_write_value < MOTORMAX)
    {
        analogWrite(Motor_nr, analog_write_value);
    }
    else
    {
        debugln("Motor speed over limit.");
    }
}