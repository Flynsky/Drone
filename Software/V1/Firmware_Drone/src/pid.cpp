#include "header.h"
#include <MPU6050_light.h>

char mpu_print_enable = 0;
const int REGEL_MAX = ANALOG_WRITE_RANGE;
const unsigned int REGEL_FREQ = 1000;

const float X_SETPOINT = 0;
const float Y_SETPOINT = 0;

float kp_x = 0; // p 65 5 1
float ki_x = 0;
float kd_x = 0;
const float IBUF_X_MINMAX = ANALOG_WRITE_RANGE * 0.1;

int pid_x_update();
int pid_y_update();
static void printPid(float p, float i, float d, float regl);

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
    mpu.update();

    if (millis() >= last_regel)
    {
        last_regel = millis() + (1000 / REGEL_FREQ);
        if (!pid_init)
        {
            pid_setup();
        }

        int pidx = pid_x_update();
        int pidy = pid_y_update();

        unsigned pidx_negativ = 0;
        unsigned pidx_positiv = 0;
        if (pidx < 0)
        {
            pidx_negativ = pidx * -1;
        }
        else
        {
            pidx_positiv = pidx;
        }

        unsigned pidy_negativ = 0;
        unsigned pidy_positiv = 0;
        if (pidy < 0)
        {
            pidy_negativ = pidy * -1;
        }
        else
        {
            pidy_positiv = pidy;
        }

        // debugf_blue("pidx: %i, pidx_pos:%u,pidx_ned:%u\n",pidx,pidx_positiv,pidx_negativ);
        // debugf_yellow("pidy: %i, pidy_pos:%u,pidy_ned:%u\n",pidy,pidy_positiv,pidy_negativ);

        set_motor(PIN_MOT1, pidx_positiv + pidy_negativ);
        set_motor(PIN_MOT2, pidx_positiv + pidy_positiv);
        set_motor(PIN_MOT0, pidx_negativ+pidy_negativ);
        set_motor(PIN_MOT3, pidx_negativ+pidy_positiv);
    }
}

/**
 * Updates the pid controll of the X achsis of the Drone
 *@return:float regl
 */
int pid_x_update()
{
    static unsigned int time_last = millis();
    static float ibuf_x = 0;

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
    float d = kd_x * -mpu.getGyroX() * 0.01;

    /*------------calc----------*/

    int regl = (p + i + d);
    if (regl > REGEL_MAX)
    {
        regl = REGEL_MAX;
    }
    else
    {
        if (regl < (-REGEL_MAX))
        {
            regl = -REGEL_MAX;
        }
    }
    printPid(p, i, d, regl);

    time_last = millis();
    return regl;
}

/**
 * Updates the pid controll of the X achsis of the Drone
 *@return:float regl
 */
int pid_y_update()
{
    static unsigned int time_last = millis();
    static float ibuf_y = 0;

    float error = Y_SETPOINT - mpu.getAngleY();

    unsigned int Ti = millis() - time_last; // Time Constant

    /*-------------p------------*/
    float p = kp_x * error;

    /*-------------i------------*/
    ibuf_y += error * Ti * 0.001;
    float i = ki_x * ibuf_y;

    if (ibuf_y > IBUF_X_MINMAX)
    {
        ibuf_y = IBUF_X_MINMAX;
    }
    else if (ibuf_y < -1 * IBUF_X_MINMAX)
    {
        ibuf_y = -1 * IBUF_X_MINMAX;
    }

    /*-------------d------------*/
    float d = kd_x * -mpu.getGyroY() * 0.01;

    /*------------calc----------*/

    int regl = (int)(p + i + d);

    if (regl > REGEL_MAX)
    {
        regl = REGEL_MAX;
    }
    else
    {
        if (regl < (-REGEL_MAX))
        {
            regl = -REGEL_MAX;
        }
    }

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

const unsigned int MOTORMAX = ANALOG_WRITE_RANGE+1; // max motor power in percent/100

void set_motor(uint8_t Motor_nr, unsigned int analog_write_value)
{
    if (analog_write_value < MOTORMAX)
    {
        analogWrite(Motor_nr, analog_write_value);
    }
    else
    {
        // debugln("Motor speed over limit.");
    }
}