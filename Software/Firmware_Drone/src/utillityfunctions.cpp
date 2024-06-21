#include "header.h"

void print_batVolt();
void printMemoryUse();
void battery_uv_protection();

/*Reads Serial Commands wich start with /*/
void serial_commands()
{
    if (Serial.available())
    {
        char buffer_comand = Serial.read();
        if (buffer_comand == '/')
        {
            buffer_comand = Serial.read();
            switch (buffer_comand)
            {
            case '?':
            {
                debugln("<help>");
                debugln("/b|Returns Battery Voltage. If followed by a number (z.B.4) it reads out battery every 4 [ms]. Set to -1 to disable this freature");
                debugln("/m[number][duty in %]|Motorsteuerung: Give Speed in Percent (for example 8) and add one (so 9). Set to 1 to turn Motor off");
                debugln("/f|Frequncyrange of the Motors: Give Frequency in Percent and subtract one Percent");
                debugln("/w|I2C Scanner");
                debugln("r/reboot.");
                debugln("/t|radio recieve");
                debugln("/a prints mpu values. /ae enable autoprinting, /ad disable autoprinting /ac to calibrate");
                break;
            }
            case 'm':
            {
                int motor_number = -1;
                float duty = -1;
                debugf_yellow("Command Motorsteuerung recieved.\n");
                if (Serial.available() > 2)
                {
                    motor_number = Serial.parseInt(SKIP_WHITESPACE);
                    if (motor_number == -1)
                    {
                        set_motor(PIN_MOT0, 0);
                        set_motor(PIN_MOT1, 0);
                        set_motor(PIN_MOT2, 0);
                        set_motor(PIN_MOT3, 0);
                        break;
                    }
                    if (motor_number == 4)
                    {
                        const unsigned int STRENGTH = ANALOG_WRITE_RANGE * 0.005;
                        const unsigned int DURATION = 200;
                        set_motor(PIN_MOT0, STRENGTH);
                        delay(DURATION);
                        set_motor(PIN_MOT0, 0);
                        delay(DURATION);
                        set_motor(PIN_MOT1, STRENGTH);
                        delay(DURATION);
                        set_motor(PIN_MOT1, 0);
                        delay(DURATION);
                        set_motor(PIN_MOT2, STRENGTH);
                        delay(DURATION);
                        set_motor(PIN_MOT2, 0);
                        delay(DURATION);
                        set_motor(PIN_MOT3, STRENGTH);
                        delay(DURATION);
                        set_motor(PIN_MOT3, 0);

                        break;
                    }

                    if (Serial.available() > 2)
                    {
                        duty = Serial.parseFloat(SKIP_WHITESPACE) / 100.0;
                        set_motor(motor_number, (uint)(duty * ANALOG_WRITE_RANGE));
                    }

                    else
                    {
                        debugln("No pwm value given");
                    }
                }
                else
                {
                    debugln("No Motor Number given");
                }
                break;
            }
            case 'f':
            {
                debugln("Command Frequenzänderung recieved\n");
                int buffer = Serial.parseInt(SKIP_WHITESPACE);
                if (!buffer)
                {
                    debug("Frequenxy need to be bigger than 0 ");
                    break;
                }
                debug("Frequenxy set to: ");
                debugln(buffer);
                analogWriteFreq(buffer);
                break;
            }
            case 'b':
            {
                debug("BatteryVoltage: ");
                debugln(3.3 * 2 * (analogRead(PIN_BATT) / 1024.0));
                int buffer = Serial.parseInt(SKIP_WHITESPACE);
                switch (buffer)
                {
                case 0:
                    break;
                case -1:
                    BatReadIntervall = -1;
                    debugln("Stops automatic Battery readout");
                    break;
                default:
                    BatReadIntervall = buffer;
                    debug("Automatic Battery readout every ");
                    debug(buffer);
                    debugln("ms");
                    break;
                }
                break;
            }
            case 's':
            {
                debugln("Entering Sleep");
                int buffer = Serial.parseInt(SKIP_WHITESPACE) * 1000;
                // sleep_config_set_default();
                // // Set wake-up source to time
                // sleep_configure_wakeup(
                //     rtc_time_ms() + 5000, // Wake up after 5 seconds
                //     0,                    // Don't repeat
                //     false                 // Wake up on time, not alarm
                // );
                // // Enter deep sleep mode
                // sleep_manager_deep_sleep();
                debugln("Awake Again");
                break;
            }
            case 'u':
            {
                printMemoryUse();
                break;
            }
            case 'w':
            {
                scan_wire();
                break;
            }
            case 'a':
            {
                char buffer = Serial.read();
                if (buffer == 'e')
                {
                    mpu_print_enable = 1;
                    debugln("Autoprint of MPU info enabled");
                }
                else if (buffer == 'd')
                {
                    mpu_print_enable = 0;
                    debugln("Autoprint of MPU info disabled");
                }
                else if (buffer == 'c')
                {
                    mpu.calcOffsets();
                }
                else
                {
                    mpu_print();
                }
                break;
            }
            case 't':
            {
                debugln("<Start recieveing packets>");
                while (1)
                {
                    radio_recieve_buffer();
                    if (Serial.available())
                    {
                        if (Serial.read() == 'q')
                        {
                            break;
                        }
                    }
                }
                debugln("<Stop recieveing packets>");
                break;
            }
            case 'r':
            {
                int buffer = -1;
                if (Serial.available() > 3)
                {
                    buffer = Serial.parseInt(SKIP_WHITESPACE);
                }

                if (buffer == 1)
                {
                    debugln("-Reboot in Boot Mode-");
                    rp2040.rebootToBootloader();
                    break;
                }
                else
                {
                    debugln("-Reboot-");
                    rp2040.reboot();
                    break;
                }
            }
            case 'p':
            {
                debugln("change PID");
                kp_x = Serial.parseFloat(SKIP_WHITESPACE);
                ki_x = Serial.parseFloat(SKIP_WHITESPACE);
                kd_x = Serial.parseFloat(SKIP_WHITESPACE);

                break;
            }
            default:
            {
                debugln("Command invalid\n");
                break;
            }
            }
        }
    }
}

int BatReadIntervall = -1;
unsigned int bat_timestamp = 0;
/*prints battery voltage every BatReadIntervall milliseconds if it is not set to -1*/
void print_batVolt()
{
    if (BatReadIntervall != -1)
    {
        if (millis() > bat_timestamp)
        {
            // debug("baVol: ");
            debugln(3.3 * 2 * (analogRead(PIN_BATT) / 1024.0));
            bat_timestamp = millis() + BatReadIntervall;
        }
    }
}

void printMemoryUse()
{
    int totalHeap = rp2040.getTotalHeap();
    debug("-TotalHeap: ");
    debug(rp2040.getTotalHeap());
    debugln(" bytes");

    debug("-FreeHeap: ");
    debug(rp2040.getFreeHeap());
    debug(" bytes| ");
    debug((float)rp2040.getFreeHeap() / totalHeap);
    debugln(" percent");

    debug("-UsedHeap: ");
    debug(rp2040.getUsedHeap());
    debug(" bytes| ");
    debug((float)rp2040.getUsedHeap() / totalHeap);
    debugln(" percent");
}

void battery_uv_protection()
{
    float batvolt = 3.3 * 2 * (analogRead(PIN_BATT) / 1096.0);
    if (BATVOLTMIN > batvolt)
    {
        debug("BatVotage: ");
        debugln(batvolt);
        debugln("BatteryVoltage under Threshold Deep Sleep Initialised");
        for (int i = 0; i < 30; i++)
        {
            pinMode(i, INPUT);
            digitalWrite(i, LOW);
        }
    }
}