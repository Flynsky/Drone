/*Main programm coordinating the different sections*/
#include "header.h"

void print_startup_message();

//------------------------core1--------------------------------
void setup()
{
    pinMode(PIN_BATT, INPUT);
    print_startup_message();
    pinMode(23, 1);
    digitalWrite(23,1);
}

void loop()
{
    // Radio_recieve();
    serial_commands();
    print_batVolt();
    battery_uv_protection();
}

//-------------------------core2------------------------
void setup1()
{
    // debugln("----------Main is running-C1-------------------");
}

void loop1()
{
    if(kp_x)
    {
        pid_update();
    }

    // delay(10);
}

void print_startup_message()
{
#if DEBUG == 1
    Serial.begin(9600);
    while (!Serial)
    {
    }
    delay(100);
    debugln(" ");
    debugf_yellow("<<[Drone.ino] is running on Chip %i>>\nCore %i|Freq %.2f", rp2040.getChipID(), rp2040.cpuid(), rp2040.f_cpu() / 1000000.0);
    if (watchdog_caused_reboot())
    {
        debugf_yellow("MHz|Watchdog Reset\n");
    }
    else
    {
        debugf_yellow("MHz\n");
    }

    debugf_blue("⠀     (\\__/)\n      (•ㅅ•)      Don't talk to\n   ＿ノヽ ノ＼＿      me or my son\n`/　`/ ⌒Ｙ⌒ Ｙ  ヽ     ever again.\n( 　(三ヽ人　 /　  |\n|　ﾉ⌒＼ ￣￣ヽ   ノ\n ヽ＿＿＿＞､＿_／\n    ｜( 王 ﾉ〈   (\\__/)\n    /ﾐ`ー―彡\\   (•ㅅ•)\n   / ╰    ╯ \\  /    \\&gt;\n");

    debugf_yellow("\"/?\" for help\n");
    // rp2040.enableDoubleResetBootloader();
#endif
}