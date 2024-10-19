#ifndef DEBUG_IN_COLOR_H
#define DEBUG_IN_COLOR_H

#define DEBUG 1         /*actrivates debug debug statements*/
#define COLOUR_SERIAL 1 /*activates Serial.printing with color*/

/*-------Debug Features-------------------*/
#if DEBUG == 1
#define debug(...) Serial.print(__VA_ARGS__)
#define debugln(...) Serial.println(__VA_ARGS__)
#define debugf(...) Serial.printf(__VA_ARGS__)
#define MESSURETIME_START        \
    unsigned long tb = 0;        \
    unsigned long ta = micros(); \
    debugf_magenta(".");
#define MESSURETIME_STOP \
    tb = micros();       \
    ta = tb - ta;        \
    debugf_magenta("Time: %.2fs|%.2fms|%ius\n", ta / 1000000.0, ta / 1000.0, ta);
#else
#define debug(...)
#define debugln(...)
#define debugf(...)
#define MESSURETIME_START
#define MESSURETIME_STOP
#endif

/*-------Serial printing colours------------*/
#if COLOUR_SERIAL == 1
#define debugf_yellow(...)   \
    debug("\033[33m");       \
    debugf(__VA_ARGS__);     \
    debug("\033[0m");
// / debug("\033[1m");

#define debugf_black(...) \
    debug("\033[30m");    \
    debugf(__VA_ARGS__);  \
    debug("\033[0m");

#define debugf_red(...)  \
    debug("\033[31m");   \
    debug("\033[7m");    \
    debugf(__VA_ARGS__); \
    debug("\033[0m");

#define debugf_green(...) \
    debug("\033[32m");    \
    debugf(__VA_ARGS__);  \
    debug("\033[0m");

#define debugf_blue(...) \
    debug("\033[34m");   \
    debugf(__VA_ARGS__); \
    debug("\033[0m");

#define debugf_magenta(...) \
    debug("\033[35m");      \
    debugf(__VA_ARGS__);    \
    debug("\033[0m");

#define debugf_cyan(...) \
    debug("\033[36m");   \
    debugf(__VA_ARGS__); \
    debug("\033[0m");

#define debugf_white(...) \
    debug("\033[37m");    \
    debugf(__VA_ARGS__);  \
    debug("\033[0m");

// You can keep the original definitions for color setting macros without changes.
#define SET_COLOUR_BLACK debug("\033[30m");
#define SET_COLOUR_RED debug("\033[31m");
#define SET_COLOUR_GREEN debug("\033[32m");
#define SET_COLOUR_YELLOW debug("\033[33m");
#define SET_COLOUR_BLUE debug("\033[34m");
#define SET_COLOUR_MAGENTA debug("\033[35m");
#define SET_COLOUR_CYAN debug("\033[36m");
#define SET_COLOUR_WHITE debug("\033[37m");
#define SET_COLOUR_RESET debug("\033[0m");
#else
#define debugf_yellow(...) debugf(__VA_ARGS__)
#define debugf_black(...) debugf(__VA_ARGS__)
#define debugf_red(...) debugf(__VA_ARGS__)
#define debugf_green(...) debugf(__VA_ARGS__)
#define debugf_blue(...) debugf(__VA_ARGS__)
#define debugf_magenta(...) debugf(__VA_ARGS__)
#define debugf_cyan(...) debugf(__VA_ARGS__)
#define debugf_white(...) debugf(__VA_ARGS__)

#define SET_COLOUR_BLACK
#define SET_COLOUR_RED
#define SET_COLOUR_GREEN
#define SET_COLOUR_YELLOW
#define SET_COLOUR_BLUE
#define SET_COLOUR_MAGENTA
#define SET_COLOUR_CYAN
#define SET_COLOUR_WHITE
#define SET_COLOUR_RESET
#endif
#endif