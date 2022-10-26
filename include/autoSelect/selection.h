#pragma once

#include <string>

// selector configuration
#define HUE 360
#define AUTONS "Do Nothing", \
               "Left WP",    \
               "Right WP"
#define DEFAULT 1

namespace selector
{

    extern int auton;
    const char *b[] = {AUTONS, ""};
    void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
