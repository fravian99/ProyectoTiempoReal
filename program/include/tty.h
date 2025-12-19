
#pragma once

#include <termios.h>

#define B_RATE B115200

void config_flags(struct termios*);
int config_tty(struct termios*, int, int);