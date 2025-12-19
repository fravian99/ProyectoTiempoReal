#include <stdio.h>
#include <fcntl.h>

#include "../include/tty.h"

void config_flags(struct termios* tty_ptr) {
    struct termios tty = *tty_ptr;
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                     // Disable break processing
    tty.c_lflag = 0;                            // No signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                            // No remapping, no delays
    tty.c_cc[VMIN] = 0;                         // Read blocks
    tty.c_cc[VTIME] = 1;                        // Read timeout (tenths of a second)
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // Shut off XON/XOFF ctrl
    tty.c_cflag |= (CLOCAL | CREAD);            // Ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);          // Disable parity
    tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                    // Disable RTS/CTS flow control
    *tty_ptr = tty;
}

int config_tty(struct termios* tty_ptr, int serial_port, int b_rate) {
    struct termios tty = *tty_ptr;
    if (tcgetattr(serial_port, &tty) != 0)
    {
        perror("Failed to get attributes");
        return 1;
    }
    // Configure the serial port
    cfsetispeed(&tty, b_rate);
    cfsetospeed(&tty, b_rate);
    config_flags(&tty);
   
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
    {
        perror("Failed to set attributes");
        return 1;
    }
    *tty_ptr = tty;
    return 0;
}
