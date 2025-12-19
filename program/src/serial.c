#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../include/serial.h"

void serial_write(int serial_port, const char* msg) {
    write(serial_port, msg, strlen(msg));
}

int serial_read(int serial_port, char* buffer, int size) {
     // Read from the serial port
    memset(buffer, 0, size);
    int num_bytes = read(serial_port, buffer, size);
    if (num_bytes < 0)
    {
        perror("Failed to read from the serial port");
    }
    else
    {
        //printf("Read %d bytes: |||%s|||\n", num_bytes, buffer);
    }
    return num_bytes;
}

int serial_until_new_line(int serial_port, char* buffer, int size, int * running) {
    int flag, n_bytes, actual_size;
    char * buffer2;
    int iter = 0;
    flag = 0;
    actual_size = 0;
    buffer2 =  buffer;
    while(!flag && *running && actual_size < size) {
        serial_read(serial_port, buffer2, size - actual_size);
        n_bytes = strlen(buffer2);
        if (n_bytes > 0) {
            actual_size += n_bytes;   
            flag = buffer[actual_size - 2] == '\n' ;
            iter++;
            buffer2 =  buffer + actual_size;
        }
    }
    return actual_size;
}