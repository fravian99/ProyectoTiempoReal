#include <time.h>

#include "../include/logger.h"
#include "../include/program.h"

void get_date_buffer(char * time_buffer, int buffer_size) {
    struct tm  * actual_time;
    time_t current_time = time(0);
    actual_time = localtime(&current_time);
    strftime(time_buffer, buffer_size, "%d-%m-%Y %H:%M:%S", actual_time);
}


void write_serial_log(char * msg, FILE * file, enum Origin origin) {
   char time_buffer[TIME_BUFFER_SIZE];
    get_date_buffer(time_buffer, TIME_BUFFER_SIZE);
    switch (origin)
    {
    case SEND:
        fprintf(file, "%s %s: %s\n", time_buffer, SEND_TXT, msg);
        break;
    case RECEIVED:
        fprintf(file, "%s %s: %s\n", time_buffer, RECEIVED_TXT, msg);
        break;
    default:
        fprintf(file, "%s: %s\n", time_buffer, msg);
        break;
    }
    fflush(file);
}

void write_log(char * msg, FILE * file) {
    write_serial_log(msg, file, OTHER);
}

