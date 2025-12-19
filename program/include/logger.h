#pragma once

#include <stdio.h>

#define TIME_BUFFER_SIZE 100
#define LOG_FILE "file.log"
#define SEND_TXT "[Enviado]"
#define RECEIVED_TXT "[Recibido]"

enum Origin {
    SEND,
    RECEIVED,
    OTHER
};

void write_log(char *, FILE *); 
void write_serial_log(char *, FILE *, enum Origin);