#pragma once

#include <pthread.h>

#include "tty.h"
#include "serial.h"
#include "interface.h"
#include "logger.h"
#include "buffer.h"


#define READ_BUFFER_SIZE 100
#define WRITE_MAX_N_CHARS 4
#define DEVICE "/dev/ttyACM0"


typedef struct {
    int running;
    int serial_port;
    FILE * file;
} AppState;

typedef struct {
    AppState * app_state;
    GtkEntryBuffer * buffer;
    char * copy;
} SendEventParams;

typedef struct {
    AppState * app_state;
    GtkTextBuffer * buffer;
    char * input_buffer;
    char * copy;
} ActivateParams;


typedef struct {
    AppState * app_state;
    GtkTextBuffer * buffer;
    char * msg;
    pthread_mutex_t * mutex_read;
    Buffer * buf;
} SerialTask;