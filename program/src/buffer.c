#include <stdio.h>
#include "../include/buffer.h"

Buffer buffer_init(char * inner_buffer, int size) {
    Buffer buffer = {
        .inner_buffer = inner_buffer,
        .size = size,
        .read_ptr = 0,
        .write_ptr = 0
    };
    return buffer;
}

int buffer_is_full(Buffer * buffer) {
    return (((buffer->write_ptr + 1) % buffer->size) == buffer->read_ptr);
}

int buffer_is_empty(Buffer * buffer) {
    return (buffer->read_ptr == buffer->write_ptr);
}

int buffer_enque(Buffer* buffer, char c) {
    int is_full = buffer_is_full(buffer);
    if(!is_full) {
        buffer->inner_buffer[buffer->write_ptr] = c;
        buffer->write_ptr++;
        buffer->write_ptr %= buffer->size;
    }
    return is_full;
}

int buffer_deque(Buffer* buffer, char* value) {
    int is_empty =  buffer_is_empty(buffer);
    if(!is_empty) {
        *value = buffer->inner_buffer[buffer->read_ptr];
        buffer->read_ptr++;
        buffer->read_ptr %= buffer->size;
    }
    return(is_empty);
}

int buffer_insert_element(Buffer * buffer, char * buffer_ptr) {
    int i, is_full;
    char actual, previous;
    i = 0;
    is_full = buffer_is_full(buffer);
    previous = '\n';
    while (buffer_ptr[i] != '\0' && !is_full) {
        actual = buffer_ptr[i];
        actual = actual == '\0' ? '\n' : actual;
        if (previous != '\n' || actual != '\n') {
            buffer_enque(buffer, actual);
        }
        i++;
        is_full = buffer_is_full(buffer);
        previous = actual;
    }
    return is_full;
}

int buffer_cpy_element(Buffer * buffer, char * value) {
    int i, is_empty;
    char c = '\0';
    is_empty = buffer_is_empty(buffer);
    i = 0;
    while(!is_empty && !(c == '\n' && i>0 )) {
        buffer_deque(buffer, &c);
        value[i] = c;
        is_empty = buffer_is_empty(buffer);
        i++;
    }
    if (value[i] == '\n') {
       value[i] = '\0';
    }

    if (i > 0 && value[i-1] == '\n') {
        value[i-1] = '\0';
    }
    return is_empty;
}
