#pragma once

void serial_write(int serial_port, const char* msg);
int serial_read(int serial_port, char* buffer, int size);
int serial_until_new_line(int serial_port, char* buffer, int size, int *running);