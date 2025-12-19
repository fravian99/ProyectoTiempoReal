#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>

#include "../include/program.h"

void send_event(GtkWidget *, gpointer);
static void activate(GtkApplication *, gpointer);

void insert_text_to_buffer(GtkTextBuffer * text_buffer, char * msg) {
    GtkTextIter end, start;
    gtk_text_buffer_get_start_iter(text_buffer, &start);
    gtk_text_buffer_get_end_iter(text_buffer, &end);
    gtk_text_buffer_delete(text_buffer, &start, &end);
    gtk_text_buffer_insert(text_buffer, &end, msg, -1);
}

gboolean idle_append_text(gpointer data) {
    char * msg;
    FILE * file;
    int locked;
    Buffer * buffer;
    GtkTextBuffer * text_buffer;
    SerialTask* serial_task = (SerialTask *)data;
    buffer = serial_task->buf;
    while ((locked = pthread_mutex_trylock(serial_task->mutex_read)) == 0 && serial_task->app_state->running);
    if (locked) {
        if (serial_task->app_state->running) {
            file = serial_task->app_state->file;
            msg =  serial_task->msg;
            text_buffer = serial_task->buffer;
            while(!buffer_is_empty(buffer)) {
                memset(msg, 0, READ_BUFFER_SIZE);
                buffer_cpy_element(buffer, msg);     
                insert_text_to_buffer(text_buffer, msg);
                write_serial_log(msg, file, RECEIVED);
            }
        }
        locked = 0;
        pthread_mutex_unlock(serial_task->mutex_read);
    }

    return FALSE; // Ejecutar solo una vez
}


void * task_read(void * arguments) {
    int num, serial_port;
    char buffer[READ_BUFFER_SIZE] = {0};
    int * running;
    int locked;
    SerialTask * serial_task = ((SerialTask *) arguments);
    AppState * app_state = serial_task->app_state; 
    serial_port = serial_task->app_state->serial_port;
    running = &app_state->running;

    while (*running)
    {
        num = serial_until_new_line(serial_port, buffer, READ_BUFFER_SIZE, running);
        if (num > 0) {
            while ((locked = pthread_mutex_trylock(serial_task->mutex_read)) == 0 && serial_task->app_state->running);
            if (locked) {
                buffer_insert_element(serial_task->buf, buffer);
                pthread_mutex_unlock(serial_task->mutex_read);
                locked = 0;
            }
            if (*running) {
                g_idle_add(idle_append_text, serial_task);
            }
        }
    }
    pthread_exit(NULL);
}


int main(int argc, char **argv)
{
    pthread_t thread_read;
    pthread_mutex_t mutex;

    GtkApplication *app;
    SerialTask serial_task;
    ActivateParams activate_params;

    Buffer buf;

    char msg[READ_BUFFER_SIZE] = {0};
    char input_buffer[WRITE_MAX_N_CHARS] = {0};
    char copy[WRITE_MAX_N_CHARS + 2] = {0};
    char buf_ptr[READ_BUFFER_SIZE * 5] = {0};
    int status;
    
    AppState app_state = {
        .running = false
    };

    pthread_mutex_init(&mutex, NULL);
    buf = buffer_init(buf_ptr, READ_BUFFER_SIZE * 5);
    serial_task.mutex_read = &mutex;
    serial_task.app_state = &app_state;
    serial_task.msg = msg;
    serial_task.buf = &buf;

    activate_params.app_state = &app_state;
    activate_params.copy = copy; 
    activate_params.input_buffer = input_buffer; 

    app_state.file = fopen(LOG_FILE, "a+");

    int serial_port = open(DEVICE, O_RDWR); // Change to the appropriate serial port
    if (serial_port < 0)
    {
        perror("Failed to open the serial port");
        return 1;
    }
    struct termios tty;
    
    if (config_tty(&tty, serial_port, B_RATE)) {
        close(serial_port);
        return 1;
    }

    app_state.serial_port = serial_port;

    activate_params.buffer = gtk_text_buffer_new(NULL);
    serial_task.buffer = activate_params.buffer;
    gtk_text_buffer_set_text (activate_params.buffer, "0", -1);

    app = gtk_application_new ("org.gtk.canasta", G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect (app, "activate", G_CALLBACK (activate), &activate_params);
    
    app_state.running = 1;
    pthread_create(&thread_read, NULL, task_read, &serial_task);
    status = g_application_run (G_APPLICATION (app), argc, argv);
    
    app_state.running = 0;
    pthread_join(thread_read, NULL);
    g_object_unref (app);   

    close(serial_port);
    fclose(app_state.file);
    return status;
}


void send_event(GtkWidget *widget, gpointer data) {
    char * copy;
    SendEventParams * send_event_params = (SendEventParams *)data;
    copy = send_event_params->copy;
    char* buffer = (char*)gtk_entry_buffer_get_text(send_event_params->buffer);
    int n = strlen(buffer) + 2;
    if (n > 2) {
        memset(copy, 0, WRITE_MAX_N_CHARS + 2);
        strcpy(copy, buffer);
        copy[n-1] = '\0'; 
        copy[n-2] = '\n';
        serial_write(send_event_params->app_state->serial_port, copy);
    }
    write_serial_log(buffer, send_event_params->app_state->file, SEND);
}

void send_reset(GtkWidget *widget, gpointer data) {
    AppState * app_state = (AppState *) data;
    const char reset_text[3] = "r\n";
    serial_write(app_state->serial_port, reset_text);
    write_serial_log("RESET", app_state->file, SEND);
}

static void activate(GtkApplication *app, gpointer user_data) {
    GtkWidget *window, *input, *button, *view_marcador, *box, *label_marcador, *reset_btn, *label_objetivo;
    GtkEntryBuffer *buffer;
    ActivateParams * activate_paramas; 
    static SendEventParams static_send;
    SendEventParams * send_event_params;
    GtkWidget * list[5];
    
    activate_paramas = (ActivateParams *) user_data;
    
    buffer = gtk_entry_buffer_new(activate_paramas->input_buffer, 0);
    input = gtk_entry_new_with_buffer(buffer);
    gtk_entry_set_max_length(GTK_ENTRY(input), 4);
    
    send_event_params = &static_send;    
    send_event_params->app_state = activate_paramas->app_state;
    send_event_params->buffer = buffer;
    send_event_params->copy = activate_paramas->copy;
    
    label_objetivo = gtk_label_new("Objetivo:");
    label_marcador = gtk_label_new("Marcador:");

    view_marcador = gtk_text_view_new_with_buffer(activate_paramas->buffer);
    gtk_text_view_set_editable (GTK_TEXT_VIEW (view_marcador), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW (view_marcador), FALSE);
    gtk_text_view_set_justification(GTK_TEXT_VIEW (view_marcador), GTK_JUSTIFY_CENTER);
    gtk_widget_set_size_request(GTK_WIDGET(view_marcador), 1, 30);
    gtk_widget_set_margin_start(GTK_WIDGET(view_marcador), 15);
    gtk_widget_set_margin_end(GTK_WIDGET(view_marcador), 15);

    button = gtk_button_new_with_label("Enviar");
    g_signal_connect(button, "clicked", G_CALLBACK(send_event), send_event_params);

    reset_btn = gtk_button_new_with_label("Reiniciar");
    g_signal_connect(reset_btn, "clicked", G_CALLBACK(send_reset), activate_paramas->app_state);
    list[0] = label_objetivo;
    list[1] = create_row_entry(input, button);
    list[2] = label_marcador;
    list[3] = view_marcador;
    list[4] = reset_btn;
    box = create_box(list, sizeof(list)/sizeof(GtkWidget *));

    window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(window), "Canasta");
    gtk_window_set_default_size(GTK_WINDOW(window), 600, 500);
    gtk_window_set_child(GTK_WINDOW(window), GTK_WIDGET(box));
    gtk_window_present(GTK_WINDOW(window));
}

