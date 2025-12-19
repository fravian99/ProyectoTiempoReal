#pragma once
#include <gtk/gtk.h>
#include <glib/gi18n.h>

GtkWidget * create_row_entry(GtkWidget *widget1, GtkWidget *widget2);
GtkWidget * create_box(GtkWidget ** widget_list, int size);