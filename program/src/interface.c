#include "../include/interface.h"

GtkWidget * create_row_entry(GtkWidget *widget1, GtkWidget *widget2) {
    GtkWidget * grid_input;
    grid_input = gtk_grid_new();

    gtk_grid_set_column_spacing(GTK_GRID(grid_input), 30);
    gtk_grid_set_row_spacing(GTK_GRID(grid_input), 2);

    gtk_widget_set_hexpand(GTK_WIDGET(widget1), true);
    gtk_widget_set_vexpand(GTK_WIDGET(widget1), false);

    gtk_widget_set_hexpand(GTK_WIDGET(widget2), true);
    gtk_widget_set_vexpand(GTK_WIDGET(widget2), false);

    gtk_grid_attach(GTK_GRID(grid_input), GTK_WIDGET(widget1), 1, 10, 8, 2);
    gtk_grid_attach(GTK_GRID(grid_input), GTK_WIDGET(widget2), 14, 10, 8, 2);

    //gtk_widget_set_hexpand(GTK_WIDGET(grid_input), true);
    //gtk_widget_set_vexpand(GTK_WIDGET(grid_input), true);
    return GTK_WIDGET(grid_input);
}

GtkWidget * create_box(GtkWidget ** widget_list, int size) {
    GtkWidget * box;
    GtkWidget * actual;
    box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 30);
    
    gtk_widget_set_margin_top(GTK_WIDGET(box), 12);
    gtk_widget_set_margin_bottom(GTK_WIDGET(box), 12);
    gtk_widget_set_margin_start(GTK_WIDGET(box), 12);
    gtk_widget_set_margin_end(GTK_WIDGET(box), 12);

    for (int i = 0; i < size; i++) {
        actual = widget_list[i];
        gtk_box_append(GTK_BOX(box), GTK_WIDGET(actual));
    }
    return box;
}