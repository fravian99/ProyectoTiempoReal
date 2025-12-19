SRCDIR := ./src
INCLUDEDIR := ./include
OBJDIR := ./obj

CC:=cc
CFLAGS:= -g -pthread
CFLAGS += -I$(INCLUDEDIR)
CFLAGS += -Wall -Wextra #-Werror
CFLAGS += `pkg-config --cflags gtk4`

LDLIBS += -lm
#LDLIBS :=  $(shell pkg-config --libs gtk4)
LDLIBS +=`pkg-config --libs gtk4`

SRCS:= $(wildcard $(SRCDIR)/*.c)
OBJS:= $(patsubst $(SRCDIR)/%.c,$(OBJDIR)/%.o, $(SRCS))
TARGET:= program

TAR := tar cvf
TARCONTENT := Makefile $(SRCDIR) $(INCLUDEDIR) meson.build
TARNAME := $(TARGET)
TAREXTENSION := tgz
TARFILENAME := $(TARNAME).$(TAREXTENSION)


MESON := meson
MESON_DIR:= build
MESON_TARGET:=$(MESON_DIR)/$(TARGET)

VALGRIND_FILENAME :=valgrind.log
VALGRIND_FLAGS := --track-origins=yes --show-leak-kinds=all --leak-check=full --log-file="$(VALGRIND_FILENAME)"

LOG_FILE := file.log

all: $(TARGET)
	@echo "compilado correctamente"

$(TARGET): $(OBJS)
	@echo "enlazando $^"
	$(CC) $^ -o $@ $(LDLIBS)

$(OBJDIR)/%.o :  $(SRCDIR)/%.c | $(OBJDIR)
	@echo "compilando $<"
	$(CC) $(CFLAGS) -c $< -o $@

$(OBJDIR):
	@echo "creando directorio $@"
	mkdir $@

clean:
	@echo "eliminando $(OBJDIR), $(TARGET) $(TARFILENAME) $(MESON_DIR) "
	rm -rf $(OBJDIR) $(TARGET) $(TARFILENAME) $(MESON_DIR) $(VALGRIND_FILENAME) $(LOG_FILE)


tar: clean
	 $(TAR) $(TARFILENAME) $(TARCONTENT) 

meson_compile: $(MESON_TARGET)
	cp $^ $(TARGET)

$(MESON_TARGET): $(MESON_DIR)
	@echo "compile $(MESON_TARGET)"
	meson compile -C $^

$(MESON_DIR):
	meson setup $@

valgrind: $(TARGET)
	valgrind $(VALGRIND_FLAGS) ./$(TARGET)

install: $(MESON_DIR) | $(MESON_TARGET)
	@echo "creando directorio $<"
	sudo meson install -C $< 

uninstall: $(MESON_DIR) | $(MESON_TARGET)
	sudo ninja uninstall -C $< $(TARGET)

.PHONY: all clean meson_compile valgrind install uninstall