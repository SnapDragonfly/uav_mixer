# Define the compiler
CC = gcc

# Define compiler options
CFLAGS = -Wall -Iinclude -Ilibs/mavlink/all -pthread

# Define source directory and object directory
SRCDIR = src
OBJDIR = obj

# Define the target executable name
TARGET = uav_mixer

# Define source files and object files
SRCS = $(wildcard $(SRCDIR)/*.c)
OBJS = $(SRCS:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

# Default target
all: $(TARGET)

# Link the object files to create the executable
$(TARGET): $(OBJS)
	./version.sh
	$(CC) $(OBJS) -o $(TARGET) -lpthread -lm

# Compile source files into object files
$(OBJDIR)/%.o: $(SRCDIR)/%.c | $(OBJDIR)
	$(CC) $(CFLAGS) -c $< -o $@

# Create the object file directory
$(OBJDIR):
	mkdir -p $(OBJDIR)

# Clean up generated files
clean:
	rm -rf $(OBJDIR) $(TARGET)

.PHONY: all clean
