TARGET = copilotmain
SRCS =  copilotmain.c \
	PmodGPS.c \
	xuartns550.c xuartns550_l.c xuartns550_stats.c xuartns550_selftest.c xuartns550_options.c\
	PmodNAV.c \
	xspi.c xspi_options.c \
	PmodOLED.c OledGrph.c OledChar.c OledDriver.c

OBJS = $(SRCS:.c=.o)
CFLAGS = -O2 -Wall -g
LDFLAGS += -lpthread -lm

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(TARGET) $(OBJS)

