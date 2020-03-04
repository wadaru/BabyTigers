# Compile options
URGDIR = ./urg_library/current/samples/c
SRCDIR = $(URGDIR)/../../src
INCLUDEDIR = $(URGDIR)/../../include/c
CC = gcc
CFLAGS = -g -O0 -Wall -Werror -W $(INCLUDES)
INCLUDES = -I$(INCLUDEDIR) -I$(URGDIR)
LDFLAGS =
LDLIBS = -lm `/bin/sh $(URGDIR)/ld_wsock.sh` `/bin/sh $(URGDIR)/ld_setupapi.sh`

# Target
TARGET = \
	get_distance \

all : $(TARGET)

clean :
	$(RM) *.o $(TARGET) *.exe

depend :
	makedepend -Y -- $(INCLUDES) -- $(wildcard *.h *.c)

.PHONY : all depend clean

######################################################################
REQUIRE_LIB = $(SRCDIR)/liburg_c.a
$(REQUIRE_LIB) : $(wildcard $(SRCDIR)/*.[ch])
	cd $(@D)/ && $(MAKE) $(@F)

get_distance : ./urg_library/current/samples/c/open_urg_sensor.o $(REQUIRE_LIB)
find_port : $(REQUIRE_LIB)

