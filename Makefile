
# Makefile

CXX = g++
CFLAGS = -O3 -Wall -Wextra -msse3 -std=c++1z `pkg-config eigen3 --cflags`
LDFLAGS = -lm `pkg-config eigen3 --libs`

OBJDIR = ./obj
OUTDIR = ./bin

TARGET = $(OUTDIR)/tiny_slam
SOURCES = $(notdir $(wildcard ./*.cpp))
OBJECTS = $(addprefix $(OBJDIR)/, $(notdir $(SOURCES:.cpp=.o)))
DEPENDS = $(OBJECTS:.o=.d)

default: $(TARGET)

debug: CFLAGS += -ggdb
debug: $(TARGET)

$(TARGET) : $(OBJECTS)
	mkdir -p $(OUTDIR)
	$(CXX) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(OBJDIR)/%.o : %.cpp
	mkdir -p $(OBJDIR)
	$(CXX) $(CFLAGS) -o $@ -c $<

clean:
	rm -f $(OBJECTS) $(DEPENDS)

test:
	$(TARGET) ./data/exp1.dat 350

.PHONY: clean
.PHONY: debug

-include $(DEPENDS)

