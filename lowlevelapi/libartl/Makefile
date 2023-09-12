# Platform to build for: MACOS or LINUX
PLATFORM := MACOS
COMPILER := gcc-13

# Compilation settings
CFLAGS_OBJ := -O2 -Wall -Wextra -fPIC
CFLAGS_DL := 
INC :=
LIBS :=

# Output library names
DLIBOUT := libartl.so
SLIBOUT := libartl.a

# Platform-specific settings
ifeq ($(PLATFORM),MACOS)
CFLAGS_OBJ  += -undefined dynamic_lookup
CFLAGS_DL  += -undefined dynamic_lookup
LIBS    += -L/opt/homebrew/lib
INC     += -I/opt/homebrew/include
else
LIBS    += -L/usr/local/lib
INC     += -I/usr/local/include
endif

# Find all .c files used to create the library
SRC := $(wildcard *.c)

# Library object files
OBJ := $(patsubst %.c,%.o,$(SRC))

# Tests
TESTS := file_test file_partial_test pubsub_test unstructured_test

# By default, build both the static and dynamic library
all: $(DLIBOUT) $(SLIBOUT)

# Object files
%.o: %.c
	$(COMPILER) -c $< $(INC) $(CFLAGS_OBJ) -o $@

# Dynamic library
$(DLIBOUT): $(OBJ)
	$(COMPILER) $(OBJ) -shared $(LIBS) $(CFLAGS_DL) -o $(DLIBOUT)
	strip -u $(DLIBOUT)

# Static library
$(SLIBOUT): $(OBJ)
	ar rcs $(SLIBOUT) $(OBJ)

# Clean just object files
cleanobj:
	find . -name '*.o' -delete

# Clean all generated files
clean: cleanobj
	rm -f $(LDLIBOUT)
	rm -f $(WDLIBOUT)
	rm -f $(LSLIBOUT)
	rm -f $(WSLIBOUT)
	rm -f $(TESTS)
	rm -f test.log
	rm -f unstructured.log

# Build and run all of the tests/examples
# Note: Tests are linux-only
test: $(TESTS)

$(TESTS): %: tests/%.c $(DLIBOUT) Makefile
	$(COMPILER) $< $(DLIBOUT) -g -I. -lpthread -Wl,--rpath='$$ORIGIN' -o $@
	./$@

# Virtual targets
.PHONY: all clean cleanobj test
