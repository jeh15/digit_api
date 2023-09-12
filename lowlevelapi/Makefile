# Platform to build for: MACOS or LINUX
PLATFORM := MACOS
COMPILER := g++-13

INCLUDE := $(shell python3 -m pybind11 --includes)
FLAG := -O3 -Wall -shared -std=c++11 -fPIC
SUFFIX := $(shell python3-config --extension-suffix)

ifeq ($(PLATFORM),MACOS)
FLAG += -undefined dynamic_lookup
endif

digit_api: lowlevelapi_py.cc lowlevelapi.c libartl/libartl.a
	$(COMPILER) $(FLAG) $(INCLUDE) $^ -o $@$(SUFFIX)

# Build the example program
lowlevelapi_example: lowlevelapi_example.c lowlevelapi.c libartl/libartl.a
	$(COMPILER) $^ -std=c++11 -undefined dynamic_lookup -o $@

# Build the static ARTL library
libartl/libartl.a:
	$(MAKE) -C libartl libartl.a

# Clean outputs from both this directory and libartl
clean:
	rm -f lowlevelapi_example
	$(MAKE) -C libartl clean

.PHONY: clean
