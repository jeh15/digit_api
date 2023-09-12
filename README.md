# Digit API

Repo containing Agility's lowlevelapi.

I had to change the original Makefiles to be able to compile the code for MacOS.

The original Makefiles are the old_Makefile.

## Dependencies:
The `libartl` library depends on `zstd`. It came pre-packaged with the library but I was not able to get it to work.
As a result, my new `libartl/Makefile` depends on a homebrew installation. 

Additionally, the `lowlevelapi/Makefile` assumes a pip install of pybind11.