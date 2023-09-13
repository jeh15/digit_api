#include "lowlevelapi.h"

#include <pybind11/pybind11.h>

namespace py = pybind11;

PYBIND11_MODULE(digit_api, m) {
    m.doc() = "Python bindings for Digit's low level api.";

    m.def(
        "initialize_communication", &llapi_init_custom, "Starts subscriber and publisher communication."
    );

}