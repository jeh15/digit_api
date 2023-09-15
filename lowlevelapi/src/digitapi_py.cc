#include "lowlevelapi.h"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

// Wrapper for send command:
void wrapper_llapi_send_command(
    py::array_t<double, py::array::c_style | py::array::forcecast> array,
    int32_t fallback_opmode,
    bool apply_command
) {
    // Map Numpy array to llapi_motor_t struct:

    // Data structure to map:
    // Motor:
    // typedef struct __attribute__((packed)) {
    //   double torque;
    //   double velocity;
    //   double damping;
    // } llapi_motor_t;
    // Command:
    // typedef struct __attribute__((packed)) {
    //   llapi_motor_t motors[NUM_MOTORS];
    //   int32_t fallback_opmode;
    //   bool apply_command;
    // } llapi_command_t;
}

PYBIND11_MODULE(digit_api, m) {
    m.doc() = "Python bindings for Digit's low level api.";

    // Communication Functions:
    m.def(
        "initialize_communication", &llapi_init_custom, "Starts subscriber and publisher communication.",
        py::arg("publisher_address"), py::arg("subscriber_port") = 25501, py::arg("publisher_port") =25500
    );

    m.def(
        "check_connection", &llapi_connected, "Returns true if the subscriber is connected to Digit."
    );

    // Get and Send Functions:
    m.def(
        "get_states", &llapi_get_observation, "Returns a copy of Digit's current states."
    );

    m.def(
        "send_command", &llapi_send_command, "Sends a command to Digit."
    );

    // Limit Functions:
    m.def(
        "get_limits", &llapi_get_limits, "Returns a copy of Digit's command limits."
    );

}