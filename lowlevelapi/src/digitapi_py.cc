#include "lowlevelapi.h"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <iostream>

namespace py = pybind11;

// connection function: Keep sending a message until a response is received.


// Wrapper for send command:
// void wrapper_llapi_send_command(
//     py::array_t<double, py::array::c_style | py::array::forcecast> array,
//     int32_t fallback_opmode,
//     bool apply_command
// ) {
//     // Map Numpy array to llapi_motor_t struct:
//     llapi_command_t command = {0};
//     for(int i = 0; i < NUM_MOTORS; i++) {
//         command.motors[i].torque = array.at(i, 0);
//         command.motors[i].velocity = array.at(i, 1);
//         command.motors[i].damping = array.at(i, 2);

//         std::cout << "Torque: " << command.motors[i].torque << std::endl;
//     }
// }

void wrapper_llapi_send_command(
    const Eigen::Ref<const Eigen::MatrixXd>& array,
    int32_t fallback_opmode,
    bool apply_command
) {
    // Map Numpy array to llapi_motor_t struct:
    llapi_command_t command = {0};
    for(int i = 0; i < NUM_MOTORS; i++) {
        command.motors[i].torque = array(i, 0);
        command.motors[i].velocity = array(i, 1);
        command.motors[i].damping = array(i, 2);
    }
    command.fallback_opmode = fallback_opmode;
    command.apply_command = apply_command;

    // Send command:
    llapi_send_command(&command);
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

    // m.def(
    //     "send_command", &llapi_send_command, "Sends a command to Digit."
    // );

    m.def(
        "send_command", &wrapper_llapi_send_command, "Sends a command to Digit."
    );

    // Limit Functions:
    m.def(
        "get_limits", &llapi_get_limits, "Returns a copy of Digit's command limits."
    );

    // py::class_<llapi_motor_t>(m, "Motor")
    //     .def(py::init<>())
    //     .def_readwrite("torque", &llapi_motor_t::torque)
    //     .def_readwrite("velocity", &llapi_motor_t::velocity)
    //     .def_readwrite("damping", &llapi_motor_t::damping);

    // py::class_<llapi_command_t>(m, "Command")
    //     .def(py::init<>())
    //     .def_readwrite("motors", &llapi_command_t::motors)
    //     .def_readwrite("fallback_opmode", &llapi_command_t::fallback_opmode)
    //     .def_readwrite("apply_command", &llapi_command_t::apply_command);
    
}