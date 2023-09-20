#include "lowlevelapi.h"
#include "include/artl/artl.h"

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <iostream>

namespace py = pybind11;

// TODO: Wrap this in namespace:

// Local variables:
llapi_command_t command = {0};
llapi_observation_t observation;

// connection function: Keep sending a message until a response is received.
void llapi_wait_for_connection() {
    // llapi_command_t command = {0};
    // llapi_observation_t observation;

    // Connect to robot (need to send commands until the subscriber connects)
    command.apply_command = false;
    while (!llapi_get_observation(&observation)) llapi_send_command(&command);
}

void wrapper_llapi_send_command(
    const Eigen::Ref<const Eigen::MatrixXd>& array,
    int32_t fallback_opmode,
    bool apply_command
) {
    // Map Numpy array to llapi_motor_t struct:
    // llapi_command_t command = {0};
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

// Create a wrappers for observation struct:
// TODO(jeh15): 
//  1. Error Handling
//  2. Create structure mapping for observation struct instead of individual wrapper functions.

// Get positions of actuated joints:
py::array_t<double> wrapper_llapi_get_observation_actuated_joints_position(){
    // Get Updated Observations:
    py::array_t<double> np_array(NUM_MOTORS);
    auto position = np_array.mutable_unchecked<1>();
    int return_val = llapi_get_observation(&observation);
    for(int i = 0; i < NUM_MOTORS; i++) {
        position(i) = observation.motor.position[i];
    }
    return np_array;
}

// Get velocities of actuated joints:
py::array_t<double> wrapper_llapi_get_observation_actuated_joints_velocity(){
    // Get Updated Observations:
    py::array_t<double> np_array(NUM_MOTORS);
    auto velocity = np_array.mutable_unchecked<1>();
    int return_val = llapi_get_observation(&observation);
    for(int i = 0; i < NUM_MOTORS; i++) {
        velocity(i) = observation.motor.velocity[i];
    }
    return np_array;
}

// Get torques of actuated joints:
py::array_t<double> wrapper_llapi_get_observation_actuated_joints_torque(){
    // Get Updated Observations:
    py::array_t<double> np_array(NUM_MOTORS);
    auto torque = np_array.mutable_unchecked<1>();
    int return_val = llapi_get_observation(&observation);
    for(int i = 0; i < NUM_MOTORS; i++) {
        torque(i) = observation.motor.torque[i];
    }
    return np_array;
}

// Get positions of unactuated joints:
py::array_t<double> wrapper_llapi_get_observation_unactuated_joints_position(){
    // Get Updated Observations:
    py::array_t<double> np_array(NUM_JOINTS);
    auto position = np_array.mutable_unchecked<1>();
    int return_val = llapi_get_observation(&observation);
    for(int i = 0; i < NUM_JOINTS; i++) {
        position(i) = observation.joint.position[i];
    }
    return np_array;
}

// Get velocities of inactuated joints:
py::array_t<double> wrapper_llapi_get_observation_unactuated_joints_velocity(){
    // Get Updated Observations:
    py::array_t<double> np_array(NUM_JOINTS);
    auto velocity = np_array.mutable_unchecked<1>();
    int return_val = llapi_get_observation(&observation);
    for(int i = 0; i < NUM_JOINTS; i++) {
        velocity(i) = observation.joint.velocity[i];
    }
    return np_array;
}


// Wrapper for limit struct:
py::array_t<double> wrapper_llapi_get_limits_torque(){
    // Get Updated Observations:
    py::array_t<double> np_array(NUM_MOTORS);
    auto limits = np_array.mutable_unchecked<1>();
    const llapi_limits_t* limits_ptr = llapi_get_limits();
    for(int i = 0; i < NUM_MOTORS; i++) {
        limits(i) = limits_ptr->torque_limit[i];
    }
    return np_array;
}

py::array_t<double> wrapper_llapi_get_limits_damping(){
    // Get Updated Observations:
    py::array_t<double> np_array(NUM_MOTORS);
    auto limits = np_array.mutable_unchecked<1>();
    const llapi_limits_t* limits_ptr = llapi_get_limits();
    for(int i = 0; i < NUM_MOTORS; i++) {
        limits(i) = limits_ptr->damping_limit[i];
    }
    return np_array;
}

py::array_t<double> wrapper_llapi_get_limits_velocity(){
    // Get Updated Observations:
    py::array_t<double> np_array(NUM_MOTORS);
    auto limits = np_array.mutable_unchecked<1>();
    const llapi_limits_t* limits_ptr = llapi_get_limits();
    for(int i = 0; i < NUM_MOTORS; i++) {
        limits(i) = limits_ptr->velocity_limit[i];
    }
    return np_array;
}

PYBIND11_MODULE(digit_api, m) {
    m.doc() = "Python bindings for Digit's low level api.";

    // Communication Functions:
    m.def(
        "initialize_communication", &llapi_init_custom, "Starts subscriber and publisher communication.",
        py::arg("publisher_address"), py::arg("subscriber_port") = 25501, py::arg("publisher_port") = 25500
    );

    m.def(
        "check_connection", &llapi_connected, "Returns true if the subscriber is connected to Digit."
    );

    m.def(
        "wait_for_connection", &llapi_wait_for_connection, "Waits for subscriber to connect."
    );

    // Get and Send Functions:
    m.def(
        "get_actuated_joint_position", &wrapper_llapi_get_observation_actuated_joints_position, "Returns a copy of Digit's current states."
    );

    m.def(
        "get_actuated_joint_velocity", &wrapper_llapi_get_observation_actuated_joints_velocity, "Returns a copy of Digit's current states."
    );

    m.def(
        "get_actuated_joint_torque", &wrapper_llapi_get_observation_actuated_joints_torque, "Returns a copy of Digit's current states."
    );

    m.def(
        "get_unactuated_joint_position", &wrapper_llapi_get_observation_unactuated_joints_position, "Returns a copy of Digit's current states."
    );

    m.def(
        "get_unactuated_joint_velocity", &wrapper_llapi_get_observation_unactuated_joints_velocity, "Returns a copy of Digit's current states."
    );

    m.def(
        "send_command", &wrapper_llapi_send_command, "Sends a command to Digit."
    );

    // Limit Functions:
    m.def(
        "get_torque_limits", &wrapper_llapi_get_limits_torque, "Returns a copy of Digit's command limits."
    );

    m.def(
        "get_damping_limits", &wrapper_llapi_get_limits_damping, "Returns a copy of Digit's command limits."
    );

    m.def(
        "get_velocity_limits", &wrapper_llapi_get_limits_velocity, "Returns a copy of Digit's command limits."
    );

}