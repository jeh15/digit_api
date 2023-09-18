import digit_api
import numpy as np
import time


def main(argv=None):
    # Initialize Communication:
    publisher_address = "127.0.0.1"
    digit_api.initialize_communication(publisher_address, 25501, 25500)

    # Wait for connection:
    digit_api.wait_for_connection()

    # Check connection:
    print(f"Connection status: {digit_api.check_connection()}")

    # Limits Wrapper Test:
    torque_limits = digit_api.get_torque_limits()
    damping_limits = digit_api.get_damping_limits()
    velocity_limits = digit_api.get_velocity_limits()

    # PD Controller:
    NUM_MOTORS = 20
    NUM_JOINTS = 10
    target_position = np.array([
        -0.0462933,
        -0.0265814,
        0.19299,
        -0.3,
        -0.0235182,
        -0.0571617,
        0.0462933,
        0.0265814,
        -0.19299,
        0.3,
        -0.0235182,
        0.0571617,
        -0.3,
        0.943845,
        0.0,
        0.3633,
        0.3,
        -0.943845,
        0.0,
        -0.3633,
    ])

    while True:
        motor_position = digit_api.get_actuated_joint_position()
        desired_torques = []
        desired_velocities = []
        desired_damping = []
        for i in range(NUM_MOTORS):
            desired_torques.append(150.0 * (target_position[i] - motor_position[i]))
            desired_velocities.append(0.0)
            desired_damping.append(0.75 * damping_limits[i])

        # Create Command Array:
        desired_torques = np.asarray(desired_torques)
        desired_velocities = np.asarray(desired_velocities)
        desired_damping = np.asarray(desired_damping)
        command = np.array([desired_torques, desired_velocities, desired_damping]).T

        # Send Command:
        digit_api.send_command(command, 0, True)

        # Control Rate: 1000Hz
        control_rate = 1.0 / 1000.0
        time.sleep(control_rate)


if __name__ == "__main__":
    main()
