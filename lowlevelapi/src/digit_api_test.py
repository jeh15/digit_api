import digit_api
import numpy as np


def main(argv=None):
    publisher_address = "127.0.0.1"
    digit_api.initialize_communication(publisher_address, 25501, 25500)
    print(f"Connection status: {digit_api.check_connection()}")

    motor_array = np.hstack(
        [
            1 * np.ones((20, 1), dtype=np.float64),
            2 * np.ones((20, 1), dtype=np.float64),
            3 * np.ones((20, 1), dtype=np.float64),
        ],
    )
    print(motor_array.shape)
    # motor_array = np.zeros((20, 3), dtype=np.float64)
    digit_api.send_command(motor_array, 0, False)


if __name__ == "__main__":
    main()
