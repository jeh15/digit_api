import digit_api


def main(argv=None):
    publisher_address = "127.0.0.1"
    digit_api.initialize_communication(publisher_address, 25501, 25500)


if __name__ == "__main__":
    main()
