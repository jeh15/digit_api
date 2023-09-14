import example


def main(argv=None):
    i, j = 1, 2
    result = example.add(i, j)
    print(f"example.add({i}, {j}) = {result}")


if __name__ == "__main__":
    main()
