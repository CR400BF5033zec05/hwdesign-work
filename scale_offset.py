import numpy as np


def scale_offset(a, b, x):
    """
    Bit-conscious float32 model of

        y = a*x + b

    Multiply and add are deliberately separate float32 operations.
    """

    a = np.float32(a)
    b = np.float32(b)
    x = np.asarray(x, dtype=np.float32)

    prod = (a * x).astype(np.float32)
    y = (prod + b).astype(np.float32)

    return y


def float32_bits(x):
    """Return float32 values represented as uint32 bit patterns."""
    return np.asarray(x, dtype=np.float32).view(np.uint32)


def check_equal_bits(actual, expected):
    actual_bits = float32_bits(actual)
    expected_bits = float32_bits(expected)

    if not np.array_equal(actual_bits, expected_bits):
        raise AssertionError(
            f"\nactual:   {actual}\n"
            f"expected: {expected}\n"
            f"actual bits:   {actual_bits}\n"
            f"expected bits: {expected_bits}"
        )


def run_hand_tests():
    # Test 1
    y = scale_offset(
        2.0,
        1.0,
        [0.0, 1.0, -2.0, 3.5]
    )

    expected = np.array(
        [1.0, 3.0, -3.0, 8.0],
        dtype=np.float32
    )

    check_equal_bits(y, expected)

    # Test 2
    y = scale_offset(
        -0.5,
        2.0,
        [2.0, -4.0, 0.0]
    )

    expected = np.array(
        [1.0, 4.0, 2.0],
        dtype=np.float32
    )

    check_equal_bits(y, expected)

    # Test 3: identity transform
    x = np.array(
        [0.0, 1.25, -2.75, 100.125, -0.03125],
        dtype=np.float32
    )

    y = scale_offset(1.0, 0.0, x)

    check_equal_bits(y, x)

    print("All hand-computed arithmetic tests PASS")


if __name__ == "__main__":
    run_hand_tests()
