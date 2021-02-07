import pyadrc
import math
import matplotlib.pyplot as plt


def main():

    dadrc = pyadrc.adrc.transfer_function(order=2, delta=0.001,
                                          b0=1/0.028, w_cl=2 * math.pi, k_eso=10)

    system = pyadrc.QuadAltitude()

    _u, _y, _setpoint, _td, = [], [], [], []

    u = 0
    y = 0
    counter = 0

    r = 50

    while counter < 10000:
        y = system(u)

        counter = counter + 1

        [filtered_r, u] = dadrc(y, r)
        # [filtered_r, u] = [0, 5]

        _y.append(y)
        _u.append(u)
        _setpoint.append(r)
        _td.append(filtered_r)

        if counter == 5000:
            r = 8

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(_y, ds='steps', label='output')
    plt.plot(_setpoint, ds='steps', label='setpoint')
    plt.plot(_td, ds='steps', label='td')
    plt.title('Output')
    plt.xlabel('Samples')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(_u, ds='steps', label='input')
    plt.legend()

    plt.show()


if __name__ == "__main__":
    main()