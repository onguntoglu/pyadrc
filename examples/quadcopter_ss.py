import pyadrc
import matplotlib.pyplot as plt

import time


def main():

    dadrc = pyadrc.adrc.state_space(order=2, delta=0.001,
                                    b0=1/0.028, t_settle=1, k_eso=10)

    system = pyadrc.QuadAltitude()

    _u, _y, _setpoint = [], [], []

    u = 0
    y = 0
    counter = 0

    r = 5

    while counter < 10000:
        y = system(u)

        dadrc.magnitude_lim = (0, 10)
        _y.append(y)
        _u.append(u)
        _setpoint.append(r)

        counter = counter + 1
        # time.sleep(0.001)

        if counter >= 5000:
            u = dadrc(y, u, r, False)

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(_y, ds='steps', label='output')
    plt.plot(_setpoint, ds='steps', label='setpoint')
    plt.title('Output')
    plt.xlabel('Samples')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(_u, ds='steps', label='input')
    plt.title('Samples')
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
