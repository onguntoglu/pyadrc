import numpy as np
import scipy
import matplotlib.pyplot as plt

import pyadrc


def main():

    dadrc = pyadrc.ADRC.adrc_ss(order=2, delta=0.001, b0=0.028, t_settle=1,
                                k_eso=10, eso_init=False,
                                rate_lim=(-5, 5),
                                magnitude_lim=(-100, 300),
                                half_gain=(False, False))

    quadcopter = pyadrc.QuadAltitude()

    _u, _y = [], []

    u = 0
    counter = 0

    _u.append(0)
    _y.append(0)

    while counter < 10000:
        y = quadcopter(u)

        # if counter > 5000:
        #    y = y + 2

        u = dadrc(y, u, 5)
        counter = counter + 1

        _y.append(y)
        _u.append(u)

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(_y, ds='steps')
    plt.title('Output')
    plt.xlabel('Amplitude')
    plt.ylabel('Samples')

    plt.subplot(2, 1, 2)
    plt.plot(_u, ds='steps')
    plt.title('Input')
    plt.xlabel('Amplitude')
    plt.ylabel('Samples')

    plt.show()


if __name__ == "__main__":
    main()