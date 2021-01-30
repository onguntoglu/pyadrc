import numpy as np
import scipy
import matplotlib.pyplot as plt

import pyadrc


def main():

    dadrc = pyadrc.ADRC.adrc_ss(order=2, delta=0.001, b0=1/16, t_settle=0.5,
                                k_eso=10, eso_init=False,
                                rate_lim=(None, None),
                                magnitude_lim=(None, None),
                                half_gain=(False, False))

    system = pyadrc.System(K=1.0, T=4.0, D=1.0, delta=0.001)

    _u, _y = [], []

    u = 0
    y = 0
    counter = 0

    _y.append(0)
    _u.append(0)

    r = 5

    while counter < 1000:
        y = system(u)

        u = dadrc(y, u, r)
        counter = counter + 1

        _y.append(y)
        _u.append(u)

        if counter == 500:
            r = 8

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