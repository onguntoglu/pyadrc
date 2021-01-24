import numpy as np
import scipy
import matplotlib.pyplot as plt

import pyadrc


def main():

    dadrc = pyadrc.ADRC.adrc_tf(order=2, delta=1,
                                b0=1, w_cl=1, k_eso=1)

    system = pyadrc.System(K=1.0, T=1.0, D=1.0, delta=0.001)

    _u, _y, _setpoint, _td = [], [], [], []

    u = 0
    y = 0
    counter = 0

    _y.append(0)
    _u.append(0)

    r = 5

    while counter < 10000:
        y = system(u)

        counter = counter + 1

        [filtered_r, u] = dadrc(y, r)

        _y.append(y)
        _u.append(u)
        _setpoint.append(r)
        _td.append(filtered_r)

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(_y, ds='steps')
    plt.plot(_setpoint, ds='steps', label='setpoint')
    plt.plot(_td, ds='steps', label='td')
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