import numpy as np
import scipy
import matplotlib.pyplot as plt

import pyadrc


def main():

    dadrc = pyadrc.ADRC.DiscreteLinearADRC(order=2, delta=0.001,
                                           b0=0.028, t_settle=2,
                                           k_eso=10, eso_init=False,
                                           half_gain=(False, False),
                                           inc_form=True)

    quadcopter = pyadrc.QuadAltitude(delta=0.001)

    ylist = list()
    ulist = list()

    u = 0
    counter = 0

    ylist.append(0)
    ulist.append(0)

    while counter < 100000:
        y = quadcopter(u)
        u = dadrc(y, u, 1)
        counter = counter + 1

        ylist.append(y)
        ulist.append(u)

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(ylist, ds='steps')
    plt.title('Output')
    plt.xlabel('Amplitude')
    plt.ylabel('Samples')

    plt.subplot(2, 1, 2)
    plt.plot(ulist, ds='steps')
    plt.title('Input')
    plt.xlabel('Amplitude')
    plt.ylabel('Samples')

    plt.show()


if __name__ == "__main__":
    main()