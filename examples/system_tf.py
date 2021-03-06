import pyadrc
import math
import matplotlib.pyplot as plt


def main():

    adrc_tf = pyadrc.TransferFunction(order=1, delta=0.001,
                                      b0=1, w_cl=0.2, k_eso=10)

    system = pyadrc.System(K=1.0, T=1., D=0.95, delta=0.001)

    _u, _y, _setpoint, _td, = [], [], [], []

    u = 0
    y = 0
    counter = 0

    r = 5

    while counter < 10000:
        y = system(u)

        u = adrc_tf(y, r)

        _y.append(y)
        _u.append(u)
        _setpoint.append(r)

        if counter == 5000:
            r = 5

        counter = counter + 1

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
