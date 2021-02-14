import pyadrc
import matplotlib.pyplot as plt

import time


def main():

    dadrc = pyadrc.adrc.state_space(order=2, delta=0.001,
                                    b0=1, t_settle=0.1, k_eso=10)

    system = pyadrc.RandomSystem(states=12)

    _u, _y, _setpoint = [], [], []

    u = 1
    y = 0
    counter = 0

    r = 10

    while counter < 1000:
        y = system(u)
        # u = dadrc(y, u, r, True)

        _y.append(y)
        _u.append(u)
        _setpoint.append(r)

        counter = counter + 1

        time.sleep(0.001)
        
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
