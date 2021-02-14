import pyadrc
import matplotlib.pyplot as plt
import numpy as np

import time


def main():
    
    delta = 0.001

    dadrc = pyadrc.adrc.state_space(order=2, delta=delta,
                                    b0=1/0.028, t_settle=0.5, k_eso=10)

    system = pyadrc.QuadAltitude()

    _u, _y, _setpoint = [], [], []

    u = 0
    y = 0
    counter = 0
    r = 10
    sample = 1000

    while counter <= sample:
        y = system(u)
        u = dadrc(y, u, r, False)

        _y.append(y)
        _u.append(u)
        _setpoint.append(r)

        counter = counter + 1

    t = np.linspace(0, delta*sample, sample+1)

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(t, _y, ds='steps', label='altitude')
    plt.plot(t, _setpoint, ds='steps', label='reference')
    plt.title('Output')
    plt.legend()
    plt.grid()

    plt.subplot(2, 1, 2)
    plt.plot(t, _u, ds='steps', label='thrust')
    plt.legend()
    plt.xlabel('Time [s]')
    plt.grid()
    plt.show()
    


if __name__ == "__main__":
    main()
