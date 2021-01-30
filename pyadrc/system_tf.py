import numpy as np
import scipy
import matplotlib.pyplot as plt

import pyadrc


def main():

    dadrc = pyadrc.ADRC.adrc_tf(order=1, delta=0.001,
                                b0=2/5, w_cl=128, k_eso=10)

    system = pyadrc.System(K=2.0, T=5., D=None, delta=0.001)

    _u, _y, _setpoint, _td = [], [], [], []
    
    u = 0
    y = 0
    counter = 0

    r = 5

    while counter < 1000:
        y = system(u)

        counter = counter + 1

        [filtered_r, u] = dadrc(y, r)
        # [filtered_r, u] = [0, 5] 

        _y.append(y[0][0])
        _u.append(u[0][0])
        _setpoint.append(r)
        _td.append(filtered_r)

        if counter == 500:
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
    plt.title('Samples')
    plt.legend()
    plt.show()
    


if __name__ == "__main__":
    main()