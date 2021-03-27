import pyadrc
import matplotlib.pyplot as plt


def main():
    dadrc = pyadrc.StateSpace(order=2, delta=0.001,
                              b0=2/25, t_settle=0.1, k_eso=10, inc_form=True)

    system = pyadrc.System(K=2.0, T=5., D=0.9, delta=0.001)

    _u, _y, _setpoint, _delta_u = [], [], [], []

    u = 0
    y = 0
    counter = 0
    r = 5

    while counter < 1000:
        y = system(u)

        counter = counter + 1

        delta_u = dadrc(y, u, r)
        u += delta_u
        # [filtered_r, u] = [0, 5]

        _y.append(y)
        _u.append(u)
        _setpoint.append(r)
        _delta_u.append(delta_u)

        if counter == 500:
            r = 15

    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(_y, ds='steps', label='output')
    plt.plot(_setpoint, ds='steps', label='setpoint')
    plt.title('Output')
    plt.xlabel('Samples')
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(_u, ds='steps', label='input')
    plt.plot(_delta_u, ds='steps', label='integrating')
    plt.title('Samples')
    plt.legend()
    
    plt.show()


if __name__ == "__main__":
    main()
