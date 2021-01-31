import numpy as np
import scipy


class QuadAltitude(object):

    def __init__(self, delta: float = 0.001,
                 m: float = 0.028, g: float = 9.807):
        """Discrete-time model of altitude of a quadcopter

        :param delta: Discretization time (zero-order hold)
        in seconds, defaults to 0.001
        :type delta: float, optional
        :param m: Mass of the quadcopter, defaults to 0.028
        :type m: float, optional
        :param g: Gravitational acceleration, defaults to 9.807
        :type g: float, optional
        """

        assert isinstance(g, float), 'Gravity needs to be of type float'
        assert isinstance(m, float), 'Mass of quadcopter \
            needs to be of type float'

        self.g = g
        self.delta = delta

        self.Ad = np.vstack(([1, delta],
                             [0, 1]))
        self.Bd = np.vstack(([0, 0],
                            [delta * m, -delta * g]))

        self.Cd = np.hstack((1, 0)).reshape(1, -1)
        self.Dd = 0

        self.delta = delta

        self.x = np.zeros((2, 1), dtype=np.float64)

    def __call__(self, u):

        # Turn on gravity
        u_k = np.vstack((u, 1))

        self.x = self.Ad.dot(self.x) + self.Bd.dot(u_k)

        return self.Cd.dot(self.x)


class System(object):

    def __init__(self, K: float = 1.0, T: float = 1.0,
                 D: float = None, delta: float = 0.001):
        """Python class to generate a first-or second-order process
        for simulation and verification purposes

        :param K: system gain, defaults to 1.0
        :type K: float, optional
        :param T: time constant, defaults to 1.0
        :type T: float, optional
        :param D: damping factor, defaults to None
        :type D: float, optional
        :param delta: discretization time in seconds,
        defaults to 0.001
        :type delta: float, optional
        """
        assert isinstance(K, float)
        assert isinstance(T, float)
        assert delta > 0, "Sampling time has to be positive"

        # First-order process
        num = np.array([K]).reshape(-1)
        den = np.array([T, 1]).reshape(-1)

        if D is not None:
            assert isinstance(D, float),\
                "Damping factor D must be type float"
            # Second-order process
            den = np.array([T**2, 2 * D * T, 1]).reshape(-1)

        system = scipy.signal.tf2ss(num, den)

        system = scipy.signal.cont2discrete(system, delta)

        self.A = system[0]
        self.B = system[1]
        self.C = system[2]

        self.x = np.zeros((len(den) - 1, 1), dtype=np.float64)

    def __call__(self, u: float) -> float:
        """System response w.r.t. control signal u

        :param u: control signal u
        :type u: float
        :return: system response y
        :rtype: float
        """

        self.x = self.A.dot(self.x) + self.B.dot(u)

        return self.C.dot(self.x)
