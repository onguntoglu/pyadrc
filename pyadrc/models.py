import numpy as np
import scipy.signal


class QuadAltitude(object):

    def __init__(self, dt: float = 0.001,
                 m: float = 0.028, g: float = 9.807):
        """Discrete-time model of altitude of a quadcopter

        :param dt: Discretization time (zero-order hold)
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
        self.m = m
        self.dt = dt

        self.vel = 0
        self.pos = 0

    def __call__(self, u):

        acc = u * (1/self.m) - self.g

        self.vel += acc * self.dt
        self.pos += self.vel * self.dt

        return self.pos


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
