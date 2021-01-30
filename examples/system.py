import numpy as np
import scipy


class System(object):

    def __init__(self, K=1.0, T=1.0, D=None, delta=None):
        """Python class to generate a first-or second-order process
        for simulation and verification purposes

        Args:
            K (float): DC gain
            T (float): Time constant
            D (float): Damping factor
            delta (float): Sampling time in seconds
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
            den = np.array([T**2, 2*D*T, 1]).reshape(-1)

        system = scipy.signal.tf2ss(num, den)

        system = scipy.signal.cont2discrete(system, delta)

        self.A = system[0]
        self.B = system[1]
        self.C = system[2]

        self.x = np.zeros((len(den) - 1, 1), dtype=np.float64)

    def __call__(self, u):

        self.x = self.A.dot(self.x) + self.B.dot(u)

        return self.C.dot(self.x)
