import numpy as np


class QuadAltitude(object):

    def __init__(self, delta=0.001, m=0.028, g=9.807):
        """Discrete-time model of altitude of a quadcopter

        Args:
            delta (float): Sampling time
            m (float): Mass of the quadcopter in kg
            g (float): Gravity
            ukm1 (float): Previous control signal (time k-1)
        """

        assert isinstance(g, float), 'Gravity needs to be of type float'
        assert isinstance(m, float), 'Mass of quadcopter \
            needs to be of type float'

        self.g = g
        self.delta = delta

        self.Ad = np.vstack(([1, delta],
                             [0, 1]))
        self.Bd = np.vstack(([0, 0],
                            [delta*m, -delta*g]))

        self.Cd = np.hstack((1, 0)).reshape(1, -1)
        self.Dd = 0

        self.delta = delta

        self.x = np.zeros((2, 1), dtype=np.float64)

    def __call__(self, u):

        # Turn on gravity
        u_k = np.vstack((u, 1))

        self.x = self.Ad.dot(self.x) + self.Bd.dot(u_k)

        return self.Cd.dot(self.x)
