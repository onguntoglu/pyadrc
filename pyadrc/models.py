import numpy as np
import scipy.signal


class QuadAltitude(object):
    """Discrete-time model of altitude of a quadcopter

    Parameters
    ----------
    dt : float, optional
        Discretization time (zero-order hold), by default 0.001
    m : float, optional
        Mass of the quadcopter, by default 0.028
    g : float, optional
        Gravitational acceleration, by default 9.807
    """

    def __init__(self, dt: float = 0.001,
                 m: float = 0.028, g: float = 9.807):

        assert isinstance(g, float), 'Gravity needs to be of type float'
        assert isinstance(m, float), 'Mass of quadcopter \
            needs to be of type float'

        self.g = g
        self.m = m
        self.dt = dt

        self.vel = 0
        self.pos = 0

    def __call__(self, u: float) -> float:
        """Input port to the quadcopter

        Parameters
        ----------
        u : float
            Thrust of the rotors

        Returns
        -------
        float:
            Current altitude of quadcopter

        """

        acc = u * (1/self.m) - self.g

        self.vel += acc * self.dt
        self.pos += self.vel * self.dt

        # Include ground
        if self.pos <= 0:
            self.vel = 0
            self.pos = 0

        return self.pos


class System(object):

    """Python class to generate a first-or second-order process
    for simulation and verification purposes

    Parameters
    ----------
    K : float, optional
        system gain, by default 1.0
    T : float, optional
        time constant, by default 1.0
    D : float, optional
        damping factor, by default None
    delta : float, optional
        discretization time in seconds, by default 0.001
    """

    def __init__(self, K: float = 1.0, T: float = 1.0,
                 D: float = None, delta: float = 0.001) -> None:

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

        Parameters
        ----------
        u : float
            control signal u

        Returns
        -------
        float
            system response y
        """

        self.x = self.A.dot(self.x) + self.B.dot(u)

        return float(self.C.dot(self.x))


class RandomSystem(object):

    def __init__(self, states=1) -> None:
        """Create random discrete state space system

        Args:
            states (int, optional): number of states. Defaults to 1.

        Returns:
            None: [description]
        """

        import control

        system = control.drss(states=states)

        self.A = np.array(system.A)
        self.B = np.array(system.B)
        self.C = np.array(system.C)
        self.D = np.array(system.D)

        self.x = np.zeros((states, 1), dtype=np.float64)

    def __call__(self, u: float) -> float:

        self.x = self.A.dot(self.x) + self.B.dot(u)
        return float(self.C.dot(self.x) + self.D.dot(u))
