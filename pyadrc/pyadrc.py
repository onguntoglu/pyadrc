import numpy as np
import scipy.signal

import do_mpc


class ADRC():

    """
    Python class implementing Active Disturbance Rejection Control

    It is highly recommended to check the README first before attempting
    to use the package. Although it is simple in nature, ADRC requires some
    basic knowledge about PID control, observers and state-feedback.
    """

    class ContLinearADRC(object):
        """Continuous time linear active disturbance rejection control

        Args:
            order (int): first- or second-order ADRC
            b0 (float): modelling parameter b0
            t_settle (float): settling time in seconds, determines
                controller bandwidth
            k_eso (float): observer bandwidth
            half_gain (tuple) (bool, bool):  half gain tuning for
                controller/observer gains, Default to False
        """

        def __init__(self, order, b0, t_settle,
                     k_eso, half_gain=(False, False)):

            assert (order == 1) or (order == 2),\
                'Only first- and second-order ADRC is implemented'

            self.b0 = b0
            nx = order + 1

            if order == 1:
                # Controller dynamics
                sCL = -4 / t_settle
                self.Kp = -sCL

                # Observer dynamics
                sESO = k_eso * sCL

                # Observer gains resulting in common-location observer poles
                L = np.array([-2 * sESO, sESO**2]).reshape(-1, 1)
                self.L = L

                # Controller gains
                self.w = np.array([self.Kp / self.b0,
                                   1 / self.b0]).reshape(-1, 1)

                # Half gain tuning for controller/observer
                if half_gain[0] is True:
                    self.w = self.w / 2
                if half_gain[1] is True:
                    self.L = self.L / 2

                self.A = np.vstack(([-L[0], 1], [-L[1], 0]))
                self.B = np.vstack((b0, 0))
                self.C = np.hstack((1, 0)).reshape(1, -1)
                self.D = 0

            if order == 2:

                # Controller parameters for closed-loop dynamics
                sCL = -6 / t_settle
                self.Kp = sCL**2
                self.Kd = -2 * sCL

                # Observer dynamics
                sESO = k_eso * sCL

                # Observer gains resulting in common-location observer poles
                L = np.array([-3 * sESO,
                              3 * sESO**2,
                              -(sESO)**3]).reshape(-1, 1)
                self.L = L

                # Controller gains
                self.w = np.array([self.Kp / self.b0,
                                   self.Kd / self.b0,
                                   1 / self.b0]).reshape(-1, 1)

                if half_gain[0] is True:
                    self.w = self.w / 2
                if half_gain[1] is True:
                    self.L = self.L / 2

                self.A = np.vstack(([-L[0], 1, 0],
                                    [-L[1], 0, 1],
                                    [-L[2], 0, 0]))
                self.B = np.vstack((0, b0, 0))
                self.C = np.hstack((1, 0, 0)).reshape(1, -1)
                self.D = 0

            self.xhat = np.zeros((nx, 1), dtype=np.float64)

        def update_eso(self, y, u):
            """Update the linear extended state observer

            Args:
                y (float): Current measurement (time k) of the process
                u (float): Previous control signal
            """

            self.xhat = self.A.dot(self.xhat) + self.B.dot(
                u).reshape(-1, 1) + self.L.dot(y)

        def __call__(self, y, u, r):
            """Update the linear ADRC

            Args:
                y (float): Current measurement (time k) of the process
                u (float): Previous control signal
                r (float): reference (setpoint)

            Returns:
                u (float): Control signal u
            """

            self.update_eso(y, u)

            u = (self.Kp / self.b0) * r - self.w.T @ self.xhat

            return u

    class DiscreteLinearADRC_SS():
        """Discrete time linear active disturbance rejection control:

        Args:
            order (int): first- or second-order ADRC
            delta (float): sampling time in seconds
            b0 (float): modelling parameter b0
            t_settle (float): settling time in seconds, determines
                controller bandwidth
            k_eso (float): observer bandwidth
            eso_init (np.array): initial state for the extended state observer,
                Defaults to False, i.e. x0 = 0
            half_gain (tuple) (bool, bool):  half gain tuning for
                controller/observer gains, Default to False
            inc_form (bool): incremental form of ADRC, Defaults to False
        """

        def __init__(self, order, delta, b0,
                     t_settle, k_eso, eso_init=False,
                     half_gain=(False, False), inc_form=False):

            assert (order == 1) or (order == 2),\
                'Only first- and second-order ADRC is implemented'

            self.b0 = b0
            nx = order + 1

            if order == 1:
                self.Ad = np.vstack(([1, delta], [0, 1]))
                self.Bd = np.vstack((b0 * delta, 0))
                self.Cd = np.hstack((1, 0)).reshape(1, -1)
                self.Dd = 0

                # Controller parameters for closed-loop dynamics
                sCL = -4 / t_settle
                self.Kp = -2 * sCL

                # Observer dynamics
                sESO = k_eso * sCL
                zESO = np.exp(sESO * delta)

                # Observer gains resulting in common-location observer poles
                self.L = np.array([1 - (zESO)**2, (1 / delta) *
                                   (1 - zESO)**2]).reshape(-1, 1)

                # Controller gains
                self.w = np.array([self.Kp / self.b0,
                                   1 / self.b0]).reshape(-1, 1)

            if order == 2:
                self.Ad = np.vstack((
                    [1, delta, (delta**2) / 2],
                    [0, 1, delta],
                    [0, 0, 1]))
                self.Bd = np.vstack((b0 * (delta**2) / 2, b0 * delta, 0))
                self.Cd = np.hstack((1, 0, 0)).reshape(1, -1)
                self.Dd = 0

                # Controller parameters for closed-loop dynamics
                sCL = -6 / t_settle
                self.Kp = sCL**2
                self.Kd = -2 * sCL

                # Observer dynamics
                sESO = k_eso * sCL
                zESO = np.exp(sESO * delta)

                # Observer gains resulting in common-location observer poles
                self.L = np.array([1 - (zESO)**3,
                                   (3 / (2 * delta)) * (1 - zESO)**2 *
                                   (1 + zESO),
                                   (1 / delta**2) * (1 - zESO)**3]
                                  ).reshape(-1, 1)

                # Controller gains
                self.w = np.array([self.Kp / self.b0,
                                   self.Kd / self.b0,
                                   1 / self.b0]).reshape(-1, 1)

            self.xhat = np.zeros((nx, 1), dtype=np.float64)
            self.ukm1 = 0

            self.delta_x = np.zeros((nx, 1), dtype=np.float64)

            self.inc_form = inc_form

            if half_gain[0] is True:
                self.w = self.w / 2
            if half_gain[1] is True:
                self.L = self.L / 2

            if eso_init is not False:
                assert len(eso_init) == nx,\
                    'Length of initial state vector of LESO not compatible\
                        with order'
                self.xhat = np.array(eso_init).reshape(-1, 1)

            self._linear_extended_state_observer()

        def _linear_extended_state_observer(self):
            """
            Internal function implementing the one-step update
            equation for the linear extended state observer
            """

            self.oA = self.Ad - self.L @ self.Cd @ self.Ad
            self.oB = self.Bd - self.L @ self.Cd @ self.Bd
            self.oC = self.Cd

            if self.inc_form is True:
                self.oA = self.oA - np.eye(self.oA.shape[0])

        def update_eso(self, y, ukm1):
            """Update the linear extended state observer

            Args:
                y (float): Current measurement (time k) of the process
                ukm1 (float): Previous control signal (time k-1)
            """

            if self.inc_form is False:
                self.xhat = self.oA.dot(self.xhat) + self.oB.dot(
                    ukm1).reshape(-1, 1) + self.L.dot(y)
            else:
                self.delta_x = self.oA.dot(self.xhat) + self.oB.dot(
                    ukm1).reshape(-1, 1) + self.L.dot(y)

                self.xhat = self.delta_x + self.xhat

        def __call__(self, y, u, r):
            """Update the linear ADRC controller

            Args:
                y (float): Current measurement (time k) of the process
                u (float): Previous control signal (time k-1)
                r (float): reference (setpoint)

            Returns:
                u (float): Control signal u
            """

            self.update_eso(y, u)

            if self.inc_form is False:
                u = (self.Kp / self.b0) * r - self.w.T @ self.xhat
            else:
                delta_u = (self.Kp / self.b0) * r - self.w.T @ self.delta_x
                u = self.ukm1 + delta_u

            self.ukm1 = u

            return u[0][0]


class QuadAltitude(object):

    def __init__(self, delta=0.001, m=0.028, g=9.807):
        """Discrete-time model of altitude of a quadcopter

        Args:
            delta (float): Sampling time
            m (float): Mass of the quadcopter in kg
            g (float): Gravity
            ukm1 (float): Previous control signal (time k-1)
        """

        self.g = g

        self.Ad = np.vstack(([1, delta], [0, 1]))
        self.Bd = np.vstack((0, delta*m))
        self.Cd = np.hstack((1, 0)).reshape(1, -1)
        self.Dd = 0

        self.x = np.zeros((2, 1), dtype=np.float64)

    def __call__(self, u):

        self.x = self.Ad.dot(self.x) + self.Bd.dot(u - self.g)
        
        if self.x[0][0] < 0:
            self.x = np.zeros((2, 1), dtype=np.float64)

        return self.Cd.dot(self.x)


class System(object):

    def __init__(self, K=1.0, T=1.0, D=None, delta=0):
        """Python class to generate a first-or second-order process
        for simulation and verification purposes

        Args:
            K (float): DC gain
            T (float): Time constant
            D (float): Damping factor
            delta (float): Sampling time in seconds
                if delta == 0: continuous-time process
        """

        assert isinstance(K, float)
        assert isinstance(T, float)
        assert delta >= 0, "Sampling time nonnegative"

        # First-order process
        num = np.array([K]).reshape(-1)
        den = np.array([T, 1]).reshape(-1)

        if D is not None:
            assert isinstance(D, float),\
                "Damping factor D must be type float"
            # Second-order process
            den = np.array([T**2, 2*D*T, 1]).reshape(-1)

        system = scipy.signal.tf2ss(num, den)

        if delta != 0:
            # If sampling time delta is not zero, discretize system
            system = scipy.signal.cont2discrete(system, delta)

        self.system = system

        self.x = np.zeros()

    def __call__(self, u, dt, x0):

        if self.system[4] is None:
            # _, yout, xout = scipy.signal.lsim(tf, U=[self_u, u], T=[0., .1], X0=x0)
            [tout, y, x] = scipy.signal.lsim(self.system, U=u,
                                             T=[0., .1], X0=x0)

            self.t += dt

            return tout, y, x

        else:

            self.t += dt

            return tout, y, x
