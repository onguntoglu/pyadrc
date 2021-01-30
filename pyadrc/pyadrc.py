import numpy as np
import scipy.signal

from collections import deque


class ADRC():

    """
    Python class implementing Active Disturbance Rejection Control

    It is highly recommended to check the README first before attempting
    to use the package. Although it is simple in nature, ADRC requires some
    basic knowledge about PID control, observers and state-feedback.
    """

    class adrc_ss():
        """Discrete time linear active disturbance rejection control
        in state-space representation

        Args:
            order (int): first- or second-order ADRC

            delta (float): sampling time in seconds

            b0 (float): modelling parameter b0

            t_settle (float): settling time in seconds, determines
                closed-loop bandwidth

            k_eso (float): observer bandwidth

            eso_init (np.array): initial state for the extended state observer,
                Defaults to False, i.e. x0 = 0

            rate_lim (tuple) (float, float): rate limits for the control
                output. Defaults to -np.inf, np.inf

            magnitude_lim (tuple) (float, float): magnitude limits for the
                control output. Defaults to -np.inf, np.inf

            half_gain (tuple) (bool, bool):  half gain tuning for
                controller/observer gains, Default to False

        """

        def __init__(self,
                     order,
                     delta,
                     b0,
                     t_settle,
                     k_eso,
                     eso_init=False,
                     rate_lim=(None, None),
                     magnitude_lim=(None, None),
                     half_gain=(False, False)):

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

            elif order == 2:
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

            self.ukm1 = np.zeros((1, 1), dtype=np.float64)

            self.magnitude_lim = magnitude_lim
            self.rate_lim = rate_lim

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

            # if self.inc_form is True:
            #    self.oA = self.oA - np.eye(self.oA.shape[0])

        def update_eso(self, y, ukm1):
            """Update the linear extended state observer

            Args:
                y (float): Current measurement (time k) of the process
                ukm1 (float): Previous control signal (time k-1)
            """

            self.xhat = self.oA.dot(self.xhat) + self.oB.dot(
                    ukm1).reshape(-1, 1) + self.L.dot(y)

            """
            if self.inc_form is False:
            else:
                self.delta_x = self.oA.dot(self.xhat) + self.oB.dot(
                    ukm1).reshape(-1, 1) + self.L.dot(y)
                    self.xhat = self.delta_x + self.xhat
            """

        def _saturation(self, _limits, _val):

            lo, hi = _limits

            if _val is None:
                return None
            elif hi is not None and _val > hi:
                return hi
            elif lo is not None and _val < lo:
                return lo

            return _val

        def limiter(self, u_control):
            """Implements rate and magnitude limiter"""

            # Limiting the rate of u (delta_u)
            # delta_u = SaturatedInteger(self.rate_lim[0], self.rate_lim[1],
            #                            u_control - self.ukm1)

            delta_u = self._saturation((self.rate_lim[0], self.rate_lim[1]),
                                       u_control - self.ukm1)

            # Limiting the magnitude of u
            self.ukm1 = self._saturation((self.magnitude_lim[0],
                                         self.magnitude_lim[1]),
                                         delta_u + self.ukm1)

            return self.ukm1

        def __call__(self, y, u, r):
            """Update the linear ADRC controller

            Args:
                y (float): Current measurement (time k) of the process
                u (float): Previous control signal (time k-1)
                r (float): reference (setpoint)

            Returns:
                u (float): Control signal u
            """

            u = (self.Kp / self.b0) * r - self.w.T @ self.xhat
            u = self.limiter(u)
            self.update_eso(y, u)

            return u

    class adrc_tf(object):

        """Discrete time linear active disturbance rejection control
        in transfer function representation

        Args:
            order (int): first- or second-order ADRC

            delta (float): sampling time in seconds

            b0 (float): modelling parameter b0

            w_cl (float): desired closed-loop bandwidth

            k_eso (float): observer bandwidth

            eso_init (np.array): initial state for the extended state observer,
                Defaults to False, i.e. x0 = 0

            rate_lim (tuple) (float, float): rate limits for the control
                output. Defaults to -np.inf, np.inf

            magnitude_lim (tuple) (float, float): magnitude limits for the
                control output. Defaults to -np.inf, np.inf

            half_gain (tuple) (bool, bool):  half gain tuning for
                controller/observer gains, Default to False

        """

        def __init__(self, order, delta, b0,
                     w_cl, k_eso, eso_init=False,
                     rate_lim=(None, None),
                     magnitude_lim=(None, None),
                     half_gain=(False, False)):

            self.b0 = b0
            zESO = np.exp(-k_eso * w_cl * delta)

            self.integrator = 0.

            self.order = order

            if order == 1:

                # Controller gains
                k1 = w_cl

                # Observer gains
                l1 = 1 - zESO**2
                l2 = (1/delta) * (1 - zESO)**2

                self.alpha1 = (delta * k1 - 1) * (1 - l1)
                self.beta0 = (1 / b0) * (k1 * l1 + l2)
                self.beta1 = (1 / b0) * (delta * k1 * l2 - k1 * l1 - l2)
                self.gam0 = k1 / (k1 * l1 + l2)
                self.gam1 = k1 * (delta * l2 + l1 - 2) / (k1 * l1 + l2)
                self.gam2 = k1 * (1 - l1) / (k1 * l1 + l2)

                self.prefilt_x = deque([0, 0], maxlen=2)
                self.prefilt_y = deque([0], maxlen=1)
                self.ctrl_in = deque([0], maxlen=1)
                self.ctrl_out = deque([0], maxlen=1)

            elif order == 2:

                k1 = w_cl**2
                k2 = 2 * w_cl

                l1 = 1 - zESO**3
                l2 = (3/(2*delta)) * (1 - zESO)**2 * (1 + zESO)
                l3 = (1 / delta**2) * (1 - zESO)**3

                self.alpha1 = (delta ** 2 / 2) * (
                    k1 - k1 * l1 - k2 * l2) + delta * k2 + delta * l2 + l1 - 2

                self.alpha2 = (((delta**2 * k1) / 2) - delta * k2 + 1) * (1 - l1)

                self.beta0 = (1 / b0) * (k1 * l1 + k2 * l2 + l3)

                self.beta1 = (1 / b0) * (((
                    delta**2 * k1 * l3) / 2) + delta * k1 * l2 +
                    delta * k2 * l3 - 2 * (k1 * l1 + k2 * l2 + l3))

                self.beta2 = (1 / b0) * (((
                    delta**2 * k1 * l3) / 2) - delta * k1 * l2 -
                    delta * k2 * l3 + (k1 * l1 + k2 * l2 + l3))

                self.gam0 = k1 / (k1 * l1 + k2 * l2 + l3)

                self.gam1 = (k1 * (delta**2 * l3 + 2 *
                                   delta * l2 + 2 * l1 - 6)) / (k1 * l1 + k2 * l2 + l3)

                self.gam2 = (k1 * (delta**2 * l3 - 2 * delta * l2 - 4 * l1 + 6)) / (k1 * l1 + k2 * l2 + l3)

                self.gam3 = (k1 * (l1 - 1)) / (k1 * l1 + k2 * l2 + l3)

                self.prefilt_x = deque([0, 0, 0], maxlen=3)
                self.prefilt_y = deque([0, 0], maxlen=2)
                self.ctrl_in = deque([0, 0], maxlen=2)
                self.ctrl_out = deque([0, 0], maxlen=2)

        def _ref_prefilter(self, x):
            """Filter the reference signal"""

            if self.order == 1:

                y = self.gam0 * x +\
                    self.gam1 * self.prefilt_x[0] +\
                    self.gam2 * self.prefilt_x[1] -\
                    (self.beta1/self.beta0) * self.prefilt_y[0]

            else:

                y = self.gam0 * x +\
                    self.gam1 * self.prefilt_x[0] +\
                    self.gam2 * self.prefilt_x[1] +\
                    self.gam3 * self.prefilt_x[2] -\
                    (self.beta1/self.beta0) * self.prefilt_y[0] -\
                    (self.beta2/self.beta0) * self.prefilt_y[1]
                    
            self.prefilt_y.appendleft(y)
            self.prefilt_x.appendleft(x)

            return y

        def _fb_ctrl(self, x):
            """
            Calculate controller output
            """

            if self.order == 1:

                y = self.beta0 * x +\
                    self.beta1 * self.ctrl_in[0] -\
                    (self.alpha1 - 1) * self.ctrl_out[0] +\
                    self.alpha1 * self.ctrl_out[1]

            else:
                y = self.beta0 * x +\
                    self.beta1 * self.ctrl_in[0] +\
                    self.beta2 * self.ctrl_in[1] -\
                    (self.alpha1 - 1) * self.ctrl_out[0] -\
                    (self.alpha2 - self.alpha1) * self.ctrl_out[1] +\
                    self.alpha2 * self.ctrl_out[2]

            self.ctrl_in.appendleft(x)
            self.ctrl_out.appendleft(y)

            return y

        def _c_fb(self, x):

            if self.order == 1:

                y = self.beta0 * x + self.beta1 * self.ctrl_in[0] -\
                    self.alpha1 * self.ctrl_out[0]

            else:

                y = self.beta0 * x + self.beta1 * self.ctrl_in[0] +\
                    self.beta2 * self.ctrl_in[1] -\
                    self.alpha1 * self.ctrl_out[0] -\
                    self.alpha2 * self.ctrl_out[1]

            self.ctrl_in.appendleft(x)
            self.ctrl_out.appendleft(y)

            self.integrator += y

            return self.integrator

        def __call__(self, y, r):
            """Update the linear ADRC controller

            Args:
                y (float): Current measurement (time k) of the process
                r (float): reference (setpoint)

            Returns:
                u (float): Control signal u
            """

            filt_r = self._ref_prefilter(r)

            return filt_r, self._c_fb(filt_r - y)



class QuadAlt(object):
    """
    Altitude simulation of a quadcopter
    """

    def __init__(self):

        self.g = 9.807
        self.m = 0.028
        self.z = 0
        self.zdot = 0

    def __call__(self, thrust, dt):

        self.zdot += self.zdot + dt*(-self.g + (1/self.m) * thrust)
        self.z += self.z + dt * self.zdot

        return self.z


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
        self.delta = delta

        self.Ad = np.vstack(([1, delta], [0, 1]))
        self.Bd = np.vstack((0, delta*m))
        self.Cd = np.hstack((1, 0)).reshape(1, -1)
        self.Dd = 0

        self.delta = delta

        self.x = np.zeros((2, 1), dtype=np.float64)

    def __call__(self, u):

        self.x = self.Ad.dot(self.x) + self.Bd.dot(u)

        return self.Cd.dot(self.x)


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
