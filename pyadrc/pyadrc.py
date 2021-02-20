"""Main module."""

import numpy as np
import time

from collections import deque


def saturation(_limits: tuple, _val: float) -> float:
    """Saturation function

    Args:
        _limits (tuple): saturation limits (low, high)
        _val (float): sat(_val)

    Returns:
        float: saturated signal
    """

    lo, hi = _limits

    if _val is None:
        return None
    elif hi is not None and _val > hi:
        return hi
    elif lo is not None and _val < lo:
        return lo

    return _val


class adrc():

    """
    Python class implementing Active Disturbance Rejection Control

    It is highly recommended to check the documentation first before attempting
    to use the package. Although it is simple in nature, ADRC requires some
    basic knowledge about PID control, observers and state-feedback.
    """

    class state_space():

        def __init__(self,
                     order: int,
                     delta: float,
                     b0: float,
                     t_settle: float,
                     k_eso: float,
                     eso_init: np.array = False,
                     r_lim: tuple = (None, None),
                     m_lim: tuple = (None, None),
                     half_gain: tuple = (False, False)):
            """Discrete linear time-invariant state space implementation\
                of ADRC

            Args:
                order (int): first- or second-order ADRC
                delta (float): sampling time in seconds
                b0 (float): gain parameter b0
                t_settle (float): settling time in seconds, determines\
                closed-loop bandwidth
                k_eso (float): observer bandwidth
                eso_init (np.array, optional): initial state for the extended\
                    state observer. Defaults to False.
                r_lim (tuple, optional): rate limits for the\
                    control signal. Defaults to (None, None).
                m_lim (tuple, optional): magnitude limits for the\
                    control signal. Defaults to (None, None).
                half_gain (tuple, optional): half gain tuning for\
                    controller/observer gains. Defaults to (False, False).
            """

            assert (order == 1) or (order == 2),\
                'Only first- and second-order ADRC is implemented'

            self.b0 = b0
            nx = order + 1
            self.delta = delta

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
                self.L = np.array([1 - (zESO)**2,
                                   (1 / delta) * (1 - zESO)**2]).reshape(-1, 1)

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
                                   (3 / (2 * delta)) * (1 - zESO)**2 * (1 + zESO),
                                   (1 / delta**2) * (1 - zESO)**3]
                                  ).reshape(-1, 1)

                # Controller gains
                self.w = np.array([self.Kp / self.b0,
                                   self.Kd / self.b0,
                                   1 / self.b0]).reshape(-1, 1)

            self.xhat = np.zeros((nx, 1), dtype=np.float64)

            self.ukm1 = np.zeros((1, 1), dtype=np.float64)

            self.m_lim = m_lim
            self.r_lim = r_lim

            if half_gain[0] is True:
                self.w = self.w / 2
            if half_gain[1] is True:
                self.L = self.L / 2

            if eso_init is not False:
                assert len(eso_init) == nx,\
                    'Length of initial state vector of LESO not compatible\
                        with order'
                self.xhat = np.array(eso_init).reshape(-1, 1)

            self._last_time = None
            self._last_output = None
            self._last_input = None

            self._linear_extended_state_observer()

        def _linear_extended_state_observer(self):
            """Internal function implementing the one-step update
            equation for the linear extended state observer
            """

            self.oA = self.Ad - self.L @ self.Cd @ self.Ad
            self.oB = self.Bd - self.L @ self.Cd @ self.Bd
            self.oC = self.Cd

            # if self.inc_form is True:
            #    self.oA = self.oA - np.eye(self.oA.shape[0])

        def update_eso(self, y: float, ukm1: float):
            """Update the linear extended state observer

            Args:
                y (float): Current measurement y[k]
                ukm1 (float): Previous control signal u[k-1]
            """

            self.xhat = self.oA.dot(self.xhat) + self.oB.dot(
                ukm1).reshape(-1, 1) + self.L.dot(y)

        def limiter(self, u_control: float) -> float:
            """Implements rate and magnitude limiter

            Args:
                u_control (float): control signal to be limited

            Returns:
                float: rate and magnitude limited control signal
            """

            # Limiting the rate of u (delta_u)
            # delta_u = SaturatedInteger(self.r_lim[0], self.r_lim[1],
            #                            u_control - self.ukm1)

            delta_u = saturation((self.r_lim[0], self.r_lim[1]),
                                 u_control - self.ukm1)

            # Limiting the magnitude of u
            self.ukm1 = saturation((self.m_lim[0],
                                    self.m_lim[1]),
                                   delta_u + self.ukm1)

            return self.ukm1

        @property
        def eso_states(self) -> tuple:
            """Returns the states of the linear extended state observer

            Returns:
                tuple: States of the linear extended state observer
            """

            return self.xhat

        @property
        def magnitude_limits(self) -> tuple:
            """Returns the magnitude limits of the controller

            Returns:
                tuple: Magnitude limits of the controller
            """

            return self.m_lim

        @magnitude_limits.setter
        def magnitude_limits(self, lim: tuple):
            """Magnitude limitter setter

            Args:
                lim (tuple): New magnitude limits
            """

            assert len(lim) == 2
            assert lim[0] < lim[1]
            self.m_lim = lim

        @property
        def rate_limits(self) -> tuple:
            """Returns the magnitude limits of the controller

            Returns:
                tuple: Rate limits of the controller
            """

            return self.r_lim

        @rate_limits.setter
        def rate_limits(self, lim: tuple):
            """Rate limiter setter

            Args:
                lim (tuple): New rate limits
            """

            assert len(lim) == 2
            assert lim[0] < lim[1]
            self.r_lim = lim

        def __call__(self, y: float, u: float, r: float, zoh: bool = False) -> float:
            """Returns value of the control signal depending on current measurements,
            previous control action, reference signal.

            Args:
                y (float): Current measurement y[k] of the process
                u (float): Previous control signal u[k-1]
                r (float): Current reference signal r[k]
                zoh (bool, optional): only update every delta seconds.\
                    Defaults to False.

            Returns:
                float: Current control signal u[k]
            """

            now = time.monotonic()

            if zoh is True:
                try:
                    dt = now - self._last_time if now - self._last_time else 1e-16
                except TypeError:
                    dt = 1e-16

            if zoh is True and dt < self.delta and self._last_output is not None:
                # Return last output of the controller if not enough time has passed
                return self._last_output

            u = (self.Kp / self.b0) * r - self.w.T @ self.xhat
            u = self.limiter(u)
            self.update_eso(y, u)

            self._last_output = float(u)
            self._last_time = now
            return float(u)

    class transfer_function(object):

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

            r_lim (tuple) (float, float): rate limits for the control
                output. Defaults to -np.inf, np.inf

            m_lim (tuple) (float, float): magnitude limits for the
                control output. Defaults to -np.inf, np.inf

            half_gain (tuple) (bool, bool):  half gain tuning for
                controller/observer gains, Default to False

        """

        def __init__(self, order, delta, b0,
                     w_cl, k_eso, eso_init=False,
                     r_lim=(None, None),
                     m_lim=(None, None),
                     half_gain=(False, False)):

            raise NotImplementedError('Transfer function form of ADRC is not\
                                      fully implemented')

            self.b0 = b0
            zESO = np.exp(-k_eso * w_cl * delta)

            self.integrator = 0.

            self.order = order

            if order == 1:

                # Controller gains
                k1 = w_cl

                # Observer gains
                l1 = 1 - zESO**2
                l2 = (1 / delta) * (1 - zESO)**2

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
                l2 = (3 / (2 * delta)) * (1 - zESO)**2 * (1 + zESO)
                l3 = (1 / delta**2) * (1 - zESO)**3

                self.alpha1 = (delta ** 2 / 2) * (
                    k1 - k1 * l1 - k2 * l2) + delta * k2 + delta * l2 + l1 - 2

                self.alpha2 = (((delta**2 * k1) / 2) - delta * k2 + 1) * (1 - l1)

                self.beta0 = (1 / b0) * (k1 * l1 + k2 * l2 + l3)

                self.beta1 = (1 / b0) * (((
                    delta**2 * k1 * l3) / 2) + delta * k1 * l2 + delta * k2 * l3 - 2 * (k1 * l1 + k2 * l2 + l3))

                self.beta2 = (1 / b0) * (((
                    delta**2 * k1 * l3) / 2) - delta * k1 * l2 - delta * k2 * l3 + (k1 * l1 + k2 * l2 + l3))

                self.gam0 = k1 / (k1 * l1 + k2 * l2 + l3)

                self.gam1 = (k1 * (delta**2 * l3 + 2 * delta * l2 + 2 * l1 - 6)) / (k1 * l1 + k2 * l2 + l3)

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
                    (self.beta1 / self.beta0) * self.prefilt_y[0]

            else:

                y = self.gam0 * x +\
                    self.gam1 * self.prefilt_x[0] +\
                    self.gam2 * self.prefilt_x[1] +\
                    self.gam3 * self.prefilt_x[2] -\
                    (self.beta1 / self.beta0) * self.prefilt_y[0] -\
                    (self.beta2 / self.beta0) * self.prefilt_y[1]

            self.prefilt_y.appendleft(y)
            self.prefilt_x.appendleft(x)

            return y

        def _c_fb(self, x):

            if self.order == 1:

                y = self.beta0 * x + \
                    self.beta1 * self.ctrl_in[0] -\
                    self.alpha1 * self.ctrl_out[0]

            else:

                y = self.beta0 * x + \
                    self.beta1 * self.ctrl_in[0] + \
                    self.beta2 * self.ctrl_in[1] - \
                    self.alpha1 * self.ctrl_out[0] - \
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
