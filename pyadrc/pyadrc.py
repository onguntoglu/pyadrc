"""Active Disturbance Rejection Control for Python

It is highly recommended to check the documentation first before attempting
to use the package. Although it is simple in nature, ADRC requires some
basic knowledge about PID control, observers and state-feedback.

"""

import numpy as np
import time

# from collections import deque


def saturation(_limits: tuple, _val: float) -> float:
    """Saturation function

    Parameters
    ----------
    _limits : tuple
        saturation limits (low, high)
    _val : float
        sat(_val)

    Returns
    -------
    float
        saturated signal
    """

    lo, hi = _limits

    if _val is None:
        return None
    elif hi is not None and _val > hi:
        return hi
    elif lo is not None and _val < lo:
        return lo

    return _val


class StateSpace():

    """Discrete linear time-invariant state space implementation\
    of ADRC

    Parameters
    ----------
    order : int
        first- or second-order ADRC
    delta : float
        sampling time in seconds
    b0 : float
        gain parameter b0
    t_settle : float
        settling time in seconds, determines closed-loop bandwidth
    k_eso : float
        observer bandwidth
    eso_init : np.array, optional
        initial state for the extended state observer, by default False
    r_lim : tuple, optional
        rate limits for the control signal, by default (None, None)
    m_lim : tuple, optional
        magnitude limits for the control signal, by default (None, None)
    half_gain : tuple, optional
        half gain tuning for controller/observer gains,\
        by default (False, False)
    """

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

    def _update_eso(self, y: float, ukm1: float):
        """Update the linear extended state observer

        Parameters
        ----------
        y : float
            Current measurement y[k]
        ukm1 : float
            Previous control signal u[k-1]
        """

        self.xhat = self.oA.dot(self.xhat) + self.oB.dot(
            ukm1).reshape(-1, 1) + self.L.dot(y)

    def _limiter(self, u_control: float) -> float:
        """Implements rate and magnitude limiter

        Parameters
        ----------
        u_control : float
            control signal to be limited

        Returns
        -------
        float
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
            tuple: magnitude limits of the controller
        """

        return self.m_lim

    @magnitude_limits.setter
    def magnitude_limits(self, lim: tuple):
        """Magnitude limitter setter

        Args:
            lim (tuple): New magnitude limits
        """

        assert len(lim) == 2
        # assert lim[0] < lim[1]
        self.m_lim = lim

    @property
    def rate_limits(self) -> tuple:
        """Returns the rate limits of the controller

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

        Parameters
        ----------
        y : float
            Current measurement y[k] of the process
        u : float
            Previous control signal u[k-1]
        r : float
            Current reference signal r[k]
        zoh : bool, optional
            Only update every delta seconds, by default False

        Returns
        -------
        float
            Current control signal u[k]
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
        u = self._limiter(u)
        self._update_eso(y, u)

        self._last_output = float(u)
        self._last_time = now
        return float(u)
