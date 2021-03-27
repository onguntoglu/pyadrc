"""Active Disturbance Rejection Control for Python

It is highly recommended to check the documentation first before attempting
to use the package. Although it is simple in nature, ADRC requires some
basic knowledge about PID control, observers and state-feedback.

"""

import numpy as np
import time

from collections import deque


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
    inc_form : float
        toggle incremental form of ADRC, by default False. If the incremental
        form is toggled, the controller will return the incrementation to the
        current control signal. This value needs to be accumulated by the user.
        Functionally identical to the non-incremental (normal) form.
    eso_init : tuple, optional
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
                 inc_form: bool = False,
                 eso_init: tuple = False,
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
            self.xhat = np.fromiter(eso_init, np.float64).reshape(-1, 1)

        self._last_time = None
        self._last_output = None
        self._last_input = None
        self.inc_form = inc_form

        self._linear_extended_state_observer()

    def _linear_extended_state_observer(self):
        """Internal function implementing the one-step update
        equation for the linear extended state observer
        """

        self.oA = self.Ad - self.L @ self.Cd @ self.Ad
        self.oB = self.Bd - self.L @ self.Cd @ self.Bd
        self.oC = self.Cd

        if self.inc_form is True:
            self.oA = self.oA - np.eye(self.oA.shape[0])
            self.rkm1 = 0.

    def _update_eso(self, y: float, ukm1: float):
        """Update the linear extended state observer

        Parameters
        ----------
        y : float
            Current measurement y[k]
        ukm1 : float
            Previous control signal u[k-1]
        """
        if self.inc_form is False:
            self.xhat = self.oA.dot(self.xhat) + self.oB.dot(
                ukm1).reshape(-1, 1) + self.L.dot(y)
        else:  # self.inc_form is True
            self.xhat_delta = self.oA.dot(self.xhat) + self.oB.dot(
                ukm1).reshape(-1, 1) + self.L.dot(y)

            self.xhat = self.xhat + self.xhat_delta

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

        Returns
        -------
        tuple
            States of the linear extended state observer
        """

        return self.xhat

    @property
    def magnitude_limiter(self) -> tuple:
        """Returns the magnitude limits of the controller

        Returns
        -------
        tuple
            magnitude limits of the controller
        """

        return self.m_lim

    @magnitude_limiter.setter
    def magnitude_limiter(self, lim: tuple):
        """Magnitude limitter setter

        Parameters
        ----------
            lim : tuple
                New magnitude limits
        """

        assert len(lim) == 2
        # assert lim[0] < lim[1]
        self.m_lim = lim

    @property
    def rate_limiter(self) -> tuple:
        """Returns the rate limits of the controller

        Returns
        -------
        tuple
            Rate limits of the controller
        """

        return self.r_lim

    @rate_limiter.setter
    def rate_limiter(self, lim: tuple):
        """Rate limiter setter

        Parameters
        ----------
        lim : tuple
            New rate limits
        """

        assert len(lim) == 2
        assert lim[0] < lim[1]
        self.r_lim = lim

    def __call__(self, y: float, u: float, r: float, zoh: bool = False):
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
            # Return last output of the controller if dt < delta
            return self._last_output

        self._update_eso(y, u)

        if self.inc_form is False:
            u = (self.Kp / self.b0) * r - self.w.T @ self.xhat
            u = self._limiter(u)
        else:  # self.inc_form is True:
            delta_r = r - self.rkm1
            delta_u = ((self.Kp / self.b0) * delta_r
                       - self.w.T @ self.xhat_delta)
            u = u + delta_u

            self.rkm1 = r

        self._last_output = float(u)
        self._last_time = now
        return float(u)

    def reset(self, x0=None):
        """Resets the extended state observer

        Parameters
        ----------
        x0 : np.array, optional
            new initial state vector for the extended state observer,
            by default None, i.e. resets to zero
        """

        nx = len(self.xhat)

        assert x0 is None or len(np.fromiter(x0, float)) == nx,\
            "Invalid state vector"

        if x0 is None:
            self.xhat = np.zeros((nx, 1), dtype=np.float64)
        else:
            self.xhat = np.fromiter(x0, np.float64).reshape(-1, 1)


class TransferFunction(object):

    """Discrete time linear active disturbance rejection control\
        in transfer function representation

    Parameters
    ----------
    order : int
        first- or second-order ADRC TF
    delta : float
        sampling time in seconds
    b0 : float
        modelling parameter b0
    w_cl : float
        desired closed-loop bandwidth
    k_eso : float
        observer bandwidth is parametrized as k_eso-multiple faster than w_cl
    eso_init : np.array, optional
        initial state for the extended state observer, by default None
    r_lim : tuple, optional
        rate limits for the control output, by default (None, None)
    m_lim : tuple, optional
        magnitude limits for the control output, by default (None, None)
    half_gain : tuple, optional
        half gain tuning for controller/observer gains,\
            by default (False, False)
    method : str, optional, 'general_terms' or 'bandwidth'
        method with which the transfer function parameters are calculated,\
            functionally identical, but general_terms is used to implement half
            gain tuning
    """

    def __init__(self, order: int, delta: float, b0: float,
                 w_cl: float, k_eso: float, eso_init: np.array = None,
                 r_lim: tuple = (None, None),
                 m_lim: tuple = (None, None),
                 half_gain: tuple = (False, False), method='general_terms'):

        assert order == 1 or order == 2, 'First- and second-order ADRC TF\
            implemented'

        # Define attributes
        self.order = order
        self.acc = 0.
        zESO = np.exp(-k_eso * w_cl * delta)

        self.params = self._calculate_parameters(order, delta, b0, w_cl, zESO,
                                                 method=method)

        self._calculate_coeffs()
        self._create_states()

    @property
    def accumulator(self):
        """Get accumulator

        Returns
        -------
        float
            accumulator output
        """
        return self.acc

    @accumulator.setter
    def accumulator(self, value):
        """Set accumulator output to value

        Parameters
        ----------
        value : float
            value to set the accumulator
        """
        self.acc = float(value)

    @property
    def parameters(self):
        """Get parameters

        Returns
        -------
        dict
            calculated paramaters
        """
        return self.params

    def _create_states(self):
        """Create state vectors to store past values for transfer functions
        """

        if self.order == 1:

            # Reference prefilter
            self.r_k = deque([0, 0], maxlen=2)
            self.r_yk = deque([0], maxlen=1)

            # Feedback controller
            self.e_k = deque([0], maxlen=1)
            self.u_k = deque([0], maxlen=1)

        else:

            # Reference prefilter
            self.r_k = deque([0, 0, 0], maxlen=3)
            self.r_yk = deque([0, 0], maxlen=2)

            # Feedback controller
            self.e_k = deque([0, 0], maxlen=2)
            self.u_k = deque([0, 0], maxlen=2)

    def _control_loop(self, y, r_k):

        if self.order == 1:

            # reference prefilter
            ref_yk = (r_k * self.pf['num'][0]
                      + self.r_k[0] * self.pf['num'][1]
                      + self.r_k[1] * self.pf['num'][2]
                      - self.r_yk[0] * self.pf['den'][1])

            # advance time
            self.r_k.appendleft(r_k)
            self.r_yk.appendleft(ref_yk)

            # calculate error
            e_k = ref_yk - y

            # feedback controller
            u_k = (e_k * self.fb['num'][0]
                   + self.e_k[0] * self.fb['num'][1]
                   - self.u_k[0] * self.fb['den'][1])

            # advance time
            self.e_k.appendleft(e_k)
            self.u_k.appendleft(u_k)

        else:  # self.order == 2

            # reference prefilter
            ref_yk = (r_k * self.pf['num'][0]
                      + self.r_k[0] * self.pf['num'][1]
                      + self.r_k[1] * self.pf['num'][2]
                      + self.r_k[2] * self.pf['num'][3]
                      - self.r_yk[0] * self.pf['den'][1]
                      - self.r_yk[1] * self.pf['den'][2])

            # advance time
            self.r_k.appendleft(r_k)
            self.r_yk.appendleft(ref_yk)

            # calculate error
            e_k = ref_yk - y

            # feedback controller
            u_k = (e_k * self.fb['num'][0]
                   + self.e_k[0] * self.fb['num'][1]
                   + self.e_k[1] * self.fb['num'][2]
                   - self.u_k[0] * self.fb['den'][1]
                   - self.u_k[1] * self.fb['den'][2])

            # advance time
            self.e_k.appendleft(e_k)
            self.u_k.appendleft(u_k)

        # Accumulate feedback controller output
        self.accumulator = self.accumulator + u_k

        return self.accumulator

    def _calculate_coeffs(self):
        """Internal function to calculate the coefficients for the reference
        prefilter and feedback controller (denominator and numerator)
        Uses the z^-1 notation, coefficients in descending order, i.e
        z^0, z^-1, ..., z^-n
        """

        if self.order == 1:
            fb = {'num': [self.params['be0'], self.params['be1']],
                  'den': [1, self.params['a1']]}
            pf = {'num': [self.params['g0'],
                          self.params['g1'], self.params['g2']],
                  'den': [1, self.params['be1'] / self.params['be0']]}
        else:
            fb = {'num': [self.params['be0'], self.params['be1'],
                          self.params['be2']],
                  'den': [1, self.params['a1'], self.params['a2']]}
            pf = {'num': [self.params['g0'], self.params['g1'],
                          self.params['g2'], self.params['g3']],
                  'den': [1, self.params['be1'] / self.params['be0'],
                          self.params['be2'] / self.params['be0']]}

        self.fb = fb
        self.pf = pf

    def _calculate_parameters(self, order, delta, b0,
                              w_cl, zESO, method='general_terms'):

        """Calculate the coefficients of num and den of TransferFunction ADRC,
        using either general terms or bandwidth parameterization

        Parameters
        ----------
        order : int
            internal variable
        delta : float
            internal variable
        b0 : float
            internal variable
        w_cl : float
            internal variable
        zESO : float
            internal variable
        method : str, optional
            Calculation via general terms (i.e. controller and observer gains)
            or bandwidth parametrization, functionally identical,
            implementation uses general terms to leverage half-gain
            tuning method, by default 'general_terms'

        Returns
        -------
        list
            Ordered transfer function parameters

        Notes
        -----
        * For first-order ADRC: [a1, be0, be1, g0, g1, g2]
        * For second-order ADRC: [a1, a2, be0, be1, be2, g0, g1, g2, g3]
        """

        if method == 'general_terms':
            if order == 1:
                # Controller gains
                k1 = w_cl

                # Observer gains
                l1 = 1 - zESO**2
                l2 = (1 / delta) * (1 - zESO)**2

                _C_TERM = (k1 * l1 + l2)

                a1 = (delta * k1 - 1) * (1 - l1)
                be0 = (1 / b0) * _C_TERM
                be1 = (1 / b0) * (delta * k1 * l2 - k1 * l1 - l2)
                g0 = k1 / _C_TERM
                g1 = k1 * (delta * l2 + l1 - 2) / _C_TERM
                g2 = k1 * (1 - l1) / _C_TERM

            elif order == 2:

                k1 = w_cl**2
                k2 = 2 * w_cl

                l1 = 1 - zESO**3
                l2 = (3 / (2 * delta)) * (1 - zESO)**2 * (1 + zESO)
                l3 = (1 / delta**2) * (1 - zESO)**3

                _C_TERM = (k1 * l1 + k2 * l2 + l3)

                a1 = ((delta ** 2 / 2) * (k1 - k1 * l1 - k2 * l2)
                      + delta * k2 + delta * l2 + l1 - 2)

                a2 = ((((delta**2 * k1) / 2) - delta * k2 + 1) * (1 - l1))

                be0 = (1 / b0) * _C_TERM

                be1 = ((1 / b0) * (((delta**2 * k1 * l3) / 2)
                                   + delta * k1 * l2 + delta
                                   * k2 * l3 - 2
                                   * _C_TERM))

                be2 = ((1 / b0) * (((delta**2 * k1 * l3) / 2)
                                   - delta * k1 * l2 - delta
                                   * k2 * l3 + _C_TERM))

                g0 = k1 / _C_TERM

                g1 = ((k1 * (delta**2 * l3 + 2 * delta * l2 + 2 * l1 - 6))
                      / _C_TERM)

                g2 = ((k1 * (delta**2 * l3 - 2 * delta * l2 - 4 * l1 + 6))
                      / _C_TERM)

                g3 = (k1 * (l1 - 1)) / _C_TERM

        elif method == 'bandwidth':
            if order == 1:

                _C_TERM = ((delta * w_cl) * (1 - zESO**2) + (1 - zESO)**2)

                a1 = (delta * w_cl - 1) * zESO**2

                be0 = ((1/(b0 * delta)) * _C_TERM)

                be1 = ((1/(b0 * delta)) * ((-2 * delta * w_cl * zESO)
                                           * (1 - zESO) - (1 - zESO)**2))

                g0 = (delta * w_cl) * _C_TERM
                g1 = (-2 * delta * w_cl * zESO) * _C_TERM
                g2 = (delta * w_cl * zESO**2) * _C_TERM

            elif order == 2:

                _C_TERM = (delta**2 * w_cl**2 * (1 - zESO**3)
                           + 3 * delta * w_cl * (1 - zESO - zESO**2 + zESO**3)
                           + (1 - zESO)**3)

                a1 = ((1/2) * (delta**2 * w_cl**2 * zESO**3
                               + delta * w_cl
                               * (1 + 3 * zESO + 3 * zESO**2 - 3 * zESO**3)
                               + (1 - 3 * zESO - 3 * zESO**2 + zESO**3)))

                a2 = ((zESO**3 / 2)
                      * (delta**2 * w_cl**2 - 4 * delta * w_cl + 2))

                be0 = ((1 / (b0 * delta**2)) * _C_TERM)

                be1 = ((1 / (b0 * delta**2))
                       * (-3 * delta**2 * w_cl**2 * zESO * (1 - zESO**2)
                          - 4 * delta * w_cl * (1 - 3 * zESO**2 + 2 * zESO**3)
                          - 2 * (1 - zESO)**3))

                be2 = ((1 / (b0 * delta**2))
                       * (-3 * delta**2 * w_cl**2 * zESO**2 * (1 - zESO)
                          + delta * w_cl
                          * (1 + 3 * zESO - 9 * zESO**2 + 5 * zESO**3)
                          + (1 - zESO)**3))

                g0 = (delta**2 * w_cl**2) / _C_TERM
                g1 = (-3 * delta**2 * w_cl**2 * zESO) / _C_TERM
                g2 = (3 * delta**2 * w_cl**2 * zESO**2) / _C_TERM
                g3 = (-1 * delta**2 * w_cl**2 * zESO**3) / _C_TERM

        if order == 1:
            params = {'a1': a1, 'be0': be0,
                      'be1': be1, 'g0': g0,
                      'g1': g1, 'g2': g2}
        else:
            params = {'a1': a1, 'a2': a2, 'be0': be0,
                      'be1': be1, 'be2': be2, 'g0': g0,
                      'g1': g1, 'g2': g2, 'g3': g3}

        return params

    def __call__(self, y, r):
        """Control loop call

        Parameters
        ----------
        y : float
            current output signal
        r : float
            current reference signal

        Returns
        -------
        float
            control signal to the plant (accumulator output)
        """

        return self._control_loop(y, r)
