.. _introduction-label:

Introduction
============

**Active disturbance rejection control** (ADRC) is a model-free, robust control method, combining the simplicity of PID controllers and the powerful state observers from modern control theory.

The basic premise of ADRC is to approximate a plant using low-order processes (mostly limited to first and second-orders) and extend this approximation by a virtual state, which is then estimated online using a Luenberger state observer (usually called an **extended state observer** (ESO) in ADRC context).

The estimated virtual state is the **total disturbance** (i.e. not accounted by our approximation using low-order processes) which acts on the plant, including any nonlinearity unknown to the designer. The estimated total disturbance is used then in the state-feedback to **decouple the plant**, resulting in a state-feedback controller which rejects the total disturbance in real-time.

.. note::

    Head over to Section :ref:`api-label` for detailed explanation of the complete library.


Tuning ADRC
-----------
    
The designers need to determine whether the process is better approximated by a first- or a second-order process.

.. tip::

    If your process responds to an input with a time-delay, chances are, it can be approximated by a first-order process. 

    However, if there is oscillatory behaviour and some overshoot/undershoot, it is best approximated as a second-order process.

    N-order processes are usually a combination of the aforementioned elementary components.

After selecting the order for ADRC, you need to choose the **gain parameter** :math:`b_0` of your process (sometimes also called a modeling parameter). The optimal value is given by:

    * For first-order processes

    .. math::

        G(s) = \frac{K}{s\cdot T + 1} \rightarrow b_0 = \frac{K}{T}

    * For second-order processes

    .. math::

        G(s) = \frac{K}{s^2 \cdot T + 2\cdot D \cdot T + 1} \rightarrow b_0 = \frac{K}{T^2}

.. important::

    This parameter can be determined by trial-and-error, and it does not have to be perfectly tuned. Somewhere in the vicinity of 50% of the actual value is enough to get a satisfiying closed-loop behaviour.

There are two parameters which determine the **controller** and the **observer** bandwidth - **settling time** and :math:`\mathbf{k_{ESO}}`:

    * **Settling time:**

        This determines the closed-loop dynamics. Specifically, it is the time (in seconds) it takes for the process to reach the 10%-band of the steady-state value. The controller gains are derived from this this parameter.

    * :math:`\mathbf{k_{ESO}}`

        Selects how fast the poles of the extended state observer is in relation to the controller poles. Usually an integer between 3 and 10.

.. important::

    This method of tuning the controller and observer gains is called **bandwith parametrization**.

Congratulations! Tuning an ADRC is that simple!


Magnitude-Rate Limiter
----------------------

In contrast to PID controllers, this implementation of ADRC does not suffer from the dreaded **integral windup**, a condition which occurs when the control signal is saturated (a minimum or a maximum value has been reached), which causes the error to be accumulated. When the integrator gets large enough, it **unwinds**, causing strong oscillations of the system output or even instability

ADRC, however, solves this problem by simply feeding the saturated control signal :math:`u_{lim}` to the extended state observer. This small modification results in slower closed-loop dynamics, but does not cause any windup or related oscillations. 

The rate of change (:math:`\Delta u = u(k+1)-u(k)`) of the control signal is also an important variable to consider when looking at practical implementations. An artifical limitation on rate of change is especially useful to guarentee closed-loop dynamics, i.e. effecting rise and settling times.

.. important::

    The **magnitude limiter** can be used to limit the control signal between a minimum and a maximum value.

    The **rate limiter** puts constraints on the rate of change :math:`\Delta u = u(k+1)-u(k)`. The control signal can only change :math:`\Delta u` in a sampling instant.


Half-Gain Tuning
----------------

In industrial settings, the control loop depends on the measurements of sensors, which are innately noisy.

In high-noise environments, the performance of ESO (in general all Luenberger observers) deteriorates, which in turn results in decreased performance of the closed-loop dynamics.

Half-gain tuning refers to **halving** of controller gains (K/2-case), observer gains (L/2-case) or both applied simultaneously.

    * K/2-case results in low-frequency performance penalty, i.e. slower closed-loop performance.
    * L/2-case, however, significantly increases high-frequency damping while not affecting the low-frequency components and results in reduction of noise effects on the closed-loop performance.
    * Applying both tunings together causes both a controller performance penalty and the reduction of noise effects.

.. important::

    L/2-case results in improved performance in high-noise environments with a very negligible cost of closed-loop performance. Try it out!


Bumpless Transfer
-----------------

Ideally, ADRC is enabled at time t=0, however, in case you need to activate ADRC during runtime without causing any discontinuities in the control signal (i.e. bumpless transfer), you can initialize the ADRC, given that the system in question is in steady-state mode, i.e. no transient or deadband-phases. **Initialize ADRC** using the following state vectors:

    * For first-order ADRC:

    .. math::

        \hat{x}(k-1) = \left(y(k-1) \quad -b_0\cdot u(k-1)\right)^T

    * For second-order ADRC:

    .. math::

        \hat{x}(k-1) = \left(y(k-1) \quad 0 \quad -b_0\cdot u(k-1)\right)^T


What's next?
------------

Head over to :ref:`examples-label` to check out various implementation of the library.

Take a look at :ref:`api-label` for a complete documentation of the library.

Use it in your project, contribute, share!