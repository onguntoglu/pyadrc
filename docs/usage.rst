=====
Usage
=====

To use pyadrc in a project::

    import pyadrc

StateSpace
##########

After importing pyadrc, create StateSpace controller::

    adrc_statespace = pyadrc.StateSpace(order, delta, b0, w_cl, k_eso)

You can find out how to select the parameters by checking :ref:`introduction-label` or :ref:`api-label`. Examples can be found in :ref:`notebooks-label`. 

If you want to configure the initial values for ESO, rate and magnitude limiter and half-gain tuning, tune the parameters at object creation or use the getter/setters to turn them on/off within the control loop::

    adrc_statespace = pyadrc.StateSpace(order, delta, b0, w_cl, k_eso,
                                        eso_init=eso_init, r_lim=r_lim,
                                        m_lim=m_lim, half_gain=half_gain)

TransferFunction
################

Transfer function form of ADRC, functionally identical to StateSpace without limiters. The controller only needs the measurement and the reference::

    adrc_tf = pyadrc.TransferFunction(order, delta, b0, w_cl, k_eso)
    ctrl = adrc_tf(y, r)

FeedbackTF
##########

Minimum-footprint implementation with rate and magnitude limiters::

    adrc_fbtf = pyadrc.FeedbackTF(order, delta, b0, w_cl, k_eso,
                                  r_lim=r_lim, m_lim=m_lim)
    ctrl = adrc_fbtf(y, r, zoh=False)

TrackingDifferentiator
######################

Han's tracking differentiator turns a reference signal with jumps into a smooth transient profile ``v1`` with derivative ``v2``, limited to the acceleration ``r0``. Feed ``v1`` to any controller as the reference signal; the controllers themselves are unchanged::

    td = pyadrc.TrackingDifferentiator(delta, r0=5.0)

    while True:
        v1, v2 = td(setpoint)
        pv = some_process(ctrl)
        ctrl = adrc_statespace(pv, ctrl, v1)

Choose ``r0`` from what the actuator can deliver. The optional filter factor ``h0`` (by default ``delta``) makes the profile slower and smoother. Use ``td.reset(v1=pv)`` to start from the current process output.

Control loop
############

All controllers within this package have __call__() implemented, that means you call the object itself. Using the example above::

    adrc_statespace(y, u, r, zoh=False)

When the controller is tuned and ready, it is time to create the control loop. Lets imagine you have a process and you are periodically sending input signals and receiving output signals::

    setpoint = 1.
    ctrl = 0.

    while True:
        pv = some_process(ctrl)
        ctrl = adrc_statespace(pv, ctrl, setpoint)

If you set the variable **zoh** to True, the controller will take on a zero-order hold behaviour and wait **delta** sampling seconds before adjusting the control signal::

    setpoint = 1.
    ctrl = 0.

    while True:
        pv = some_process(ctrl)
        ctrl = adrc_statespace(pv, ctrl, setpoint, zoh=True)

Models
######

QuadAltitude
============

Create a quadcopter altitude model using the following code::

    quad = pyadrc.QuadAltitude(dt, m, g)

Send and receive signal using __call__()::

    while True:
        pv = quad(ctrl)


System
======

Create a first- or second-order process using the following code::

    system = pyadrc.System(K, T, D, delta)

If you set parameter **D** to None, the system will be a first-order process, otherwise a second-order.

Send and receive signal using __call__()::

    while True:
        pv = system(ctrl)
