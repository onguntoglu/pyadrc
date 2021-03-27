=====
Usage
=====

To use pyadrc in a project::

    import pyadrc

StateSpace
##########

After importing pyadrc, create StateSpace controller::

    adrc_statespace = pyadrc.StateSpace(order, delta, b0, t_settle, k_eso)

You can find out how to select the parameters by checking :ref:`introduction-label` or :ref:`api-label`. Examples can be found in :ref:`notebooks-label`. 

If you want to configure the initial values for ESO, rate and magnitude limiter and half-gain tuning, tune the parameters at object creation or use the getter/setters to turn them on/off within the control loop::

    adrc_statespace = pyadrc.StateSpace(order, delta, b0, t_settle,
                                        k_eso, eso_init, r_lim, m_lim, half_gain)

TransferFunction
################

Transfer function form of ADRC has not been fully implemented and not functional. Use at your own risk! Check out :ref:`api-label` if you want to use it anyway.

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
