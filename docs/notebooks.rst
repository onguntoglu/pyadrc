.. _notebooks-label:

=========
Notebooks
=========

Altitude Control of a Quadcopter
================================

    * Basics - StateSpace
        * Demonstrates how to create StateSpace controller, and how to design a control loop with the API
        * The importance of the modelling parameter is indicated along with possibly scenarios
        * See how ADRC rejects disturbances.

    * Closed-loop dynamics
        * Examines how controller and observer poles are placed using bandwidth parametrization
        * The effects of noise are investigated.
        * Half-gain tuning is introduced.

    * Magnitude and rate limiters
        * Limiters and their significance for practical implementation are discussed. 

Tracking Differentiator
=======================

    * Shaping the reference signal with Han's tracking differentiator
        * The time-optimal transient profile and the effect of r0 and h0
        * Quadcopter altitude steps: peak thrust with and without the tracking differentiator
