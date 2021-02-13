.. _introduction-label:

Introduction
============

**Active disturbance rejection control** (ADRC) is a model-free, robust control method, combining the simplicity of PID controllers and the powerful state observers from modern control theory.

The basic premise of ADRC is to approximate a plant using low-order processes (mostly limited to first and second-orders) and extend this approximation by a virtual state, which is then estimated online using a state observer (usually called an **extended state observer** (ESO) in ADRC context).

The estimated virtual state is the **total disturbance** (i.e. not accounted by our approximation using low-order processes) which acts on the plant, including any nonlinearity unknown to the designer. The estimated total disturbance is used then in the state-feedback to **decouple the plant**, resulting in a state-feedback controller which rejects the total disturbance in real-time.

Quick-start
-----------

So, imagine a quadcopter, and for some reason, you need the drone to hover at a certain altitude. 

