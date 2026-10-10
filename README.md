# pyadrc's Documentation

[![PyPI](https://img.shields.io/pypi/v/pyadrc.svg)](https://pypi.python.org/pypi/pyadrc)
[![CI](https://github.com/onguntoglu/pyadrc/actions/workflows/ci.yml/badge.svg)](https://github.com/onguntoglu/pyadrc/actions/workflows/ci.yml)
[![Documentation Status](https://readthedocs.org/projects/pyadrc/badge/?version=latest)](https://pyadrc.readthedocs.io/en/latest/?badge=latest)
[![Licence](https://img.shields.io/github/license/onguntoglu/pyadrc.svg)](https://github.com/onguntoglu/pyadrc/blob/master/LICENSE)
[![Downloads](https://pepy.tech/badge/pyadrc)](https://pepy.tech/project/pyadrc)

Active Disturbance Rejection Control for Python

A simple, yet powerful control method for when a PID controller just doesn't cut it.

## Features

- Discrete linear time invariant active disturbance rejection controller for digital control systems.
- First- and second-order ADRC in state-space, transfer function and minimum-footprint (FeedbackTF) form.
- For practical application - half-gain tuning, magnitude and rate limiter for actuator limitations
- Han's tracking differentiator to shape reference signals and avoid setpoint jumps.
- User guide and/or theoretical background (for when you don't have anything else to read).
- A first/second-order LTI model generator and a quadcopter altitude model for experimentation, testing and verification.

## Installation

Installing **pyadrc** is really simple, just run this command in your terminal:

```bash
pip install pyadrc
```

or you can clone the repository and install it manually:

```bash
git clone https://github.com/onguntoglu/pyadrc
cd pyadrc
pip install .
```

## Licence

**pyadrc** is licenced under the MIT Licence

## Acknowledgements

None of this work would be possible without **Gernot Herbst** and his papers:

- *[Practical Active Disturbance Rejection Control: Bumpless Transfer, Rate Limitation and Incremental Algorithm](https://arxiv.org/abs/1908.04610)*
- *[A Simulative Study on Active Disturbance Rejection Control (ADRC) as a Control Tool for Practitioners](https://arxiv.org/abs/1908.04596)*
- *[Transfer Function Analysis and Implementation of Active Disturbance Rejection Control](https://arxiv.org/abs/2011.01044)*
- *[Half-Gain Tuning for Active Disturbance Rejection Control](https://arxiv.org/abs/2003.03986)*
- *[A Minimum-Footprint Implementation of Discrete-Time ADRC](https://arxiv.org/abs/2104.01943)*
