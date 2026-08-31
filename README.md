# pyadrc's Documentation

[![PyPI](https://img.shields.io/pypi/v/pyadrc.svg)](https://pypi.python.org/pypi/pyadrc)
[![Build Status](https://www.travis-ci.com/onguntoglu/pyadrc.svg?branch=master)](https://www.travis-ci.com/onguntoglu/pyadrc)
[![Documentation Status](https://readthedocs.org/projects/pyadrc/badge/?version=latest)](https://pyadrc.readthedocs.io/en/latest/?badge=latest)
[![Codecov](https://codecov.io/gh/onguntoglu/pyadrc/branch/master/graph/badge.svg?token=V8WT0V43QD)](https://codecov.io/gh/onguntoglu/pyadrc)
[![Licence](https://img.shields.io/github/license/onguntoglu/pyadrc.svg)](https://github.com/onguntoglu/pyadrc/blob/master/LICENSE)
[![Downloads](https://pepy.tech/badge/pyadrc)](https://pepy.tech/project/pyadrc)

Active Disturbance Rejection Control for Python

A simple, yet powerful control method for when a PID controller just doesn't cut it.

## Features

- Discrete linear time invariant active disturbance rejection controller for digital control systems.
- Implementation in state-space representation form of first- and second-order ADRC.
- For practical application - half-gain tuning, magnitude and rate limiter for actuator limitations
- User guide and/or theoretical background (for when you don't have anything else to read).
- A first/second-order LTI model generator and a quadcopter altitude model for experimentation, testing and verification.

## Installation

Installing **pyadrc** is really simple, just run this command in your terminal:

```bash
pip install pyadrc
```

or you can clone the repository and install it manually:

```bash
git clone git://github.com/onguntoglu/pyadrc
cd pyadrc
python setup.py install
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
