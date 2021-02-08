======
pyadrc
======


.. image:: https://img.shields.io/pypi/v/pyadrc.svg
        :target: https://pypi.python.org/pypi/pyadrc

.. image:: https://www.travis-ci.com/onguntoglu/pyadrc.svg?branch=master
    :target: https://www.travis-ci.com/onguntoglu/pyadrc

.. image:: https://readthedocs.org/projects/pyadrc/badge/?version=latest
        :target: https://pyadrc.readthedocs.io/en/latest/?badge=latest
        :alt: Documentation Status

.. image:: https://img.shields.io/github/license/onguntoglu/pyadrc.svg
        :target: https://github.com/onguntoglu/pyadrc/blob/master/LICENSE
        :alt: Licence


.. image:: https://codecov.io/gh/onguntoglu/pyadrc/branch/master/graph/badge.svg?token=V8WT0V43QD
      :target: https://codecov.io/gh/onguntoglu/pyadrc
    


Active Disturbance Rejection Control for Python


A simple, yet powerful control method for when a PID controller just doesn't cut it.


* Free software: MIT license
* Documentation: https://pyadrc.readthedocs.io.


Features
--------

* Discrete linear time invariant active disturbance rejection controller for digital control systems.
* Implementation in state-space representation form of first- and second-order ADRC.
* Quick-start guide and/or theoretical background (for when you don't have anything else to read).
* A first/second-order LTI model generator and a quadcopter altitude model for experimentation, testing and verification.


Installation
------------

Installing **pyadrc** is really simple, just run this command in your terminal:

.. code-block:: console

        pip install pyadrc

or you can clone the repository and install it manually:

.. code-block:: console

        git clone git://github.com/onguntoglu/pyadrc
        cd pyadrc
        python setup.py install