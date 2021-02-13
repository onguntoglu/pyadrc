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


Features
--------

* Discrete linear time invariant active disturbance rejection controller for digital control systems.
* Implementation in state-space representation form of first- and second-order ADRC.
* User guide and/or theoretical background (for when you don't have anything else to read).
* A first/second-order LTI model generator and a quadcopter altitude model for experimentation, testing and verification.


Installation
------------

Installing **pyadrc** is really simple, just run this command in your terminal:

.. code-block:: bash

        pip install pyadrc

or you can clone the repository and install it manually:

.. code-block:: bash

        git clone git://github.com/onguntoglu/pyadrc
        cd pyadrc
        python setup.py install


Guides
------

Although ADRC is more complex than a standard PID controller, it can definitely be applied to various projects with relative ease. It is certainly advantageous to have some background in classical and modern control theory, which are commonly taught in undergradute electrical and/or computer engineering courses.

If you are just interested in using the control method, head over to :ref:`introduction-label`. However, if you want to learn more about the math and control theory, check out :ref:`article-label` (written by me).

Licence
-------

**pyadrc** is licenced under the MIT Licence