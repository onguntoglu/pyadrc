=======
History
=======

Unreleased
----------

* Fix crash with NumPy 2 (``StateSpace`` and models returned 1x1 arrays).
* Fix first-order ``StateSpace`` proportional gain (was ``2 * w_cl``,
  now ``w_cl``), matching ``TransferFunction`` and ``FeedbackTF``.
* Fix ``TransferFunction`` coefficients: first-order ``bandwidth`` prefilter,
  second-order ``general_terms`` prefilter and second-order ``bandwidth``
  ``be2``. All implementations now produce the same closed-loop response.
* Remove unused ``eso_init``, ``r_lim``, ``m_lim`` and ``half_gain`` arguments
  from ``TransferFunction``.
* Export ``FeedbackTF`` and ``saturation`` in ``__all__``.
* Require Python 3.10+, move packaging to ``pyproject.toml``, CI to GitHub
  Actions, lint to ruff.

0.1.0 (2021-01-30)
------------------

* First release on PyPI.
