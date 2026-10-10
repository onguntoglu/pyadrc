=======
History
=======

0.6.1 (2026-10-10)
------------------

* ``FeedbackTF`` now uses its documented ``x_init`` argument (it was
  ignored).
* Remove ``pyadrc.models.RandomSystem`` (never exported or documented).
* Docs render both notebooks; stale pages removed.
* Internal: one shared rate/magnitude limiter, type hint fixes, faster
  tests.

0.6.0 (2026-10-10)
------------------

* Add ``TrackingDifferentiator`` and ``fhan`` (Han 2009, eqs. (10)-(11)) to
  shape reference signals. Uses ``d = r0 * h0**2`` in ``fhan``; the paper
  prints ``d = h0 * r0**2``, which does not give the time-optimal transient
  it describes.

0.5.2 (2026-10-10)
------------------

* Fix incremental ``StateSpace``: ``reset()`` no longer causes a jump in
  the control signal (regression in 0.5.1), and ``zoh=True`` holds return
  a zero increment instead of repeating the last one.
* ``StateSpace.reset()`` accepts any iterable again.
* Quadcopter notebook: physical displacement and thrust-loss disturbances,
  text matches the code.

0.5.1 (2026-10-10)
------------------

* Apply rate and magnitude limiters in incremental ``StateSpace``
  (``inc_form=True``), following eqs. (22)-(24) of Herbst,
  arXiv:1908.04610. Matches the normal form in closed loop (#25).
* Raise ``ValueError`` instead of ``assert`` for invalid arguments.
* ``QuadAltitude`` and ``System`` accept ints for float parameters.

0.5.0 (2026-10-10)
------------------

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
