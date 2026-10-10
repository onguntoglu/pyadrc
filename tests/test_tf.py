#!/usr/bin/env python

"""Tests for `pyadrc.TransferFunction` and `pyadrc.FeedbackTF`."""

import pytest
import pyadrc
import numpy as np


@pytest.mark.parametrize("order", [1, 2])
@pytest.mark.parametrize("delta, b0, w_cl, k_eso", [
    (0.001, 1, 1, 1),
    (0.001, 0.1, 1, 1),
    (0.001, 1.5, 1, 1),
    (0.001, 1, 0.1, 1),
    (0.001, 1, 1.5, 1),
    (0.001, 1, 1, 5),
    (0.001, 1, 1, 10),
    (0.01, 2, 5, 4),
])
def test_check_param_method(order, delta, b0, w_cl, k_eso):

    tf1 = pyadrc.TransferFunction(order, delta, b0, w_cl, k_eso,
                                  method='general_terms')
    tf2 = pyadrc.TransferFunction(order, delta, b0, w_cl, k_eso,
                                  method='bandwidth')
    assert tf1.params == pytest.approx(tf2.params, rel=1e-6)


def _closed_loop(controller, order, steps=600):
    plant = pyadrc.System(K=2.0, T=0.5, D=0.8 if order == 2 else None,
                          delta=0.01)
    y = u = 0.
    ys = []
    for _ in range(steps):
        u = controller(y, u)
        y = plant(u)
        ys.append(y)
    return np.array(ys)


@pytest.mark.parametrize("order", [1, 2])
def test_implementations_agree(order):
    """All ADRC implementations are the same controller, so the closed-loop
    responses on the same plant must match."""

    args = (order, 0.01, 2.0, 5.0, 4.0)
    r = 1.

    ss = pyadrc.StateSpace(*args)
    ss_inc = pyadrc.StateSpace(*args, inc_form=True)
    tf_gt = pyadrc.TransferFunction(*args, method='general_terms')
    tf_bw = pyadrc.TransferFunction(*args, method='bandwidth')
    fbtf = pyadrc.FeedbackTF(*args)

    reference = _closed_loop(lambda y, u: ss(y, u, r), order)

    others = [
        lambda y, u: u + ss_inc(y, u, r),
        lambda y, u: tf_gt(y, r),
        lambda y, u: tf_bw(y, r),
        lambda y, u: fbtf(y, r),
    ]

    assert reference[-1] == pytest.approx(r, abs=1e-3)
    for ctrl in others:
        np.testing.assert_allclose(_closed_loop(ctrl, order), reference,
                                   atol=1e-9)
