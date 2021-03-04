#!/usr/bin/env python

"""Tests for `pyadrc.TransferFunction` module."""

import pytest
import pyadrc
import time
import numpy as np


@pytest.fixture
def get_adrc_tf_params():
    def _adrc_tf_params(order, delta, b0, w_cl, k_eso, method):
        adrc_tf = pyadrc.TransferFunction(order, delta, b0, w_cl,
                                          k_eso, method)
        return adrc_tf.parameters
    return _adrc_tf_params


@pytest.mark.parametrize("order, delta, b0, w_cl, k_eso", [
    (1, 0.001, 1, 1, 1),
    (1, 0.001, 0.1, 1, 1),
    (1, 0.001, 0.5, 1, 1),
    (1, 0.001, 1.1, 1, 1),
    (1, 0.001, 1.5, 1, 1),
    (1, 0.001, 1, 0.1, 1),
    (1, 0.001, 1, 0.5, 1),
    (1, 0.001, 1, 1.1, 1),
    (1, 0.001, 1, 1.5, 1),
    (1, 0.001, 1, 1, 3),
    (1, 0.001, 1, 1, 5),
    (1, 0.001, 1, 1, 8),
    (1, 0.001, 1, 1, 10),
    (2, 0.001, 1, 1, 1),
    (2, 0.001, 0.1, 1, 1),
    (2, 0.001, 0.5, 1, 1),
    (2, 0.001, 1.1, 1, 1),
    (2, 0.001, 1.5, 1, 1),
    (2, 0.001, 1, 0.1, 1),
    (2, 0.001, 1, 0.5, 1),
    (2, 0.001, 1, 1.1, 1),
    (2, 0.001, 1, 1.5, 1),
    (2, 0.001, 1, 1, 3),
    (2, 0.001, 1, 1, 5),
    (2, 0.001, 1, 1, 8),
    (2, 0.001, 1, 1, 10)
])
def test_calculate_param_var_method(get_adrc_tf_params,
                                    order, delta, b0,
                                    w_cl, k_eso):
    assert get_adrc_tf_params(
        order, delta, b0, w_cl, k_eso, 'general_terms') ==\
        get_adrc_tf_params(order, delta, b0, w_cl, k_eso, 'bandwidth')
