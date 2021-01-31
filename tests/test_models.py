#!/usr/bin/env python

import pytest
import pyadrc


@pytest.fixture
def quadcopter_model():
    quad = pyadrc.QuadAltitude()
    return quad
