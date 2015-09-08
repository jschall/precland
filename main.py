#!/usr/bin/python
from estimator import TargetEstimator
import numpy as np

from visual import *
from visual.graph import *

Px = 0
Pz = -10

Tx = 8
Tz = 2


dt = .2

estimator = TargetEstimator()

dt = .1
t = 0.
while True:
    rate(10)
    Px += sin(t/5.)/10
    print Px
    print estimator.state, "\n"
    estimator.predict(dt)
    estimator.update([Px, Pz, atan2(Tx-Px,-Pz-(-Tz))])

    t += dt
