#!/usr/bin/env pybricks-micropython
"""Stop all motors (use when a run was cancelled and a motor kept running)."""
from pybricks.ev3devices import Motor
from pybricks.parameters import Port

for port in (Port.A, Port.B, Port.D):
    try:
        Motor(port).stop()
    except OSError:
        pass
print('motors stopped')
