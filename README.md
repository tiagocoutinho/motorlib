# motorlib

[![Pypi Version](https://img.shields.io/pypi/v/motorlib.svg)](https://pypi.python.org/pypi/motorlib)
[![Python Versions](https://img.shields.io/pypi/pyversions/motorlib.svg)](https://pypi.python.org/pypi/motorlib)
[![Build Status](https://gitlab.com/tiagocoutinho/motorlib/badges/master/pipeline.svg)](https://gitlab.com/tiagocoutinho/motorlib/commits/master)
[![Coverage Status](https://gitlab.com/tiagocoutinho/motorlib/badges/master/coverage.svg)](https://gitlab.com/tiagocoutinho/motorlib/commits/master)

A simple motor motion/trajectory simulation python library.

## Installation

pip install it on your favorite python environment:

`$ pip install motorlib`

That's it!

## Usage


```python
>>> import motorlib
>>> motion = motorlib.Motion(10, 1000, 10, 5, (-float('inf'), float('inf')))

>>> motion.position()
38.040199

>>> motion.duration
101.0

>>> motion.accel_time
2.0

>>> motion.pb
990.0

>>> motion.dp
990

>>> motion.reaches_top_vel
True

>>> motion.top_vel_dp
970.0

>>> motion.top_vel_time
97.0
```
