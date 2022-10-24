#!/usr/bin/env python
# -*- coding: UTF-8 -*-
# sudo python setup.py install

from distutils.core import setup, Extension
import numpy as np

ext_modules = [Extension('segment', sources = ['segment.c']) ]

setup(
	name = 'Segment',
	version = '1.0',
	include_dirs = [np.get_include()],
	ext_modules = ext_modules
)