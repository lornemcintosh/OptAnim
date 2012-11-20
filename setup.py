#!/usr/bin/env python

from setuptools import setup

setup(
    name = 'optanim',
	version = '0.1',
    description = 'A library for the procedural generation of 3D skeletal character animations',
	license = 'GPL',
    keywords = ['graphics', 'procedural', 'generation', '3D', 'skeletal', 'character', 'animation', 'videogame', 'optimal', 'spacetime', 'constraints', 'control', 'non-linear', 'optimization', 'parameter', 'space', 'physics'],
    author = 'Lorne McIntosh',
    author_email = 'lorne@lorneswork.com',
	url = 'https://github.com/lornemcintosh/optanim',
    #package_dir = {'': 'src'},
	packages = ['optanim'],
    #test_suite = 'your.module.tests',
	classifiers = [
        "Programming Language :: Python",
        "Development Status :: 2 - Pre-Alpha",
        "Environment :: Console",
        "Intended Audience :: Developers",
		"Intended Audience :: Science/Research",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: OS Independent",
        "Topic :: Software Development :: Libraries :: Python Modules",
		"Topic :: Scientific/Engineering :: Physics",
		"Topic :: Scientific/Engineering :: Artificial Intelligence",
        "Topic :: Multimedia :: Graphics",
        ]
)