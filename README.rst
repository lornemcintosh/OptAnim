#######
OptAnim
#######

OptAnim ("Optimal Animation") is a Python library for the procedural generation of 3D skeletal character animations. It uses the principles of Optimal Control, coupled with a simple but powerful parameter-space system to ease the generation of large quantities of physically-accurate animations.

Features
========
- use virtually any character morphology
- physically accurate animation
- scalable: produce hundreds of animations quickly
- flexible: easily accommodate design changes
- precise: satisfy technical animation requirements exactly
- seamless animation looping (for walk cycles etc.)
- parallel processing on multi-core machines
- outputs .BVH and Ogre3D skeleton.XML files


Dependencies
============
- Python 2.6 (http://www.python.org/download/releases/2.6.6/)
- SymPy 0.7.2 (http://sympy.org/en/index.html) or ``$ pip install sympy``
- NumPy 1.6.2 (http://numpy.scipy.org/) or ``$ pip install numpy``
- cgkit 2.0.0 alpha 9 (http://sourceforge.net/projects/cgkit/files/cgkit/)
- AMPL (http://www.ampl.com/) (commercial - help remove this dependency!)
- IPOPT 3.10.1 (http://www.coin-or.org/download/binary/Ipopt/)

*Other versions of these may also work. Try your luck.*


Installation
============
First install Python, SymPy, NumPy and cgkit. Install the AMPL and IPOPT executables on your system, and make sure they can be accessed via the system PATH variable. Now download the OptAnim source and run::

  $ python setup.py install

You can test the install by running one of the examples::

  $ cd examples
  $ python ./thesis_example.py


Example Usage
=============
Define your character by connecting rigid-bodies with joints:

>>> todo

Create an animation using constraints and objectives:

>>> todo

Now make a parameter-space with dozens of animations:

>>> todo


Documentation
=============
There's no official documentation yet. For now, this video provides a quick overview:
http://www.youtube.com/watch?v=HS1h9EcvPlo
and my thesis has many more details:
https://theses.lib.sfu.ca/thesis/etd6856


Contributing
============
I'd like to get feedback from animators, developers and researchers alike. Tell me what you think.
Or jump right in, fork the repository and try something (there are some ideas in the TODO file). And send me a pull request!


License
=======
OptAnim is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

OptAnim is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with OptAnim.  If not, see <http://www.gnu.org/licenses/>.


Copyright
=========
| Copyright (c) 2010-2012, Lorne McIntosh
| http://lorneswork.com/