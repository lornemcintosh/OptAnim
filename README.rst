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
- Python 2.7.4 (http://www.python.org/download/releases/2.7.4)
- SymPy 0.7.2 (http://sympy.org/en/index.html) or ``$ pip install sympy``
- NumPy 1.7.1 (http://numpy.scipy.org/) or ``$ pip install numpy``
- cgkit 2.0.0 (http://sourceforge.net/projects/cgkit/files/cgkit/cgkit-2.0.0/)
- AMPL (http://www.ampl.com/) (commercial - help remove this dependency!)
- IPOPT 3.11.0 (http://www.coin-or.org/download/binary/Ipopt/)

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
Define characters by connecting rigid-bodies with joints. In this simple example we'll just use a single body:

>>> char = Character('Basketball')
>>> body = RigidBody(Id=0, Name='BasketballBody', Mass=0.6237, Diameter=[0.239, 0.239, 0.239])
>>> char.add_body(body)

Create parameter spaces to generate animations for characters:

>>> anim = ParameterSpace(Name='ShotTrajectory', FPS=30, Length=2.0)
>>> anim.add_character(char)

For each parameter space, specify what animations we want using constraints and objectives functions:

>>> #for a ball starting at each of these coordinates:
>>> anim.add_dimension([[ConstraintEq("Xstart", body.tx(0), x)] for x in range(2,8)])
>>> anim.add_dimension([[ConstraintEq("Ystart", body.ty(0), 0.0)]])
>>> anim.add_dimension([[ConstraintEq("Zstart", body.tz(0), z)] for z in range(-3,4)])
>>> #and passing through the hoop on frame 50
>>> anim.add_dimension([[ConstraintEq("Xend", body.tx(50), 0.0), ConstraintEq("Yend", body.ty(50), 3.0), ConstraintEq("Zend", body.tz(50), 0.0)]])

Generate the animations:

>>> anim.generate()
>>> anim.wait_for_results()


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