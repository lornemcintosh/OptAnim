#!/usr/bin/env python
# -------------------------------------------------------------------------
# Copyright (c) 2010-2012 Lorne McIntosh
#
# This file is part of OptAnim.
#
# OptAnim is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# OptAnim is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with OptAnim.  If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------------

'''
OptAnim, simple example
This example shows how to produce 3 walk-cycle animations for a simple 3-body character
It should take around 1 minute to solve on a 2010-era 4-core CPU
'''

from __future__ import division
import operator
import logging
from optanim.utils import *
from optanim.animation import *
from optanim.character import *
from optanim.specifier import *
from optanim.joints import *
from optanim.rigidbody import *

LOG = logging.getLogger(__name__)

def test():
	#===========================================================================
	# Character
	#===========================================================================
	#create a simple character with just 3 rigidbodies
	char = Character('Simple')
	
	#define the bodies/joints in character space:
	#x +/- is character front/back (ventral/dorsal)
	#y +/- is character up/down (cranial/caudal)
	#z +/- is character right/left (lateral/lateral)
	
	#define rigidbodies (ellipsoids)
	torso = RigidBody(Id=0, Name="torso", Mass=40.0, Diameter=[0.1, 0.52, 0.1])
	char.add_body(torso)
	thigh = RigidBody(Id=1, Name="thigh", Mass=10.0, Diameter=[0.1, 0.42, 0.1])
	char.add_body(thigh)
	calf = RigidBody(Id=2, Name="calf", Mass=5.0, Diameter=[0.1, 0.43, 0.1])
	char.add_body(calf)
	
	char.set_default_root(torso);	#the root bone for exporting
	
	#define some powered revolute joints
	joint_hip = JointRevolute(Name="joint_hip", BodyA=torso, PointA=torso.ep_b(), BodyB=thigh, PointB=thigh.ep_a(),
		RotationLimits=[[-0.8, 0.8], [-0.8, 0.8], [-1.5, 0.1]], TorqueLimit=280)
	char.add_joint(joint_hip)
	joint_knee = JointRevolute(Name="joint_knee", BodyA=thigh, PointA=thigh.ep_b(), BodyB=calf, PointB=calf.ep_a(),
		RotationLimits=[[0, 0], [0, 0], [0, 2.8]], TorqueLimit=180)
	char.add_joint(joint_knee)
	
	#define an (unpowered) contact joint for the foot
	#(character may push against ground plane with this point)
	joint_floor = JointContact(Name="joint_floor", Body=calf, Point=calf.ep_b(), Friction=0.5)
	char.add_joint(joint_floor)
	
	#character is lazy: minimize energy used (applies to all animations)
	char.add_specifier(SpecifierPluginMinimalTorque(0.01))
	
	#character moves smoothly: minimize joint angular acceleration (i.e. prefer smoother trajectories)
	char.add_specifier(SpecifierPluginMinimalJointAcceleration(0.0025))
	
	#===========================================================================
	# Animations
	#===========================================================================
	#create some walk-cycle animations with this character (3 different speeds)
	anim = ParameterSpace(Name='walk', FPS=20, Length=1.0)
	anim.add_character(char)
	
	#specify a floor contact starting at 0%, lasting for 70% of the animation
	anim.set_contact_times({
		joint_floor:[(0.0, 0.7)]
	})
	
	#specify a ground plane (floor) and forbid character from intersecting it
	anim.add_dimension([[SpecifierPluginGroundPlane()]])
	
	#specify that the animation should start at the origin (otherwise it may be 500 km away somewhere)
	anim.add_dimension([[ConstraintEq("startTorsoNearOrigin", torso.tx(t)**2 + torso.tz(t)**2, 0.0, TimeRange='t = 0')]])
	
	#specify that the character should generally be upright and face forward (otherwise he may hop sideways or twist etc.)
	anim.add_dimension([[Constraint("faceForward", c=torso.rx(t)**2 + torso.ry(t)**2, ub=0.1**2)]])
	
	#specify a looping/cyclic constraint dimension (with 3 different parameter values)
	anim.add_dimension([[SpecifierPluginLoop([speed, 0, 0, 0, 0, 0], [0, 0, 0])] for speed in [0.3, 0.6, 0.9]])
	
	#generate the animations
	anim.generate()  #non-blocking
	anim.wait_for_results()  #blocking
	
	#Done! if we wanted, we could do some post-processing of the animations here
	return


#===========================================================================
# Application Entry point
#===========================================================================
#this check is necessary for multiprocessing to work
if __name__ == '__main__':

	#setup the logging framework
	import sys
	import logging
	logging.basicConfig(level=logging.INFO, format="%(asctime)s  %(levelname)s  %(message)s")
	LOG.debug(sys.argv)
	LOG.debug('Running Python '+sys.version)

	#run the test function:
	test()