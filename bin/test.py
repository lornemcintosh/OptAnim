#!/usr/bin/env python
#
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
OptAnim, test module
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

def main():
    #bodies/joints are defined in character space:
    #right-handed coordinate system
    #x+- is character front, back (ventral, dorsal)
    #y+- is character up, down (cranial, caudal)
    #z+- is character right, left (lateral, lateral)

    #define a new character, Mortimer the mannequin
    char_mortimer = Character('Mortimer')
    torso = RigidBody(0, "C_torso", 26.05, [0.21, 0.52, 0.27])
    char_mortimer.add_body(torso)
    thigh_left = RigidBody(3, "L_leg_upper", 6.41, [0.16, 0.42, 0.16])
    char_mortimer.add_body(thigh_left)
    calf_left = RigidBody(7, "L_leg_lower", 3.13, [0.11, 0.43, 0.11]) #length includes foot
    char_mortimer.add_body(calf_left)
    thigh_right = RigidBody(1, "R_leg_upper", 6.41, [0.16, 0.42, 0.16])
    char_mortimer.add_body(thigh_right)
    calf_right = RigidBody(5, "R_leg_lower", 3.13, [0.11, 0.43, 0.11]) #length includes foot
    char_mortimer.add_body(calf_right)

    R_arm_upper = RigidBody(2, "R_arm_upper", 2.5, [0.076, 0.29, 0.076])
    char_mortimer.add_body(R_arm_upper)
    L_arm_upper = RigidBody(4, "L_arm_upper", 2.5, [0.076, 0.29, 0.076])
    char_mortimer.add_body(L_arm_upper)
    R_arm_lower = RigidBody(6, "R_arm_lower", 1.98, [0.07, 0.42, 0.07]) #length includes hand
    char_mortimer.add_body(R_arm_lower)
    L_arm_lower = RigidBody(8, "L_arm_lower", 1.98, [0.07, 0.42, 0.07]) #length includes hand
    char_mortimer.add_body(L_arm_lower)


    #define some joints to constrain the bodies together
    joint_hip_left = JointRevolute("L_hip", torso, [0.0, torso.ep_b()[1], -0.1], thigh_left, thigh_left.ep_a(), [[-0.8, 0.4], [-0.6, 0.8], [-1.5, 0.1]], 280)
    char_mortimer.add_joint(joint_hip_left)
    joint_knee_left = JointRevolute("L_knee", thigh_left, thigh_left.ep_b(), calf_left, calf_left.ep_a(), [[0,0], [0,0], [0, 2.8]], 180)
    char_mortimer.add_joint(joint_knee_left)
    joint_hip_right = JointRevolute("R_hip", torso, [0.0, torso.ep_b()[1], 0.1], thigh_right, thigh_right.ep_a(), [[-0.4, 0.8], [-0.8, 0.6], [-1.5, 0.1]], 280)
    char_mortimer.add_joint(joint_hip_right)
    joint_knee_right = JointRevolute("R_knee", thigh_right, thigh_right.ep_b(), calf_right, calf_right.ep_a(), [[0,0], [0,0], [0, 2.8]], 180)
    char_mortimer.add_joint(joint_knee_right)

    joint_shoulder_right = JointRevolute("R_shoulder", torso, [torso.ep_a()[0]-0.03, torso.ep_a()[1]-0.05, torso.ep_a()[2]+0.16], R_arm_upper, R_arm_upper.ep_a(), [[-0.5, 0.1], [-0.3, 0.3], [-0.8, 0.8]], 8)
    char_mortimer.add_joint(joint_shoulder_right)
    joint_shoulder_left = JointRevolute("L_shoulder", torso, [torso.ep_a()[0]-0.03, torso.ep_a()[1]-0.05, torso.ep_a()[2]-0.16], L_arm_upper, L_arm_upper.ep_a(), [[-0.1, 0.5], [-0.3, 0.3], [-0.8, 0.8]], 8)
    char_mortimer.add_joint(joint_shoulder_left)
    joint_elbow_right = JointRevolute("R_elbow", R_arm_upper, R_arm_upper.ep_b(), R_arm_lower, R_arm_lower.ep_a(), [[0,0], [0,0], [-2.0, 0]], 5)
    char_mortimer.add_joint(joint_elbow_right)
    joint_elbow_left = JointRevolute("L_elbow", L_arm_upper, L_arm_upper.ep_b(), L_arm_lower, L_arm_lower.ep_a(), [[0,0], [0,0], [-2.0, 0]], 5)
    char_mortimer.add_joint(joint_elbow_left)
    

    #define an (unpowered) contact joint for each foot
    #(character may push against ground plane with these points)
    joint_foot_left = JointContact("L_foot", calf_left, calf_left.ep_b(), Friction=0.5)
    char_mortimer.add_joint(joint_foot_left)
    joint_foot_right = JointContact("R_foot", calf_right, calf_right.ep_b(), Friction=0.5)
    char_mortimer.add_joint(joint_foot_right)

    char_mortimer.set_default_root(torso)

    #===========================================================================
    # General character preferences (for all animations)
    #===========================================================================
    #minimize energy used
    char_mortimer.add_specifier(SpecifierPluginMinimalTorque(0.01))

    #minimize joint angular acceleration (i.e. prefer smooth trajectories)
    char_mortimer.add_specifier(SpecifierPluginMinimalJointAcceleration(0.0025))

    #minimize rotation of legs on X and Y
    #(people find this orientation more comfortable)
    char_mortimer.add_specifier(Objective("thighPreferenceX",
	joint_hip_left.get_angle_expr(t)[0]**2 + joint_hip_right.get_angle_expr(t)[0]**2, 200.0))
    char_mortimer.add_specifier(Objective("thighPreferenceY",
	joint_hip_left.get_angle_expr(t)[1]**2 + joint_hip_right.get_angle_expr(t)[1]**2, 200.0))

    #minimize rotation of arms
    #(people find this orientation more comfortable)
    char_mortimer.add_specifier(Objective("shoulderPreferenceX",
	joint_shoulder_left.get_angle_expr(t)[0]**2 + joint_shoulder_right.get_angle_expr(t)[0]**2, 200.0))
    char_mortimer.add_specifier(Objective("shoulderPreferenceY",
	joint_shoulder_left.get_angle_expr(t)[1]**2 + joint_shoulder_right.get_angle_expr(t)[1]**2, 200.0))
    char_mortimer.add_specifier(Objective("shoulderPreferenceZ",
	joint_shoulder_left.get_angle_expr(t)[2]**2 + joint_shoulder_right.get_angle_expr(t)[2]**2, 100.0))

    #and because the arms aren't very constrained, we'll "lead" the solution a bit:
    #arms should swing in synch with opposite leg (match their Z angular velocity)
    char_mortimer.add_specifier(Objective("shoulderSwingRightZ", (joint_shoulder_right.get_angle_velocity_expr(t)[2] - joint_hip_left.get_angle_velocity_expr(t)[2])**2, 300.0))
    char_mortimer.add_specifier(Objective("shoulderSwingLeftZ", (joint_shoulder_left.get_angle_velocity_expr(t)[2] - joint_hip_right.get_angle_velocity_expr(t)[2])**2, 300.0))


    #===========================================================================
    # Walk Animations
    #===========================================================================
    anim_walk = ParameterSpace(Name='walk', FPS=20)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    speed = 1.35 #m/s
    time_contact = guess_contact_time(0.85, speed)
    period = 1.0 #s
    
    anim_walk.set_length(period)
    anim_walk.set_contact_times({
    	joint_foot_left:[(0.5, time_contact/period)],	#contact starting at x%, lasting for y%
    	joint_foot_right:[(0.0, time_contact/period)]	#contact starting at x%, lasting for y%
    })

    #without and with "limp" (right contact has force limit)
    #anim_walk.add_dimension([[None], ["limping", Constraint("rightLegLimp", c=joint_foot_right.fty(t), ub=char_mortimer.get_mass() * 9.81 * 0.6)]])

    #straight
    c_straight = [
        #[SpecifierPluginLoop([speed, 0, 0, 0, 0, 0], [0, 0, 0]),
	#Constraint("startTorsoNearOrigin", c=torso.tx(t) ** 2 + torso.tz(t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')],

        [SpecifierPluginLoop([speed, 0, 0, 0, 0, 0], [0, 0, 0]),
	#Constraint("torso_ry", lb=-0.1, c=torso.ry(t), ub=0.1),
        ConstraintEq("torso_ry", torso.ry(t), 0),
	Constraint("startTorsoNearOrigin", c=torso.tx(t) ** 2 + torso.tz(t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')]
        ]

    #turning
    turnRadii = [x*x*x for x in numpy.arange(0.3, 2.0, 0.2)]
    #turnRadii += [-x for x in turnRadii]
    #print turnRadii
    c_turn = [[
	SpecifierPluginLoop([0, 0, 0, 0, 0, 0], [0, min(speed/r, 2.0), 0]),
	ConstraintEq("startTorso_tx", torso.tx(t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tz", torso.tz(t), r, TimeRange='t = 0'),
        ConstraintEq("torso_ry", torso.ry(t), min(speed/r, 2.0)*(t/anim_walk.FPS))] for r in turnRadii]

    anim_walk.add_dimension(c_straight + c_turn)

    #stay above ground plane
    anim_walk.add_dimension([[SpecifierPluginGroundPlane()]])

    anim_walk.add_dimension([[SpecifierPluginMinimalJointVelocity(0.5)]])

    #some "preferences":
    #minimize rotation of torso on x and z axes (people tend to walk upright)
    anim_walk.add_dimension([[Objective("uprightPreference",
	torso.rx(t)**2 + torso.rz(t)**2, 500.0)]])

    anim_walk.add_dimension([[Objective("elbowPreferenceZ",
        joint_elbow_left.get_angle_expr(t)[2]**2 + joint_elbow_right.get_angle_expr(t)[2]**2, 50.0)]])

    anim_walk.add_character(char_mortimer)
    anim_walk.generate()
    
    #===========================================================================
    # Run Animations
    #===========================================================================
    anim_run = ParameterSpace(Name='run', FPS=37)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    speed = 3.1 #m/s
    time_contact = guess_contact_time(0.85, speed)
    period = 0.75 #s

    anim_run.set_length(period)
    anim_run.set_contact_times({
    	joint_foot_left:[(0.5, time_contact/period)],	#contact starting at x%, lasting for y%
    	joint_foot_right:[(0.0, time_contact/period)]	#contact starting at x%, lasting for y%
    })

    #straight
    c_straight = [
        [SpecifierPluginLoop([speed, 0, 0, 0, 0, 0], [0, 0, 0]),
        #Constraint("torso_ry", lb=-0.1, c=torso.ry(t), ub=0.1),
        ConstraintEq("torso_rx", torso.rx(t), 0),
        ConstraintEq("torso_ry", torso.ry(t), 0),
	Constraint("startTorsoNearOrigin", c=torso.tx(t) ** 2 + torso.tz(t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')]]

    #turning
    turnRadii = [x*x*x for x in numpy.arange(1.5, 2.0, 0.2)]
    #turnRadii += [-x for x in turnRadii]
    #print turnRadii
    c_turn = [[
	SpecifierPluginLoop([0, 0, 0, 0, 0, 0], [0, min(speed/r, 2.25), 0]),
	ConstraintEq("startTorso_tx", torso.tx(t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tz", torso.tz(t), r, TimeRange='t = 0'),
        ConstraintEq("torso_ry", torso.ry(t), min(speed/r, 2.25)*(t/anim_run.FPS))] for r in turnRadii]

    anim_run.add_dimension(c_straight + c_turn)

    #stay above ground plane
    anim_run.add_dimension([[SpecifierPluginGroundPlane()]])

    anim_run.add_dimension([[SpecifierPluginMinimalJointVelocity(0.25)]]) #half the normal, since run should be a high-velocity motion

    #some "preferences":
    #minimize rotation of torso on z axis (people tend to walk upright)
    anim_run.add_dimension([[Objective("uprightPreferenceZ",
	torso.rz(t)**2, 500.0)]])

    #keep elbow at -1.9 radians (people run with their elbows bent like this)
    anim_run.add_dimension([[Objective("elbowPreferenceZ",
        (joint_elbow_left.get_angle_expr(t)[2]+1.9)**2 + (joint_elbow_right.get_angle_expr(t)[2]+1.9)**2, 400.0)]])

    #keep knees bent (to prevent "scraping the ground" during swing phase)
    anim_run.add_dimension([[Constraint("keepLeftKneeBent",
        lb=1.8, c=joint_knee_left.get_angle_expr(t)[2], TimeRange='t in sTimeSteps_R_footOn')]])
    anim_run.add_dimension([[Constraint("keepRightKneeBent",
        lb=1.8, c=joint_knee_right.get_angle_expr(t)[2], TimeRange='t in sTimeSteps_L_footOn')]])

    anim_run.add_character(char_mortimer)
    anim_run.generate()

    #===========================================================================
    # Walk Limp Animations
    #===========================================================================
    anim_walklimp = ParameterSpace(Name='walklimp', FPS=20)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    speed = 1.35 #m/s
    time_contact = guess_contact_time(0.85, speed)
    period = 1.0 #s

    anim_walklimp.set_length(period)
    anim_walklimp.set_contact_times({
    	joint_foot_left:[(0.5, (time_contact/period)*1.5)],	#contact starting at x%, lasting for y%
    	joint_foot_right:[(0.0, (time_contact/period))]	#contact starting at x%, lasting for y%
    })

    #with "limp" (right contact has force limit)
    anim_walklimp.add_dimension([["limping", Constraint("rightLegLimp", c=joint_foot_right.fty(t), ub=char_mortimer.get_mass() * 9.81 * 0.6)]])

    #straight
    c_straight = [
        #[SpecifierPluginLoop([speed, 0, 0, 0, 0, 0], [0, 0, 0]),
	#Constraint("startTorsoNearOrigin", c=torso.tx(t) ** 2 + torso.tz(t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')],

        [SpecifierPluginLoop([speed, 0, 0, 0, 0, 0], [0, 0, 0]),
	#Constraint("torso_ry", lb=-0.1, c=torso.ry(t), ub=0.1),
        ConstraintEq("torso_ry", torso.ry(t), 0),
	Constraint("startTorsoNearOrigin", c=torso.tx(t) ** 2 + torso.tz(t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')]
        ]

    #turning
    turnRadii = [x*x*x for x in numpy.arange(0.3, 2.0, 0.2)]
    turnRadii += [-x for x in turnRadii]
    #print turnRadii
    c_turn = [[
	SpecifierPluginLoop([0, 0, 0, 0, 0, 0], [0, min(speed/r, 2.0), 0]),
	ConstraintEq("startTorso_tx", torso.tx(t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tz", torso.tz(t), r, TimeRange='t = 0'),
        ConstraintEq("torso_ry", torso.ry(t), min(speed/r, 2.0)*(t/anim_walklimp.FPS))] for r in turnRadii]

    anim_walklimp.add_dimension(c_straight + c_turn)

    #stay above ground plane
    anim_walklimp.add_dimension([[SpecifierPluginGroundPlane()]])

    anim_walklimp.add_dimension([[SpecifierPluginMinimalJointVelocity(0.5)]])

    #some "preferences":
    #minimize rotation of torso on x and z axes (people tend to walk upright)
    anim_walklimp.add_dimension([[Objective("uprightPreference",
	torso.rx(t)**2 + torso.rz(t)**2, 500.0)]])

    anim_walklimp.add_dimension([[Objective("elbowPreferenceZ",
        joint_elbow_left.get_angle_expr(t)[2]**2 + joint_elbow_right.get_angle_expr(t)[2]**2, 50.0)]])

    anim_walklimp.add_character(char_mortimer)
    anim_walklimp.generate()


    #===========================================================================
    # Walk Spin Animations
    #===========================================================================
    anim_walkspin = ParameterSpace(Name='walkspin', FPS=20)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    time_contact = 0.45
    period = 2.0 #s

    anim_walkspin.set_length(period)
    anim_walkspin.set_contact_times({
    	joint_foot_left:[(0.25, time_contact/period), (0.75, time_contact/period)],	#contact starting at x%, lasting for y%
    	joint_foot_right:[(0.0, time_contact/period), (0.5, time_contact/period)]	#contact starting at x%, lasting for y%
    })

    #straight
    c_straight = [
        #[SpecifierPluginLoop([speed, 0, 0, 0, 0, 0], [0, 0, 0]),
	#Constraint("startTorsoNearOrigin", c=torso.tx(t) ** 2 + torso.tz(t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')],

        [SpecifierPluginLoop([speed, 0, 0, 0, turnrate, 0], [0, 0, 0]),
	#Constraint("torso_ry", lb=-0.1, c=torso.ry(t), ub=0.1),
        #ConstraintEq("torso_ry", torso.ry(0), 0),
        Constraint("torso_ry", lb=(turnrate*(t/anim_walkspin.FPS))-0.25, c=torso.ry(t), ub=(turnrate*(t/anim_walkspin.FPS))+0.25), #turn evenly
        Constraint("torso_tx", lb=(speed*(t/anim_walkspin.FPS))-0.15, c=torso.tx(t), ub=(speed*(t/anim_walkspin.FPS))+0.15),  #move evenly
        ConstraintEq("torso_tz", torso.tz(0), 0)] for speed in [0.5, 1.0] for turnrate in [-math.pi, math.pi]
        ]

    anim_walkspin.add_dimension(c_straight)

    #stay above ground plane
    anim_walkspin.add_dimension([[SpecifierPluginGroundPlane()]])

    anim_walkspin.add_dimension([[SpecifierPluginMinimalJointVelocity(0.5)]])

    #some "preferences":
    #minimize rotation of torso on x and z axes (people tend to walk upright)
    anim_walkspin.add_dimension([[Objective("uprightPreference",
	torso.rx(t)**2 + torso.rz(t)**2, 1000.0)]])

    anim_walkspin.add_dimension([[Objective("elbowPreferenceZ",
        joint_elbow_left.get_angle_expr(t)[2]**2 + joint_elbow_right.get_angle_expr(t)[2]**2, 50.0)]])

    #minimize acceleration of torso
    #torsoAcc = torso.get_acceleration_expr(t)
    #torsoAcc = [x**2 for x in torso.get_acceleration_expr(t)]
    #anim_walkspin.add_dimension([[Objective("minimalTorsoAccelerationT",
        #sum(torsoAcc[:3]), 0.5, 't>pTimeBegin && t<pTimeEnd')]])
    #anim_walkspin.add_dimension([[Objective("minimalTorsoAccelerationR",
        #sum(torsoAcc[3:dof]), 0.007, 't>pTimeBegin && t<pTimeEnd')]])

    anim_walkspin.add_dimension([[Objective("minimalZ", torso.tz(t)**2, 1000.0)]])

    #without and with "limp" (right contact has force limit)
    anim_walkspin.add_dimension([[None], ["limping", Constraint("rightLegLimp", c=joint_foot_right.fty(t), ub=char_mortimer.get_mass() * 9.81 * 0.6)]])

    anim_walkspin.add_character(char_mortimer)
    anim_walkspin.generate()

    #===========================================================================
    # Start / Stop Locomotion
    #===========================================================================
    anim_stepping = ParameterSpace(Name='stepping', FPS=20)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    anim_stepping.set_length(1.8)
    anim_stepping.set_contact_times({
        joint_foot_left:[(0.0, 0.2018), (0.3893, 0.225), (0.8018, 0.1982)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.0, 0.3893), (0.6143, 0.3857)],	#contact starting at x, lasting for y
    })

    speed = 0.6 #m/s
    coords = [[math.cos(fracCircle*2*math.pi)*speed, math.sin(fracCircle*2*math.pi)*speed] for fracCircle in numpy.arange(0, 1, 1/8)]
    print coords
    c_walk = [[SpecifierPluginLoop([x, 0, z, 0, 0, 0], [0, 0, 0]),
	#Constraint("faceforwards", lb=-0.15, c=torso.ry(t), ub=0.15),
        ConstraintEq("faceforwards", torso.ry(t), 0),
        ConstraintEq("stoppedTorso_RX", torso.rx(0), 0),
        ConstraintEq("stoppedTorso_RZ", torso.rz(0), 0),

        ConstraintEq("stoppedTorso_X0", torso.tx(0), 0),
        ConstraintEq("stoppedTorso_Z0", torso.tz(0), 0),
        ConstraintEq("stoppedTorso_X1", torso.tx(1), 0),
        ConstraintEq("stoppedTorso_Z1", torso.tz(1), 0),
        #ConstraintEq("stoppedTorso_X2", torso.tx(2), 0),
        #ConstraintEq("stoppedTorso_Z2", torso.tz(2), 0),

        ConstraintEq("stoppedCL_RY", thigh_left.ry(0), 0),
        ConstraintEq("stoppedCR_RY", thigh_right.ry(0), 0),
        ConstraintEq("stoppedCL_RX", thigh_left.rx(0), 0),
        ConstraintEq("stoppedCR_RX", thigh_right.rx(0), 0),

        ConstraintEq("stoppedR_arm_upper_RY", R_arm_upper.ry(0), 0),
        ConstraintEq("stoppedL_arm_upper_RY", L_arm_upper.ry(0), 0),

        ConstraintEq("stoppedR_arm_upper_RZ", R_arm_upper.rz(0), 0),
        ConstraintEq("stoppedL_arm_upper_RZ", L_arm_upper.rz(0), 0),

        #ConstraintEq("stoppedCL_RZ", calf_left.rz(0), 0),
        #ConstraintEq("stoppedCR_RZ", calf_right.rz(0), 0),

        ConstraintEq("stoppedCL_X0", calf_left.tx(0), 0),
        ConstraintEq("stoppedCR_X0", calf_right.tx(0), 0)] for x,z in coords]
        #ConstraintEq("stoppedCL_X1", calf_left.tx(1), 0),
        #ConstraintEq("stoppedCR_X1", calf_right.tx(1), 0)]

    anim_stepping.add_dimension(c_walk)

    #stay above ground plane
    anim_stepping.add_dimension([[SpecifierPluginGroundPlane()]])

    anim_stepping.add_dimension([[SpecifierPluginMinimalJointVelocity(0.5)]])

    #some "preferences":
    #minimize rotation of torso on x and z axes (people tend to walk upright)
    anim_stepping.add_dimension([[Objective("uprightPreference",
	torso.rx(t)**2 + torso.rz(t)**2, 500.0)]])

    anim_stepping.add_dimension([[Objective("elbowPreferenceZ",
        joint_elbow_left.get_angle_expr(t)[2]**2 + joint_elbow_right.get_angle_expr(t)[2]**2, 50.0)]])

    #without and with "limp" (right contact has force limit)
    #anim_stepping.add_dimension([[None], ["limping", Constraint("rightLegLimp", c=joint_foot_right.fty(t), ub=char_mortimer.get_mass() * 9.81 * 0.6)]])

    anim_stepping.add_character(char_mortimer)
    anim_stepping.generate()
    
    
    #wait for them all to solve
    #anim_idle.wait_for_results()
    #anim_jump.wait_for_results()
    anim_walk.wait_for_results()
    anim_walklimp.wait_for_results()
    anim_run.wait_for_results()
    anim_walkspin.wait_for_results()
    anim_stepping.wait_for_results()

    animList = anim_walk.AnimationList + anim_walklimp.AnimationList + anim_run.AnimationList + anim_walkspin.AnimationList + anim_stepping.AnimationList;
    solvedAnimList = [anim for anim in animList if anim.Solved]

    #split them up into healthy / limping sets
    healthyList = [anim for anim in solvedAnimList if not anim.has_tag("limping")]
    limpingList = [anim for anim in solvedAnimList if anim.has_tag("limping")]

    #create the "Well-Connected Motion Graph" blends, using weights 0.333 and 0.666
	blendedHealthyList = get_wcmg_blends(healthyList, [0.333, 0.666])
    blendedLimpingList = get_wcmg_blends(limpingList, [0.333, 0.666])

    #export as one large skeleton.xml file
    filename = ".\\output" + "\\" + char_mortimer.Name + '.skeleton.xml'
    LOG.info('Writing %s' % filename)
    xmltree = ogre3d_export_animations(solvedAnimList+blendedHealthyList+blendedLimpingList)
    xmltree.write(filename, None, None)

    #write a string for copy/paste into the AISandbox C++ code
    for anim in healthyList+blendedHealthyList:
        print "\""+anim.Name+"\","
    print "NULL,"
    for anim in limpingList+blendedLimpingList:
        print "\""+anim.Name+"\","

    print("All done!")
    return

def get_wcmg_blends(inputAnims, weightList):
    '''Performs the Well-Connected Motion Graph blending technique (see Zhao and
    Safonova, "Achieving good connectivity in motion graphs"), using the weights in
    weightList, and returns the results. All anims should be for the same character.

    TODO: we should use center of mass for flight phases
    TODO: we should use contact points exactly - not just the body center
    '''

    #here we assume all the anims are for the same character as the first anim
    character = inputAnims[0].Character
    contactJointList = character.get_joints_contact()

    slicedAnimList = [[] for i in range(len(contactJointList)**2)]
    for anim in inputAnims:
        contactFramesSetList = [anim.get_contact_frames(x) for x in contactJointList]

        frameSetList = [set(range(0, anim.get_frame_count()))]
        for in_set in contactFramesSetList:
            out_diff = [out_set - in_set for out_set in frameSetList]
            out_union = [out_set & in_set for out_set in frameSetList]
            frameSetList = out_diff + out_union

        for i, contactType in enumerate(frameSetList):
            frameSetList[i] = sorted(contactType)
            groups = []
            for k, g in itertools.groupby(enumerate(frameSetList[i]), lambda (i,x):i-x):
                groups.append(map(operator.itemgetter(1), g))
            frameSetList[i] = groups

        for i, contactType in enumerate(frameSetList):
            for j, f in enumerate(contactType):
                newanim = anim.get_frame_slice(f[0], f[len(f)-1]+1)
                newanim.Name = str(i) + "_" + newanim.Name
                slicedAnimList[i].append(newanim)

    for i, contactType in enumerate(slicedAnimList):
        LOG.debug("Contact type " + str(i) + " has " + str(len(contactType)) + " clips.")

    #now blend them together, 2 at a time with several weights
    blendedAnimList = []
    for i, anim in enumerate(slicedAnimList):
        #first we have to decide which body will be the root for these blends
        #any body that's in contact will do (we'll take the first one we find)
        binStr = bin(i).lstrip('-0b').rjust(len(contactJointList),'0')
        binStr = binStr[::-1]   #reverse it
        firstContactJointIndex = binStr.find('1') #take the first one
        rootbody = None
        if firstContactJointIndex is -1:
            rootbody = character.DefaultRoot
        else:
            rootbody = contactJointList[firstContactJointIndex].Body

        for x in itertools.combinations(anim, 2):   #2 at a time
            for weight in weightList:
                newanim = x[0].blend(x[1], weight, rootbody)
                newanim.Name = "Blend"+str(weight)+ "_" + x[0].Name + "&" + x[1].Name
                blendedAnimList.append(newanim)

    return blendedAnimList

def test():
    #===========================================================================
    # Simple rigid body test (demonstrates torque-free precession)
    #===========================================================================
    char_test = Character('rigidbody')
    body = RigidBody(0,"Body", 6.41, [0.16, 0.42, 0.16])
    char_test.add_body(body)
    char_test.set_default_root(body)

    #joint_floor = JointContact("floor", body, body.ep_b(), Friction=1.5)
    #char_test.add_joint(joint_floor)

    anim_test = ParameterSpace(Name='torque-free', FPS=25, Length=2.0)

    #anim_test.set_contact_times({
    	#joint_floor:[(0.0, 1.0)]
    #})
    #spin on x axis, 1.0 rad/s
    #anim_test.add_dimension([[ConstraintEq('spinX', (body.rx(1) - body.rx(0)), 5.0*pH, TimeRange='t = 0')]])
    #anim_test.add_dimension([[ConstraintEq('spinY', (body.ry(1) - body.ry(0)), 0*pH, TimeRange='t = 0')]])
    #anim_test.add_dimension([[ConstraintEq('spinZ', (body.rz(1) - body.rz(0)), 0.5*pH, TimeRange='t = 0')]])

    #initial conditions
    #anim_test.add_dimension([[ConstraintEq('TX0', body.tx(0), 0.0)]])
    #anim_test.add_dimension([[ConstraintEq('TX1', body.tx(1), 0.0)]])

    #anim_test.add_dimension([[ConstraintEq('TY0', body.ty(0), 0.0)]])
    #anim_test.add_dimension([[ConstraintEq('TY1', body.ty(1), 0.0)]]) #m/s

    #anim_test.add_dimension([[ConstraintEq('TZ0', body.tz(0), 0.0)]])
    #anim_test.add_dimension([[ConstraintEq('TZ1', body.tz(1), 0.0)]])

    #anim_test.add_dimension([[ConstraintEq('RX0', body.rx(0), 0.1)]])
    #anim_test.add_dimension([[ConstraintEq('RX1', body.rx(1), 0.0*pH)]]) #rad/s

    #anim_test.add_dimension([[ConstraintEq('RY0', body.ry(0), 0.0))
    #anim_test.add_dimension([[Constraint('RY1', lb=5.0*pH, c=body.ry(1))]]) #rad/s

    anim_test.add_dimension([[ConstraintEq('RZ0', body.rz(0), 0.1)]])
    #anim_test.add_dimension([[ConstraintEq('RZ1', body.rz(1), 0.5*pH)]]) #rad/s


    #stay above ground plane
    #anim_test.add_dimension([[SpecifierPluginGroundPlane()]])

    #start within 1 unit of origin
    #anim_test.add_dimension([[Constraint("startNearOrigin", c=body.tx(0)**2 + body.ty(0)**2 + body.tz(0)**2, ub=1.0**2, TimeRange='t = 0')]])

    #anim_test.add_dimension([['sum {t in sTimeSteps} (' + ampl(body.tx(t)**2 + body.ty(t)**2 + body.tz(t)**2) + ')', 1.0)

    anim_test.add_character(char_test)
    anim_test.generate()

    anim_test.wait_for_results()


#===========================================================================
# Application Entry point
#===========================================================================

#this check is necessary for multiprocessing to work
if __name__ == '__main__':

    #setup the logging framework
	import sys
    import logging
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s  %(levelname)s  %(message)s")
    LOG.debug(sys.argv)
    LOG.debug('Running Python '+sys.version)

	#run the main function:
    main()

	#or use this instead to profile the performance of main()
    '''import cProfile, pstats
    cProfile.run('main()', 'main.profile')
    p = pstats.Stats('main.profile')
    p.sort_stats('cumulative').print_stats(30)'''
