from __future__ import division

from optanim.animation import *
from optanim.character import *
from optanim.specifier import *
from optanim.joints import *
from optanim.rigidbody import *

import operator

import logging
LOG = logging.getLogger(__name__)

def main():
    #bodies/joints are defined in character space:
    #right-handed coordinate system
    #x+- is character front, back (ventral, dorsal)
    #y+- is character up, down (cranial, caudal)
    #z+- is character right, left (lateral, lateral)

    #new character, Mortimer the mannequin
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
    joint_hip_left = JointRevolute("L_hip", torso, [0.0, torso.ep_b()[1], -0.1], thigh_left, thigh_left.ep_a(), [[-0.8, 0.4], [-0.5, 0.7], [-1.5, 0.1]], 280)
    char_mortimer.add_joint(joint_hip_left)
    joint_knee_left = JointRevolute("L_knee", thigh_left, thigh_left.ep_b(), calf_left, calf_left.ep_a(), [[0,0], [0,0], [0, 2.8]], 180)
    char_mortimer.add_joint(joint_knee_left)
    joint_hip_right = JointRevolute("R_hip", torso, [0.0, torso.ep_b()[1], 0.1], thigh_right, thigh_right.ep_a(), [[-0.4, 0.8], [-0.7, 0.5], [-1.5, 0.1]], 280)
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
    char_mortimer.add_specifier(SpecifierPluginMinimalJointVelocity(0.5))

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
    # Idle Animations
    #===========================================================================
    anim_idle = AnimationSpec(Name='idle', FPS=10)   #low fps is ok, because not much happens
    anim_idle.set_length(3.0)
    anim_idle.set_contact_times({
    	joint_foot_left:[(0.0, 1.0)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.0, 1.0)]	#contact starting at x, lasting for y
    })

    anim_idle.add_specifier(SpecifierPluginGroundPlane())

    #anim_idle.add_specifier(ConstraintEq("moveALittle", torso.q[4](10), 0.05))

    anim_idle.add_specifier(ConstraintEq("stoppedRX", torso.q[3](0), 0))
    anim_idle.add_specifier(ConstraintEq("stoppedRZ", torso.q[5](0), 0))

    anim_idle.add_specifier(ConstraintEq("stoppedX0", torso.q[0](0), 0))
    anim_idle.add_specifier(ConstraintEq("stoppedZ0", torso.q[2](0), 0))

    anim_idle.add_specifier(ConstraintEq("stoppedCLX", calf_left.q[0](0), 0))
    anim_idle.add_specifier(ConstraintEq("stoppedCRX", calf_right.q[0](0), 0))

    anim_idle.add_specifier(ConstraintEq("stoppedCL_RX", calf_left.q[3](0), 0))
    anim_idle.add_specifier(ConstraintEq("stoppedCR_RX", calf_right.q[3](0), 0))

    anim_idle.add_specifier(ConstraintEq("stoppedCL_RY", calf_left.q[4](0), 0))
    anim_idle.add_specifier(ConstraintEq("stoppedCR_RY", calf_right.q[4](0), 0))

    anim_idle.add_specifier(ConstraintEq("stoppedCL_RZ", calf_left.q[5](0), 0))
    anim_idle.add_specifier(ConstraintEq("stoppedCR_RZ", calf_right.q[5](0), 0))

    anim_idle.add_specifier(SpecifierPluginLoop([0, 0, 0, 0, 0, 0], [0, 0, 0])) #perfect loop

    anim_idle.add_specifier(Objective("elbowPreferenceZ",
        joint_elbow_left.get_angle_expr(t)[2]**2 + joint_elbow_right.get_angle_expr(t)[2]**2, 50.0))

    anim_idle.add_character(char_mortimer)
    anim_idle.generate()

    #===========================================================================
    # Jump Animations
    #===========================================================================
    anim_jump = AnimationSpec(Name='jump', FPS=20)
    anim_jump.set_length(1.8)
    anim_jump.set_contact_times({
    	joint_foot_left:[(0.65, 0.7)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.65, 0.7)]	#contact starting at x, lasting for y
    })

    anim_jump.add_specifier(SpecifierPluginGroundPlane())

    anim_jump.add_specifier(ConstraintEq("stoppedRX", torso.q[3](0), 0))
    anim_jump.add_specifier(ConstraintEq("stoppedRY", torso.q[4](0), 0))
    #anim_jump.add_specifier(ConstraintEq("stoppedRZ", torso.q[5](0), 0))

    anim_jump.add_specifier(ConstraintEq("stoppedX0", torso.q[0](0), 0))
    anim_jump.add_specifier(ConstraintEq("stoppedZ0", torso.q[2](0), 0))

    '''anim_jump.add_specifier(ConstraintEq("stoppedCLX", calf_left.q[0](0), 0))
    anim_jump.add_specifier(ConstraintEq("stoppedCRX", calf_right.q[0](0), 0))

    anim_jump.add_specifier(ConstraintEq("stoppedCL_RX", calf_left.q[3](0), 0))
    anim_jump.add_specifier(ConstraintEq("stoppedCR_RX", calf_right.q[3](0), 0))

    anim_jump.add_specifier(ConstraintEq("stoppedCL_RY", calf_left.q[4](0), 0))
    anim_jump.add_specifier(ConstraintEq("stoppedCR_RY", calf_right.q[4](0), 0))

    anim_jump.add_specifier(ConstraintEq("stoppedCL_RZ", calf_left.q[5](0), 0))
    anim_jump.add_specifier(ConstraintEq("stoppedCR_RZ", calf_right.q[5](0), 0))'''

    anim_jump.add_param_specifier([[SpecifierPluginLoop([x, 0, 0, 0, 0, 0], [0, 0, 0])] for x in [0.0, 0.5]])

    anim_jump.add_specifier(Objective("elbowPreferenceZ",
        joint_elbow_left.get_angle_expr(t)[2]**2 + joint_elbow_right.get_angle_expr(t)[2]**2, 50.0))

    anim_jump.add_specifier(Objective("minimalGroundForce",
        joint_foot_left.f[1](t)**2 + joint_foot_right.f[1](t)**2, 0.0001))

    anim_jump.add_character(char_mortimer)
    anim_jump.generate()

    #===========================================================================
    # Start / Stop Locomotion
    #===========================================================================
    anim_startstop = AnimationSpec(Name='startstop', FPS=20)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    anim_startstop.set_length(1.8)
    anim_startstop.set_contact_times({
        joint_foot_left:[(0.0, 0.2018), (0.3893, 0.225), (0.8018, 0.1982)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.0, 0.3893), (0.6143, 0.3857)],	#contact starting at x, lasting for y
    })

    speed = 0.6 #m/s
    coords = [[math.cos(fracCircle*2*math.pi)*speed, math.sin(fracCircle*2*math.pi)*speed] for fracCircle in numpy.arange(0, 1/2+1/8, 1/8)]

    c_walk = [[SpecifierPluginLoop([x, 0, z, 0, 0, 0], [0, 0, 0]),
	#Constraint("faceforwards", lb=-0.15, c=torso.q[4](t), ub=0.15),
        ConstraintEq("faceforwards", torso.q[4](t), 0),
        ConstraintEq("stoppedTorso_RX", torso.q[3](0), 0),
        ConstraintEq("stoppedTorso_RZ", torso.q[5](0), 0),

        ConstraintEq("stoppedTorso_X0", torso.q[0](0), 0),
        ConstraintEq("stoppedTorso_Z0", torso.q[2](0), 0),
        ConstraintEq("stoppedTorso_X1", torso.q[0](1), 0),
        ConstraintEq("stoppedTorso_Z1", torso.q[2](1), 0),
        #ConstraintEq("stoppedTorso_X2", torso.q[0](2), 0),
        #ConstraintEq("stoppedTorso_Z2", torso.q[2](2), 0),

        ConstraintEq("stoppedCL_RY", thigh_left.q[4](0), 0),
        ConstraintEq("stoppedCR_RY", thigh_right.q[4](0), 0),
        ConstraintEq("stoppedCL_RX", thigh_left.q[3](0), 0),
        ConstraintEq("stoppedCR_RX", thigh_right.q[3](0), 0),

        ConstraintEq("stoppedR_arm_upper_RY", R_arm_upper.q[4](0), 0),
        ConstraintEq("stoppedL_arm_upper_RY", L_arm_upper.q[4](0), 0),

        ConstraintEq("stoppedR_arm_upper_RZ", R_arm_upper.q[5](0), 0),
        ConstraintEq("stoppedL_arm_upper_RZ", L_arm_upper.q[5](0), 0),

        #ConstraintEq("stoppedCL_RZ", calf_left.q[5](0), 0),
        #ConstraintEq("stoppedCR_RZ", calf_right.q[5](0), 0),

        ConstraintEq("stoppedCL_X0", calf_left.q[0](0), 0),
        ConstraintEq("stoppedCR_X0", calf_right.q[0](0), 0)] for x,z in coords]
        #ConstraintEq("stoppedCL_X1", calf_left.q[0](1), 0),
        #ConstraintEq("stoppedCR_X1", calf_right.q[0](1), 0)]

    anim_startstop.add_param_specifier(c_walk)

    #stay above ground plane
    anim_startstop.add_specifier(SpecifierPluginGroundPlane())

    #some "preferences":
    #minimize rotation of torso on x and z axes (people tend to walk upright)
    anim_startstop.add_specifier(Objective("uprightPreference",
	torso.q[3](t)**2 + torso.q[5](t)**2, 500.0))

    anim_startstop.add_specifier(Objective("elbowPreferenceZ",
        joint_elbow_left.get_angle_expr(t)[2]**2 + joint_elbow_right.get_angle_expr(t)[2]**2, 50.0))

    anim_startstop.add_character(char_mortimer)
    anim_startstop.generate()


    #===========================================================================
    # Walk Animations
    #===========================================================================
    anim_walk = AnimationSpec(Name='walk', FPS=20)

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

    #straight
    c_straight = [
        #[SpecifierPluginLoop([speed, 0, 0, 0, 0, 0], [0, 0, 0]),
	#Constraint("startTorsoNearOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')],

        [SpecifierPluginLoop([speed, 0, 0, 0, 0, 0], [0, 0, 0]),
	#Constraint("torso_ry", lb=-0.1, c=torso.q[4](t), ub=0.1),
        ConstraintEq("torso_ry", torso.q[4](t), 0),
	Constraint("startTorsoNearOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')]
        ]

    #turning
    turnRadii = [x*x*x for x in numpy.arange(0.3, 2.1, 0.2)]
    #print zip(turnRadii,[min(speed/r, 2.0) for r in turnRadii])
    c_turn = [[
	SpecifierPluginLoop([0, 0, 0, 0, 0, 0], [0, min(speed/r, 2.0), 0]),
	ConstraintEq("startTorso_tx", torso.q[0](t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tz", torso.q[2](t), r, TimeRange='t = 0'),
        ConstraintEq("torso_ry", torso.q[4](t), min(speed/r, 2.0)*(t/anim_walk.FPS))] for r in turnRadii]

    anim_walk.add_param_specifier(c_straight + c_turn)

    #stay above ground plane
    anim_walk.add_specifier(SpecifierPluginGroundPlane())

    #some "preferences":
    #minimize rotation of torso on x and z axes (people tend to walk upright)
    anim_walk.add_specifier(Objective("uprightPreference",
	torso.q[3](t)**2 + torso.q[5](t)**2, 500.0))

    anim_walk.add_specifier(Objective("elbowPreferenceZ",
        joint_elbow_left.get_angle_expr(t)[2]**2 + joint_elbow_right.get_angle_expr(t)[2]**2, 50.0))

    anim_walk.add_character(char_mortimer)
    anim_walk.generate()


    #===========================================================================
    # Run Animations
    #===========================================================================
    anim_run = AnimationSpec(Name='run', FPS=37)

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
        #Constraint("torso_ry", lb=-0.1, c=torso.q[4](t), ub=0.1),
        #ConstraintEq("torso_rx", torso.q[3](t), 0),
        ConstraintEq("torso_ry", torso.q[4](t), 0),
	Constraint("startTorsoNearOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')]]

    #turning
    turnRadii = [x*x for x in numpy.arange(2.0, 3.0, 0.2)]
    #print zip(turnRadii,[min(speed/r, 2.25) for r in turnRadii])
    c_turn = [[
	SpecifierPluginLoop([0, 0, 0, 0, 0, 0], [0, min(speed/r, 2.25), 0]),
	ConstraintEq("startTorso_tx", torso.q[0](t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tz", torso.q[2](t), r, TimeRange='t = 0'),
        ConstraintEq("torso_ry", torso.q[4](t), min(speed/r, 2.25)*(t/anim_run.FPS))] for r in turnRadii]

    anim_run.add_param_specifier(c_straight + c_turn)

    #stay above ground plane
    anim_run.add_specifier(SpecifierPluginGroundPlane())

    #some "preferences":
    #minimize rotation of torso on z axis (people tend to walk upright)
    anim_run.add_specifier(Objective("uprightPreferenceZ",
	torso.q[5](t)**2, 500.0))

    #and because the arms aren't very constrained, we'll "lead" the solution a bit
    '''anim_run.add_specifier(ConstraintEq("shoulderSwingLeft0", joint_shoulder_left.get_angle_expr(t)[2], -0.15, TimeRange='t = 19'))
    anim_run.add_specifier(ConstraintEq("shoulderSwingLeft1", joint_shoulder_left.get_angle_expr(t)[2], 0.78, TimeRange='t = 7'))
    anim_run.add_specifier(ConstraintEq("shoulderSwingRight0", joint_shoulder_right.get_angle_expr(t)[2], -0.15, TimeRange='t = 7'))
    anim_run.add_specifier(ConstraintEq("shoulderSwingRight1", joint_shoulder_right.get_angle_expr(t)[2], 0.78, TimeRange='t = 19'))'''

    #keep elbow at -1.9 radians (people run with their elbows bent like this)
    anim_run.add_specifier(Objective("elbowPreferenceZ",
        (joint_elbow_left.get_angle_expr(t)[2]+1.9)**2 + (joint_elbow_right.get_angle_expr(t)[2]+1.9)**2, 400.0))

    anim_run.add_character(char_mortimer)
    #anim_run.generate()
    
    #wait for them all to solve
    anim_idle.wait_for_results()
    anim_jump.wait_for_results()
    anim_startstop.wait_for_results()
    anim_walk.wait_for_results()
    anim_run.wait_for_results()

    animList = anim_idle.AnimationList + anim_jump.AnimationList + anim_startstop.AnimationList + anim_walk.AnimationList + anim_run.AnimationList
    finalAnimList = []
    for anim in animList:
        #discard the poor ones
        if anim.Solved and anim.ObjectiveValue < 240:
            finalAnimList.append(anim)

    #blendedAnim = finalAnimList[1].blend(finalAnimList[2], 0.5, fps=20)
    #blendedAnim.export('.\\blended')

    #for anim in finalAnimList:
        #anim = anim.animdata_resample(60) #upsample to 60 fps
        #anim.export('.\\anims')

    #for anim in finalAnimList:
            #print "<animationlink skeletonName=\""+anim.Name+".skeleton\" />"

    #for anim in finalAnimList:
            #print "\""+anim.Name+"\","


    #experiment to see if we can blend them:
    #=================================================   
    slicedAnimList = [[] for i in range(2**2)]  #TODO: fix this for other characters

    for anim in finalAnimList:

        #discard the poor ones
        if anim.ObjectiveValue > 240:
            continue

        jointsContact = anim.Character.get_joints_contact()
        contactFramesSetList = [anim.get_contact_frames(x) for x in jointsContact]

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

    #flatSlicedAnimList = [item for sublist in slicedAnimList for item in sublist]

    #now blend them together, 2 at a time with several weights
    weightList = [0.25, 0.5, 0.75]
    blendedAnimList = []
    for i, anim in enumerate(slicedAnimList):
        #try it without double support blending
        if i is 3:
            continue
            
        rootbody = None
        #choose the root body based on which foot is in contact
        if i is 0:
            rootbody = torso        #flight
        elif i is 1:
            rootbody = calf_left    #right foot contact
        elif i is 2:
            rootbody = calf_right   #left foot contact
        elif i is 3:
            rootbody = calf_left    #double support (just pick one arbitrarily)

        for x in itertools.combinations(anim, 2):   #2 at a time
            for weight in weightList:
                newanim = x[0].blend(x[1], weight, rootbody)
                newanim.Name = "Blend"+str(weight)+ "_" + x[0].Name + "&" + x[1].Name
                blendedAnimList.append(newanim)


    #export them
    #for x in finalAnimList+blendedAnimList:
        #x.export('.\\output\\')

    #also export as one large skeleton.xml file
    filename = ".\\output" + "\\" + char_mortimer.Name + '.skeleton.xml'
    LOG.info('Writing %s' % filename)
    file = openfile(filename, 'w')
    file.write(ogre3d_export_animations(finalAnimList+blendedAnimList))
    file.close()
    
    for anim in finalAnimList+blendedAnimList:
        print "\""+anim.Name+"\","
    #=================================================

    print("All done!")
    return

def test():
    #===========================================================================
    # Simple rigid body test
    #===========================================================================
    char_test = Character('rigidbody')
    body = RigidBody("Body", 6.41, [0.16, 0.42, 0.16])
    char_test.add_body(body)

    #joint_floor = JointContact("floor", body, body.ep_b(), Friction=1.5)
    #char_test.add_joint(joint_floor)

    anim_test = AnimationSpec(Name='torque-free', FPS=25, Length=2.0)

    #anim_test.set_contact_times({
    	#joint_floor:[(0.0, 1.0)]
    #})
    #spin on x axis, 1.0 rad/s
    #anim_test.add_specifier(ConstraintEq('spinX', (body.q[3](1) - body.q[3](0)), 5.0*pH, TimeRange='t = 0'))
    #anim_test.add_specifier(ConstraintEq('spinY', (body.q[4](1) - body.q[4](0)), 0*pH, TimeRange='t = 0'))
    #anim_test.add_specifier(ConstraintEq('spinZ', (body.q[5](1) - body.q[5](0)), 0.5*pH, TimeRange='t = 0'))

    #initial conditions
    #anim_test.add_specifier(ConstraintEq('TX0', body.q[0](0), 0.0))
    #anim_test.add_specifier(ConstraintEq('TX1', body.q[0](1), 0.0))

    #anim_test.add_specifier(ConstraintEq('TY0', body.q[1](0), 0.0))
    #anim_test.add_specifier(ConstraintEq('TY1', body.q[1](1), 0.0)) #m/s

    #anim_test.add_specifier(ConstraintEq('TZ0', body.q[2](0), 0.0))
    #anim_test.add_specifier(ConstraintEq('TZ1', body.q[2](1), 0.0))

    #anim_test.add_specifier(ConstraintEq('RX0', body.q[3](0), 0.1))
    #anim_test.add_specifier(ConstraintEq('RX1', body.q[3](1), 0.0*pH)) #rad/s

    #anim_test.add_specifier(ConstraintEq('RY0', body.q[4](0), 0.0))
    #anim_test.add_specifier(Constraint('RY1', lb=5.0*pH, c=body.q[4](1))) #rad/s

    anim_test.add_specifier(ConstraintEq('RZ0', body.q[5](0), 0.1))
    #anim_test.add_specifier(ConstraintEq('RZ1', body.q[5](1), 0.5*pH)) #rad/s


    #stay above ground plane
    #anim_test.add_specifier(SpecifierPluginGroundPlane())

    #start within 1 unit of origin
    #anim_test.add_specifier(Constraint("startNearOrigin", c=body.q[0](0)**2 + body.q[1](0)**2 + body.q[2](0)**2, ub=1.0**2, TimeRange='t = 0'))

    #anim_test.add_specifier('sum {t in sTimeSteps} (' + ampl(body.q[0](t)**2 + body.q[1](t)**2 + body.q[2](t)**2) + ')', 1.0)

    anim_test.add_character(char_test)
    anim_test.generate('.', 'ipopt')


def backup():
    #bodies/joints are defined in character space:
    #x+- is character front, back (ventral, dorsal)
    #y+- is character up, down (cranial, caudal)
    #z+- is character right, left (lateral, lateral)

    #new character, Doug the dog
    char_doug = Character('Doug')
    torso = RigidBody(0, "C_torso", 26.05, [0.21, 0.52, 0.27])
    char_doug.add_body(torso)
    leg_B_L_U = RigidBody(1, "L_leg_upper_back", 6.41, [0.16, 0.42, 0.16])
    char_doug.add_body(leg_B_L_U)
    leg_B_R_U = RigidBody(2, "R_leg_upper_back", 6.41, [0.16, 0.42, 0.16])
    char_doug.add_body(leg_B_R_U)
    leg_B_L_L = RigidBody(3, "L_leg_lower_back", 3.13, [0.11, 0.43, 0.11])
    char_doug.add_body(leg_B_L_L)
    leg_B_R_L = RigidBody(4, "R_leg_lower_back", 3.13, [0.11, 0.43, 0.11])
    char_doug.add_body(leg_B_R_L)
    leg_F_L_U = RigidBody(5, "L_leg_upper_front", 6.41, [0.16, 0.42, 0.16])
    char_doug.add_body(leg_F_L_U)
    leg_F_R_U = RigidBody(6, "R_leg_upper_front", 6.41, [0.16, 0.42, 0.16])
    char_doug.add_body(leg_F_R_U)
    leg_F_L_L = RigidBody(7, "L_leg_lower_front", 3.13, [0.11, 0.43, 0.11])
    char_doug.add_body(leg_F_L_L)
    leg_F_R_L = RigidBody(8, "R_leg_lower_front", 3.13, [0.11, 0.43, 0.11])
    char_doug.add_body(leg_F_R_L)

    #define some joints to constrain the bodies together
    '''rotoffset = -math.pi/2
    joint_B_L_U = JointRevolute("L_back_upper", torso, [0.0, torso.ep_b()[1], -0.1], leg_B_L_U, leg_B_L_U.ep_a(), [[-0.8, 0.4], [-0.5, 0.7], [-1.5+rotoffset, 0.1+rotoffset]], 300)
    char_doug.add_joint(joint_B_L_U)
    joint_B_R_U = JointRevolute("R_back_upper", torso, [0.0, torso.ep_b()[1], 0.1], leg_B_R_U, leg_B_R_U.ep_a(), [[-0.4, 0.8], [-0.7, 0.5], [-1.5+rotoffset, 0.1+rotoffset]], 300)
    char_doug.add_joint(joint_B_R_U)
    joint_B_L_L = JointRevolute("L_back_lower", leg_B_L_U, leg_B_L_U.ep_b(), leg_B_L_L, leg_B_L_L.ep_a(), [[0,0], [0,0], [0, 2.8]], 200)
    char_doug.add_joint(joint_B_L_L)
    joint_B_R_L = JointRevolute("R_back_lower", leg_B_R_U, leg_B_R_U.ep_b(), leg_B_R_L, leg_B_R_L.ep_a(), [[0,0], [0,0], [0, 2.8]], 200)
    char_doug.add_joint(joint_B_R_L)
    joint_F_L_U = JointRevolute("L_front_upper", torso, [0.0, torso.ep_a()[1]-0.1, -0.1], leg_F_L_U, leg_F_L_U.ep_a(), [[-0.8, 0.4], [-0.5, 0.7], [-0.1+rotoffset, 1.5+rotoffset]], 300)
    char_doug.add_joint(joint_F_L_U)
    joint_F_R_U = JointRevolute("R_front_upper", torso, [0.0, torso.ep_a()[1]-0.1, 0.1], leg_F_R_U, leg_F_R_U.ep_a(), [[-0.8, 0.4], [-0.5, 0.7], [-0.1+rotoffset, 1.5+rotoffset]], 300)
    char_doug.add_joint(joint_F_R_U)
    joint_F_L_L = JointRevolute("L_front_lower", leg_F_L_U, leg_F_L_U.ep_b(), leg_F_L_L, leg_F_L_L.ep_a(), [[0,0], [0,0], [-2.8, 0.0]], 200)
    char_doug.add_joint(joint_F_L_L)
    joint_F_R_L = JointRevolute("R_front_lower", leg_F_R_U, leg_F_R_U.ep_b(), leg_F_R_L, leg_F_R_L.ep_a(), [[0,0], [0,0], [-2.8, 0.0]], 200)
    char_doug.add_joint(joint_F_R_L)'''
    rotoffset = -math.pi/2
    joint_B_L_U = JointRevolute("L_back_upper", torso, [0.0, torso.ep_b()[1], -0.1], leg_B_L_U, leg_B_L_U.ep_a(), [[-4*math.pi,4*math.pi], [-4*math.pi,4*math.pi], [-4*math.pi, 4*math.pi]], 300)
    char_doug.add_joint(joint_B_L_U)
    joint_B_R_U = JointRevolute("R_back_upper", torso, [0.0, torso.ep_b()[1], 0.1], leg_B_R_U, leg_B_R_U.ep_a(), [[-4*math.pi,4*math.pi], [-4*math.pi,4*math.pi], [-4*math.pi, 4*math.pi]], 300)
    char_doug.add_joint(joint_B_R_U)
    joint_B_L_L = JointRevolute("L_back_lower", leg_B_L_U, leg_B_L_U.ep_b(), leg_B_L_L, leg_B_L_L.ep_a(), [[-4*math.pi,4*math.pi], [-4*math.pi,4*math.pi], [-4*math.pi, 4*math.pi]], 200)
    char_doug.add_joint(joint_B_L_L)
    joint_B_R_L = JointRevolute("R_back_lower", leg_B_R_U, leg_B_R_U.ep_b(), leg_B_R_L, leg_B_R_L.ep_a(), [[-4*math.pi,4*math.pi], [-4*math.pi,4*math.pi], [-4*math.pi, 4*math.pi]], 200)
    char_doug.add_joint(joint_B_R_L)
    joint_F_L_U = JointRevolute("L_front_upper", torso, [0.0, torso.ep_a()[1]-0.1, -0.1], leg_F_L_U, leg_F_L_U.ep_a(), [[-4*math.pi,4*math.pi], [-4*math.pi,4*math.pi], [-4*math.pi, 4*math.pi]], 300)
    char_doug.add_joint(joint_F_L_U)
    joint_F_R_U = JointRevolute("R_front_upper", torso, [0.0, torso.ep_a()[1]-0.1, 0.1], leg_F_R_U, leg_F_R_U.ep_a(), [[-4*math.pi,4*math.pi], [-4*math.pi,4*math.pi], [-4*math.pi, 4*math.pi]], 300)
    char_doug.add_joint(joint_F_R_U)
    joint_F_L_L = JointRevolute("L_front_lower", leg_F_L_U, leg_F_L_U.ep_b(), leg_F_L_L, leg_F_L_L.ep_a(), [[-4*math.pi,4*math.pi], [-4*math.pi,4*math.pi], [-4*math.pi, 4*math.pi]], 200)
    char_doug.add_joint(joint_F_L_L)
    joint_F_R_L = JointRevolute("R_front_lower", leg_F_R_U, leg_F_R_U.ep_b(), leg_F_R_L, leg_F_R_L.ep_a(), [[-4*math.pi,4*math.pi], [-4*math.pi,4*math.pi], [-4*math.pi, 4*math.pi]], 200)
    char_doug.add_joint(joint_F_R_L)

    #define an (unpowered) contact joint for each foot
    #(character may push against ground plane with these points)
    joint_foot_B_L = JointContact("L_foot_back", leg_B_L_L, leg_B_L_L.ep_b(), Friction=0.5)
    char_doug.add_joint(joint_foot_B_L)
    joint_foot_B_R = JointContact("R_foot_back", leg_B_R_L, leg_B_R_L.ep_b(), Friction=0.5)
    char_doug.add_joint(joint_foot_B_R)
    joint_foot_F_L = JointContact("L_foot_front", leg_F_L_L, leg_F_L_L.ep_b(), Friction=0.5)
    char_doug.add_joint(joint_foot_F_L)
    joint_foot_F_R = JointContact("R_foot_front", leg_F_R_L, leg_F_R_L.ep_b(), Friction=0.5)
    char_doug.add_joint(joint_foot_F_R)

    anim_dougtest = AnimationSpec(Name='dougtest', FPS=20)
    anim_dougtest.add_character(char_doug)
    anim_dougtest.set_length(1.19)
    anim_dougtest.set_contact_times({
    	joint_foot_B_L:[(0.0, 0.65)],	#contact starting at x%, lasting for y%
        joint_foot_F_L:[(0.25, 0.65)],
        joint_foot_B_R:[(0.5, 0.65)],
    	joint_foot_F_R:[(0.75, 0.65)]
    })
    anim_dougtest.add_specifier(SpecifierPluginGroundPlane())
    anim_dougtest.add_specifier(SpecifierPluginLoop([1.4, 0, 0, 0, 0, 0], [0, 0, 0]))
    anim_dougtest.add_specifier(Constraint("startAtOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0'))
    anim_dougtest.add_specifier(Objective("energy",
        joint_B_L_U.get_torquesquared_expr() +
        joint_B_R_U.get_torquesquared_expr() +
        joint_B_L_L.get_torquesquared_expr() +
        joint_B_R_L.get_torquesquared_expr() +
        joint_F_L_U.get_torquesquared_expr() +
        joint_F_R_U.get_torquesquared_expr() +
        joint_F_L_L.get_torquesquared_expr() +
        joint_F_R_L.get_torquesquared_expr()
        , 0.01))
    anim_dougtest.generate('.', 'ipopt')

    LOG.info("Exit")
    return

#this check is necessary for multiprocessing to work
if __name__ == '__main__':
    import sys
    import logging
    logging.basicConfig(level=logging.DEBUG, format="%(asctime)s  %(levelname)s  %(message)s")

    LOG.debug(sys.argv)
    LOG.debug('Running Python '+sys.version)

    main()

    '''import cProfile, pstats
    cProfile.run('main()', 'main.profile')
    p = pstats.Stats('main.profile')
    p.sort_stats('cumulative').print_stats(30)'''
