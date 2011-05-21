from __future__ import division

from optanim.animation import *
from optanim.character import *
from optanim.constraint import *
from optanim.objective import *
from optanim.constraintplugins import *
from optanim.joints import *
from optanim.rigidbody import *

import operator

def main():

    #new character, Mortimer the mannequin
    char_mortimer = Character('Mortimer')
    torso = RigidBody(0, "C_torso", 26.05, [0.21, 0.52, 0.27])
    char_mortimer.add_body(torso)
    thigh_left = RigidBody(1, "L_leg_upper", 6.41, [0.16, 0.42, 0.16])
    char_mortimer.add_body(thigh_left)
    calf_left = RigidBody(3, "L_leg_lower", 3.13, [0.11, 0.43, 0.11])
    char_mortimer.add_body(calf_left)
    thigh_right = RigidBody(2, "R_leg_upper", 6.41, [0.16, 0.42, 0.16])
    char_mortimer.add_body(thigh_right)
    calf_right = RigidBody(4, "R_leg_lower", 3.13, [0.11, 0.43, 0.11])
    char_mortimer.add_body(calf_right)

    #define some joints to constrain the bodies together
    joint_hip_left = JointRevolute("L_hip", torso, [0.0, torso.ep_b()[1], -0.1], thigh_left, thigh_left.ep_a(), [[-0.8, 0.4], [-0.5, 0.7], [-1.5, 0.1]], 300)
    char_mortimer.add_joint(joint_hip_left)
    joint_knee_left = JointRevolute("L_knee", thigh_left, thigh_left.ep_b(), calf_left, calf_left.ep_a(), [[0,0], [0,0], [0, 2.8]], 200)
    char_mortimer.add_joint(joint_knee_left)
    joint_hip_right = JointRevolute("R_hip", torso, [0.0, torso.ep_b()[1], 0.1], thigh_right, thigh_right.ep_a(), [[-0.4, 0.8], [-0.7, 0.5], [-1.5, 0.1]], 300)
    char_mortimer.add_joint(joint_hip_right)
    joint_knee_right = JointRevolute("R_knee", thigh_right, thigh_right.ep_b(), calf_right, calf_right.ep_a(), [[0,0], [0,0], [0, 2.8]], 200)
    char_mortimer.add_joint(joint_knee_right)

    #define an (unpowered) contact joint for each foot
    #(character may push against ground plane with these points)
    joint_foot_left = JointContact("L_foot", calf_left, calf_left.ep_b(), Friction=0.5)
    char_mortimer.add_joint(joint_foot_left)
    joint_foot_right = JointContact("R_foot", calf_right, calf_right.ep_b(), Friction=0.5)
    char_mortimer.add_joint(joint_foot_right)

    #===========================================================================
    # Idle Animations
    #===========================================================================
    anim_idle = AnimationSpec(Name='idle', FPS=5)   #low fps is ok, because not much happens
    anim_idle.set_length(3.0)
    anim_idle.set_contact_times({
    	joint_foot_left:[(0.0, 1.0)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.0, 1.0)]	#contact starting at x, lasting for y
    })
    anim_idle.add_constraint(ConstraintPluginGroundPlane())
    #anim_idle.add_constraint(Constraint("startAtOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0'))

    anim_idle.add_constraint(ConstraintEq("moveALittle", torso.q[4](10), 0.05))

    anim_idle.add_constraint(ConstraintEq("stoppedRX", torso.q[3](0), 0))
    anim_idle.add_constraint(ConstraintEq("stoppedRZ", torso.q[5](0), 0))

    anim_idle.add_constraint(ConstraintEq("stoppedX0", torso.q[0](0), 0))
    anim_idle.add_constraint(ConstraintEq("stoppedZ0", torso.q[2](0), 0))

    anim_idle.add_constraint(ConstraintEq("stoppedCLX", calf_left.q[0](0), 0))
    anim_idle.add_constraint(ConstraintEq("stoppedCRX", calf_right.q[0](0), 0))

    anim_idle.add_constraint(ConstraintEq("stoppedCL_RX", calf_left.q[3](0), 0))
    anim_idle.add_constraint(ConstraintEq("stoppedCR_RX", calf_right.q[3](0), 0))

    anim_idle.add_constraint(ConstraintEq("stoppedCL_RY", calf_left.q[4](0), 0))
    anim_idle.add_constraint(ConstraintEq("stoppedCR_RY", calf_right.q[4](0), 0))

    anim_idle.add_constraint(ConstraintEq("stoppedCL_RZ", calf_left.q[5](0), 0))
    anim_idle.add_constraint(ConstraintEq("stoppedCR_RZ", calf_right.q[5](0), 0))

    anim_idle.add_constraint(ConstraintPluginLoop([0, 0, 0, 0, 0, 0], [0, 0, 0])) #perfect loop

    '''#crouched and upright
    pelvisPoint = world_xf(torso.ep_b(), [bq(t) for bq in torso.q])
    c_crouched = [Constraint("crouched", c=pelvisPoint[1], ub=0.5)]
    anim_idle.add_param_constraint([[None], c_crouched])'''

    anim_idle.add_objective(Objective("energy",
        joint_hip_left.get_torquesquared_expr() +
        joint_knee_left.get_torquesquared_expr() +
        joint_hip_right.get_torquesquared_expr() +
        joint_knee_right.get_torquesquared_expr(),
        0.01))

    anim_idle.add_character(char_mortimer)
    anim_idle.generate()

    #===========================================================================
    # Start / Stop Locomotion
    #===========================================================================
    anim_startstop = AnimationSpec(Name='startstop', FPS=25)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    anim_startstop.set_length(1.8)
    anim_startstop.set_contact_times({
        joint_foot_left:[(0.0, 0.1), (0.35, 0.3), (0.9, 0.1)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.0, 0.35), (0.65, 0.4)],	#contact starting at x, lasting for y
    })

    c_walk = [ConstraintPluginLoop([0.5, 0, 0, 0, 0, 0], [0, 0, 0]),
	Constraint("faceforwards", lb=-0.15, c=torso.q[4](t), ub=0.15),
        ConstraintEq("stoppedTorso_RX", torso.q[3](0), 0),
        ConstraintEq("stoppedTorso_RZ", torso.q[5](0), 0),

        ConstraintEq("stoppedTorso_X0", torso.q[0](0), 0),
        ConstraintEq("stoppedTorso_Z0", torso.q[2](0), 0),
        ConstraintEq("stoppedTorso_X1", torso.q[0](1), 0),
        ConstraintEq("stoppedTorso_Z1", torso.q[2](1), 0),
        ConstraintEq("stoppedTorso_X2", torso.q[0](2), 0),
        ConstraintEq("stoppedTorso_Z2", torso.q[2](2), 0),

        ConstraintEq("stoppedCL_RY", thigh_left.q[4](0), 0),
        ConstraintEq("stoppedCR_RY", thigh_right.q[4](0), 0),
        ConstraintEq("stoppedCL_RX", thigh_left.q[3](0), 0),
        ConstraintEq("stoppedCR_RX", thigh_right.q[3](0), 0),

        #ConstraintEq("stoppedCL_RY", calf_left.q[4](0), 0),
        #ConstraintEq("stoppedCR_RY", calf_right.q[4](0), 0),

        #ConstraintEq("stoppedCL_RZ", calf_left.q[5](0), 0),
        #ConstraintEq("stoppedCR_RZ", calf_right.q[5](0), 0),

        ConstraintEq("stoppedCL_X0", calf_left.q[0](0), 0),
        ConstraintEq("stoppedCR_X0", calf_right.q[0](0), 0)]
        #ConstraintEq("stoppedCL_X1", calf_left.q[0](1), 0),
        #ConstraintEq("stoppedCR_X1", calf_right.q[0](1), 0)]

    anim_startstop.add_param_constraint([c_walk])

     #stay above ground plane
    anim_startstop.add_constraint(ConstraintPluginGroundPlane())

    #minimize torques
    #we divide by time so animations of different lengths can be compared fairly
    anim_startstop.add_objective(Objective("energy",
        joint_hip_left.get_torquesquared_expr() +
        joint_knee_left.get_torquesquared_expr() +
        joint_hip_right.get_torquesquared_expr() +
        joint_knee_right.get_torquesquared_expr(),
        0.01))

    #minimize rotation of torso on x and z axes (people tend to walk upright)
    anim_startstop.add_objective(Objective("upright",
	torso.q[3](t)**2 + torso.q[5](t)**2, 100.0))

    #minimize acceleration of the torso rotation (so brain is not jostled, and eyes can see etc.)
    torsoPrev = [bq(t-1) for bq in torso.q[3:]]
    torsoCurr = [bq(t) for bq in torso.q[3:]]
    torsoNext = [bq(t+1) for bq in torso.q[3:]]
    for i in range(3):
	vPrev = torsoCurr[i] - torsoPrev[i]
	vNext = torsoNext[i] - torsoCurr[i]
	accel = vNext - vPrev
	#anim_locomote.add_objective('sum {t in sTimeSteps: t>pTimeBegin && t<pTimeEnd} ('+ampl((accel)**2)+')', 1000.0)
        anim_startstop.add_objective(Objective("smooth",accel**2, 1000.0, 't>pTimeBegin && t<pTimeEnd'))

    anim_startstop.add_character(char_mortimer)
    anim_startstop.generate()


    #===========================================================================
    # Locomotion Animations
    #===========================================================================
    anim_locomote = AnimationSpec(Name='locomote', FPS=25)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    anim_locomote.set_length(1.0)
    anim_locomote.set_contact_times({
    	joint_foot_left:[(0.37056432, 0.46671992)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.83310183, 0.52637255)]	#contact starting at x, lasting for y
    })

    walkSpeed = 1.25

    #straight
    c_straight = [
        [ConstraintPluginLoop([walkSpeed, 0, 0, 0, 0, 0], [0, 0, 0]),
	Constraint("startTorsoNearOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')],

        [ConstraintPluginLoop([walkSpeed, 0, 0, 0, 0, 0], [0, 0, 0]),
	Constraint("faceforwards", lb=-0.35, c=torso.q[4](t), ub=0.35),
	Constraint("startTorsoNearOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')]
        ]

    #turning
    turnRadii = [x*x for x in numpy.arange(0.2, 2.2, 0.2)]
    c_turn = [[
	ConstraintPluginLoop([0, 0, 0, 0, 0, 0], [0, (walkSpeed+0.6)/(r*r + 1.0) + 0.3, 0]),
	ConstraintEq("startTorso_tx", torso.q[0](t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tz", torso.q[2](t), r, TimeRange='t = 0')] for r in turnRadii]

    anim_locomote.add_param_constraint(c_straight + c_turn)

    #facing direction:
    anim_locomote.add_constraint(ConstraintEq("startTorso_ry", torso.q[4](t), 0, TimeRange='t = 0'))
    '''#facing direction: forwards, inwards (left), backwards, outwards (right)
    faceDir = [
	[ConstraintEq("startTorso_ry", torso.q[4](t), (math.pi/2)*0, TimeRange='t = 0')],
        [ConstraintEq("startTorso_ry", torso.q[4](t), (math.pi/2)*1, TimeRange='t = 0')],
        [ConstraintEq("startTorso_ry", torso.q[4](t), (math.pi/2)*2, TimeRange='t = 0')],
	[ConstraintEq("startTorso_ry", torso.q[4](t), (math.pi/2)*3, TimeRange='t = 0')]]
    anim_locomote.add_param_constraint(faceDir)'''

    #stay above ground plane
    anim_locomote.add_constraint(ConstraintPluginGroundPlane())

    #minimize torques
    anim_locomote.add_objective(Objective("energy",
        joint_hip_left.get_torquesquared_expr() +
        joint_knee_left.get_torquesquared_expr() +
        joint_hip_right.get_torquesquared_expr() +
        joint_knee_right.get_torquesquared_expr(),
        0.01))

    #minimize rotation of torso on x and z axes (people tend to walk upright)
    anim_locomote.add_objective(Objective("upright",
	torso.q[3](t)**2 + torso.q[5](t)**2, 100.0))

    #faceObjs = [('sum {t in sTimeSteps} ('+ampl((torso.q[4](t) - x)**2)+')', 400.0) for x in [i*math.pi/4 for i in range(0,5)]]
    #anim_locomote.add_param_objective(faceObjs)

    #minimize acceleration of the torso rotation (so brain is not jostled, and eyes can see etc.)
    torsoPrev = [bq(t-1) for bq in torso.q[3:]]
    torsoCurr = [bq(t) for bq in torso.q[3:]]
    torsoNext = [bq(t+1) for bq in torso.q[3:]]
    for i in range(3):
	vPrev = torsoCurr[i] - torsoPrev[i]
	vNext = torsoNext[i] - torsoCurr[i]
	accel = vNext - vPrev
	#anim_locomote.add_objective('sum {t in sTimeSteps: t>pTimeBegin && t<pTimeEnd} ('+ampl((accel)**2)+')', 1000.0)
        anim_locomote.add_objective(Objective("smooth",accel**2, 1000.0, 't>pTimeBegin && t<pTimeEnd'))

    anim_locomote.add_character(char_mortimer)
    anim_locomote.generate()
    
    #wait for them all to solve
    anim_idle.wait_for_results()
    anim_startstop.wait_for_results()
    anim_locomote.wait_for_results()

    #experiment to see if we can blend them:
    #=================================================
    animList = anim_idle.AnimationList + anim_startstop.AnimationList + anim_locomote.AnimationList
    slicedAnimList = [[] for i in range(2**2)]  #TODO: fix this for other characters

    for anim in animList:

        #discard the poor ones
        if anim.ObjectiveValue > 50:
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
                newanim = anim.get_frame_slice(f[0], f[len(f)-1])
                newanim.Name = str(i) + "_" + newanim.Name + "_" + str(f[0]) + "to" + str(f[len(f)-1])
                slicedAnimList[i].append(newanim)

    for i in slicedAnimList:
        for anim in i:
            anim.export('.\\blended')

    for i in slicedAnimList:
        for anim in i:
            print "<animationlink skeletonName=\""+anim.Name+".skeleton\" />"

    for i in slicedAnimList:
        for anim in i:
            print "\""+anim.Name+"\","

    #now blend them, 2 at a time
    blendedAnimList = []
    for i in slicedAnimList:
        for x in itertools.combinations(i, 2):
            newanim = x[0].blend(x[1], 0.5)
            newanim.Name = x[0].Name + "_blend0.5_" + x[1].Name
            blendedAnimList.append(newanim)

    for i in blendedAnimList:
        anim.export('.\\blended')

    for i in blendedAnimList:
        print "<animationlink skeletonName=\""+anim.Name+".skeleton\" />"

    for i in blendedAnimList:
        print "\""+anim.Name+"\","
    #=================================================

    #export the original animations
    anim_idle.export('.')
    anim_startstop.export('.')
    anim_locomote.export('.')

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
    #anim_test.add_constraint(ConstraintEq('spinX', (body.q[3](1) - body.q[3](0)), 5.0*pH, TimeRange='t = 0'))
    #anim_test.add_constraint(ConstraintEq('spinY', (body.q[4](1) - body.q[4](0)), 0*pH, TimeRange='t = 0'))
    #anim_test.add_constraint(ConstraintEq('spinZ', (body.q[5](1) - body.q[5](0)), 0.5*pH, TimeRange='t = 0'))

    #initial conditions
    #anim_test.add_constraint(ConstraintEq('TX0', body.q[0](0), 0.0))
    #anim_test.add_constraint(ConstraintEq('TX1', body.q[0](1), 0.0))

    #anim_test.add_constraint(ConstraintEq('TY0', body.q[1](0), 0.0))
    #anim_test.add_constraint(ConstraintEq('TY1', body.q[1](1), 0.0)) #m/s

    #anim_test.add_constraint(ConstraintEq('TZ0', body.q[2](0), 0.0))
    #anim_test.add_constraint(ConstraintEq('TZ1', body.q[2](1), 0.0))

    #anim_test.add_constraint(ConstraintEq('RX0', body.q[3](0), 0.1))
    #anim_test.add_constraint(ConstraintEq('RX1', body.q[3](1), 0.0*pH)) #rad/s

    #anim_test.add_constraint(ConstraintEq('RY0', body.q[4](0), 0.0))
    #anim_test.add_constraint(Constraint('RY1', lb=5.0*pH, c=body.q[4](1))) #rad/s

    anim_test.add_constraint(ConstraintEq('RZ0', body.q[5](0), 0.1))
    #anim_test.add_constraint(ConstraintEq('RZ1', body.q[5](1), 0.5*pH)) #rad/s


    #stay above ground plane
    #anim_test.add_constraint(ConstraintPluginGroundPlane())

    #start within 1 unit of origin
    #anim_test.add_constraint(Constraint("startNearOrigin", c=body.q[0](0)**2 + body.q[1](0)**2 + body.q[2](0)**2, ub=1.0**2, TimeRange='t = 0'))

    #anim_test.add_objective('sum {t in sTimeSteps} (' + ampl(body.q[0](t)**2 + body.q[1](t)**2 + body.q[2](t)**2) + ')', 1.0)

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
    anim_dougtest.add_constraint(ConstraintPluginGroundPlane())
    anim_dougtest.add_constraint(ConstraintPluginLoop([1.4, 0, 0, 0, 0, 0], [0, 0, 0]))
    anim_dougtest.add_constraint(Constraint("startAtOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0'))
    anim_dougtest.add_objective(Objective("energy",
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

    print "Exit"
    return

#this check is necessary for multiprocessing to work
if __name__ == '__main__':
    import sys
    print(sys.argv)
    print('Running Python '+sys.version)

    main()
