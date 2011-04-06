from __future__ import division

from optanim.animation import *
from optanim.character import *
from optanim.constraint import *
from optanim.constraintplugins import *
from optanim.joints import *
from optanim.rigidbody import *

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


def main():
    #new character
    char_mortimer = Character('Mortimer')

    #define some bodies in character space
    #x+- is character front, back (ventral, dorsal)
    #y+- is character up, down (cranial, caudal)
    #z+- is character right, left (lateral, lateral)
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

    R_arm_upper = RigidBody(5, "R_arm_upper", 2.5, [0.076, 0.36, 0.076])
    char_mortimer.add_body(R_arm_upper)
    L_arm_upper = RigidBody(6, "L_arm_upper", 2.5, [0.076, 0.36, 0.076])
    char_mortimer.add_body(L_arm_upper)

    R_arm_lower = RigidBody(7, "R_arm_lower", 1.98, [0.07, 0.36, 0.07])
    char_mortimer.add_body(R_arm_lower)
    L_arm_lower = RigidBody(8, "L_arm_lower", 1.98, [0.07, 0.36, 0.07])
    char_mortimer.add_body(L_arm_lower)

    #define some joints to constrain the bodies together
    joint_hip_left = JointRevolute("L_hip", torso, [0.0, torso.ep_b()[1], -0.1], thigh_left, thigh_left.ep_a(), [[-0.8, 0.4], [-0.5, 0.7], [-1.5, 0.1]], 300)
    char_mortimer.add_joint(joint_hip_left)
    joint_knee_left = JointRevolute("L_knee", thigh_left, thigh_left.ep_b(), calf_left, calf_left.ep_a(), [[0,0], [0,0], [0, 2.8]], 200)
    char_mortimer.add_joint(joint_knee_left)
    joint_hip_right = JointRevolute("R_hip", torso, [0.0, torso.ep_b()[1], 0.1], thigh_right, thigh_right.ep_a(), [[-0.4, 0.8], [-0.7, 0.5], [-1.5, 0.1]], 300)
    char_mortimer.add_joint(joint_hip_right)
    joint_knee_right = JointRevolute("R_knee", thigh_right, thigh_right.ep_b(), calf_right, calf_right.ep_a(), [[0,0], [0,0], [0, 2.8]], 200)
    char_mortimer.add_joint(joint_knee_right)

    joint_shoulder_right = JointRevolute("R_shoulder", torso, [torso.ep_a()[0], torso.ep_a()[1]-0.05, torso.ep_a()[2]+0.16], R_arm_upper, R_arm_upper.ep_a(), [[-3.2, 0.2], [-1.4, 2.8], [-1.5, 3.1]], 8)
    char_mortimer.add_joint(joint_shoulder_right)
    joint_shoulder_left = JointRevolute("L_shoulder", torso, [torso.ep_a()[0], torso.ep_a()[1]-0.05, torso.ep_a()[2]-0.16], L_arm_upper, L_arm_upper.ep_a(), [[-0.2, 3.2], [-2.8, 1.4], [-3.1, 1.5]], 8)
    char_mortimer.add_joint(joint_shoulder_left)

    joint_elbow_right = JointRevolute("R_elbow", R_arm_upper, R_arm_upper.ep_b(), R_arm_lower, R_arm_lower.ep_a(), [[0,0], [0,0], [0, 2.8]], 5)
    char_mortimer.add_joint(joint_elbow_right)
    joint_elbow_left = JointRevolute("L_elbow", L_arm_upper, L_arm_upper.ep_b(), L_arm_lower, L_arm_lower.ep_a(), [[0,0], [0,0], [0, 2.8]], 5)
    char_mortimer.add_joint(joint_elbow_left)

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
    anim_idle.set_length(1.0)
    anim_idle.set_contact_times({
    	joint_foot_left:[(0.0, 1.0)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.0, 1.0)]	#contact starting at x, lasting for y
    })
    anim_idle.add_constraint(ConstraintPluginGroundPlane())
    anim_idle.add_constraint(Constraint("startAtOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0'))
    anim_idle.add_constraint(ConstraintPluginLoop([0, 0, 0, 0, 0, 0], [0, 0, 0])) #perfect loop

    #crouched and upright
    pelvisPoint = world_xf(torso.ep_b(), [bq(t) for bq in torso.q])
    c_crouched = [Constraint("crouched", c=pelvisPoint[1], ub=0.5)]
    anim_idle.add_param_constraint([[None], c_crouched])

    anim_idle.add_objective('sum {t in sTimeSteps} (' +
	ampl(
	joint_hip_left.f[3](t)**2 +
	joint_hip_left.f[4](t)**2 +
	joint_hip_left.f[5](t)**2 +

	joint_knee_left.f[5](t)**2 +

	joint_elbow_left.f[5](t)**2 +

	joint_shoulder_left.f[3](t)**2 +
	joint_shoulder_left.f[4](t)**2 +
	joint_shoulder_left.f[5](t)**2 +

	joint_hip_right.f[3](t)**2 +
	joint_hip_right.f[4](t)**2 +
	joint_hip_right.f[5](t)**2 +

	joint_knee_right.f[5](t)**2 +

	joint_elbow_right.f[5](t)**2 +

	joint_shoulder_right.f[3](t)**2 +
	joint_shoulder_right.f[4](t)**2 +
	joint_shoulder_right.f[5](t)**2) +
	')', 0.01)
    anim_idle.add_character(char_mortimer)
    anim_idle.generate('.', 'ipopt')

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

    walkSpeed = 1.4

    #straight
    c_straight = [[
	ConstraintPluginLoop([walkSpeed, 0, 0, 0, 0, 0], [0, 0, 0]),
	#ConstraintEq("startTorso_ry", torso.q[4](t), 0, TimeRange='t = 0'),
	Constraint("startTorsoNearOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')]]

    #turning
    turnRadii = [1.0, 1.44, 2.56]
    c_turn = [[
	ConstraintPluginLoop([0, 0, 0, 0, 0, 0], [0, walkSpeed/r, 0]),
	#ConstraintEq("startTorso_ry", torso.q[4](t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tx", torso.q[0](t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tz", torso.q[2](t), r, TimeRange='t = 0')] for r in turnRadii]
    
    anim_locomote.add_param_constraint(c_straight + c_turn)

    #forward and backward facing
    anim_locomote.add_constraint(ConstraintEq("startTorso_ry", torso.q[4](t), 0, TimeRange='t = 0'))

    '''faceDir = [
	[ConstraintEq("startTorso_ry", torso.q[4](t), 0, TimeRange='t = 0')],
	[ConstraintEq("startTorso_ry", torso.q[4](t), math.pi, TimeRange='t = 0')]]
    anim_locomote.add_param_constraint(faceDir)'''

    #crouched and upright
    #pelvisPoint = world_xf(torso.ep_b(), [bq(t) for bq in torso.q])
    #c_crouched = [Constraint("crouched", c=pelvisPoint[1], ub=0.5)]
    #anim_locomote.add_param_constraint([c_crouched, [None]])



    #move forward on x-axis at average speed of 1.4m/s
    #anim_locomote.add_param_constraint([[
	#ConstraintPluginLoop([1.4, 0, 0, 0, 0, 0], [0, 0, 0]),
	#Constraint("startAtOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')
	#]])

    #move forward on x-axis at average speed of 1.4m/s, spinning 360 degrees
    #anim_locomote.add_constraint(ConstraintPluginLoop([1.4, 0, 0, 0, (math.pi*2)/anim_locomote.Length, 0], [0, 0, 0]))

    #move in a circular path (walking turn) at average speed of 1.4m/s, turning at 90 deg/s (turn radius = 0.891m) #v=r*w
    #anim_locomote.add_constraint(ConstraintPluginLoop([1.4, 0, 0, 0, 0, 0], math.pi/2, 0.891))
    #anim_locomote.add_constraint(ConstraintPluginLoop([0, 0, 0, 0, 0, 0], [0, 1.4, 0]))

    #stay above ground plane
    anim_locomote.add_constraint(ConstraintPluginGroundPlane())

    #start within 1 unit of origin
    #anim_locomote.add_constraint(Constraint("startAtOrigin", lb=0.8**2, c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0'))
    #anim_locomote.add_constraint(ConstraintEq("startTorso_ry", torso.q[4](t), 0, TimeRange='t = 0'))
    #anim_locomote.add_constraint(ConstraintEq("startTorso_tx", torso.q[0](t), 0, TimeRange='t = 0'))
    #anim_locomote.add_constraint(ConstraintEq("startTorso_tz", torso.q[2](t), 1, TimeRange='t = 0'))

    #face 0.0, 0.785, 1.571, 2.356, 3.142 rad, to a tolerance of +/- ~0.35 rad
    #we only do a half-circle, since animations can be mirrored
    #faceDirConstraints = [Constraint("faceDirection", c=(torso.q[4](t) - x)**2, ub=0.35**2) for x in [i*math.pi/4 for i in range(0,5)]]
    #anim_locomote.add_param_constraint(faceDirConstraints)

    #minimize torques
    #we divide by time so animations of different lengths can be compared fairly
    anim_locomote.add_objective('sum {t in sTimeSteps} (' +
	ampl(
	joint_hip_left.f[3](t)**2 +
	joint_hip_left.f[4](t)**2 +
	joint_hip_left.f[5](t)**2 +

	joint_knee_left.f[5](t)**2 +

	joint_elbow_left.f[5](t)**2 +

	joint_shoulder_left.f[3](t)**2 +
	joint_shoulder_left.f[4](t)**2 +
	joint_shoulder_left.f[5](t)**2 +

	joint_hip_right.f[3](t)**2 +
	joint_hip_right.f[4](t)**2 +
	joint_hip_right.f[5](t)**2 +

	joint_knee_right.f[5](t)**2 +

	joint_elbow_right.f[5](t)**2 +

	joint_shoulder_right.f[3](t)**2 +
	joint_shoulder_right.f[4](t)**2 +
	joint_shoulder_right.f[5](t)**2) +
	')', 0.01)

    #minimize rotation of torso on x and z axes (people tend to walk upright)
    '''anim_locomote.add_objective('sum {t in sTimeSteps} (' +
	ampl(torso.q[3](t)**2 + torso.q[5](t)**2) +
	')', 100.0)

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
	anim_locomote.add_objective('sum {t in sTimeSteps: t>pTimeBegin && t<pTimeEnd} ('+ampl((accel)**2)+')', 1000.0)'''

    '''headPointPrev = world_xf(torso.ep_a(), [bq(t-1) for bq in torso.q])
    headPointCurr = world_xf(torso.ep_a(), [bq(t) for bq in torso.q])
    headPointNext = world_xf(torso.ep_a(), [bq(t+1) for bq in torso.q])

    for i in range(3):
	vNext = headPointNext[i] - headPointCurr[i]
	vPrev = headPointCurr[i] - headPointPrev[i]
	a = vNext - vPrev
	anim_locomote.add_objective('sum {t in sTimeSteps: t>pTimeBegin && t<pTimeEnd} ('+ampl(a**2)+')', 200000.0)'''

    anim_locomote.add_character(char_mortimer)
    anim_locomote.generate('.', 'ipopt')

    print "Exit"
    return

#this check is necessary for multiprocessing to work
if __name__ == '__main__':
    import sys
    print(sys.argv)
    print('Running Python '+sys.version)

    #quick unit test
    '''eulerIn = [1.0, 6.0, 2.0]
    mat = euler_to_matrix(eulerIn)
    eulerOut = matrix_to_euler(mat)
    eulerOut = map(float, eulerOut)
    print eulerIn, eulerOut
    #assert eulerIn == eulerOut

    quat = matrix_to_quat(mat)
    axisangle = quat_to_axisangle(quat)
    axisangle = map(float, axisangle)
    print axisangle

    aaIn = [0.5,0.5,-0.1,-2.5]
    q = axisangle_to_quat(aaIn)
    aaOut = quat_to_axisangle(q)
    aaOut = map(float, aaOut)
    print aaIn, aaOut

    euler = [
    90*(math.pi/180),
    -90*(math.pi/180),
    0*(math.pi/180)]; print euler
    quat = euler_to_quat(euler); quat = map(float, quat); print quat
    aa = quat_to_axisangle(quat); aa = map(float, aa); print aa'''

    main()
