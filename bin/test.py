from __future__ import division

from optanim.animation import *
from optanim.character import *
from optanim.constraint import *
from optanim.constraintplugins import *
from optanim.joints import *
from optanim.rigidbody import *

def main():
    #new character
    character = Character('Mortimer')

    #define some bodies in character space
    #x+- is character front, back (ventral, dorsal)
    #y+- is character up, down (cranial, caudal)
    #z+- is character right, left (lateral, lateral)
    torso = RigidBody("C_torso", 26.05, [0.21, 0.52, 0.27])
    character.add_body(torso)
    thigh_left = RigidBody("L_leg_upper", 6.41, [0.16, 0.42, 0.16])
    character.add_body(thigh_left)
    calf_left = RigidBody("L_leg_lower", 3.13, [0.11, 0.43, 0.11])
    character.add_body(calf_left)
    thigh_right = RigidBody("R_leg_upper", 6.41, [0.16, 0.42, 0.16])
    character.add_body(thigh_right)
    calf_right = RigidBody("R_leg_lower", 3.13, [0.11, 0.43, 0.11])
    character.add_body(calf_right)

    #define some joints to constrain the bodies together
    joint_hip_left = JointRevolute("L_hip", torso, [0.0, torso.ep_b()[1], -0.1], thigh_left, thigh_left.ep_a(), [[-0.8, 0.4], [-0.5, 0.7], [-1.5, 0.1]], 300)
    character.add_joint(joint_hip_left)
    joint_knee_left = JointRevolute("L_knee", thigh_left, thigh_left.ep_b(), calf_left, calf_left.ep_a(), [[0,0], [0,0], [0, 2.8]], 200)
    character.add_joint(joint_knee_left)
    joint_hip_right = JointRevolute("R_hip", torso, [0.0, torso.ep_b()[1], 0.1], thigh_right, thigh_right.ep_a(), [[-0.4, 0.8], [-0.7, 0.5], [-1.5, 0.1]], 300)
    character.add_joint(joint_hip_right)
    joint_knee_right = JointRevolute("R_knee", thigh_right, thigh_right.ep_b(), calf_right, calf_right.ep_a(), [[0,0], [0,0], [0, 2.8]], 200)
    character.add_joint(joint_knee_right)

    #define an (unpowered) contact joint for each foot
    #(character may push against ground plane with these points)
    joint_foot_left = JointContact("L_foot", calf_left, calf_left.ep_b(), Friction=0.5)
    character.add_joint(joint_foot_left)
    joint_foot_right = JointContact("R_foot", calf_right, calf_right.ep_b(), Friction=0.5)
    character.add_joint(joint_foot_right)

    #create an animimation specification
    anim_locomote = AnimationSpec(Name='locomote', FPS=25)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    anim_locomote.set_length(1.0)
    anim_locomote.set_contact_times({
    	joint_foot_left:[(0.37056432, 0.46671992)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.83310183, 0.52637255)]	#contact starting at x, lasting for y
    })

    #turning
    turnRadii = [0.64, 1.0, 1.44, 1.96, 2.56]
    c_turning = [[
	ConstraintPluginLoop([0, 0, 0, 0, 0, 0], [0, 1.4/r, 0]),
	ConstraintEq("startTorso_ry", torso.q[4](t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tx", torso.q[0](t), 0, TimeRange='t = 0'),
	ConstraintEq("startTorso_tz", torso.q[2](t), r, TimeRange='t = 0')] for r in turnRadii]

    #straight
    c_straight = [[
	ConstraintPluginLoop([1.4, 0, 0, 0, 0, 0], [0, 0, 0]),
	ConstraintEq("startTorso_ry", torso.q[4](t), 0, TimeRange='t = 0'),
	Constraint("startTorsoNearOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1.0 ** 2, TimeRange='t = 0')]]
    anim_locomote.add_param_constraint(c_straight + c_turning)
    
    #crouched and upright
    #headPoint = world_xf(torso.ep_b(), [bq(t) for bq in torso.q])
    #c_crouched = [Constraint("crouched", c=headPoint[1], ub=0.5)]
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
	ampl(joint_hip_left.f[3](t)**2 +
	joint_hip_left.f[4](t)**2 +
	joint_hip_left.f[5](t)**2 +
	joint_knee_left.f[5](t)**2 +

	joint_hip_right.f[3](t)**2 +
	joint_hip_right.f[4](t)**2 +
	joint_hip_right.f[5](t)**2 +
	joint_knee_right.f[5](t)**2) +
	')', 0.01)

    #minimize rotation of torso on x and z axes (people tend to walk upright)
    #anim_locomote.add_objective('sum {t in sTimeSteps} (' +
	#ampl(torso.q[3](t)**2 + torso.q[5](t)**2) +
	#')', 300.0)

    #faceObjs = [('sum {t in sTimeSteps} ('+ampl((torso.q[4](t) - x)**2)+')', 400.0) for x in [i*math.pi/4 for i in range(0,5)]]
    #anim_locomote.add_param_objective(faceObjs)

    #minimize acceleration of the torso rotation (so brain is not jostled, and eyes can see etc.)
    torsoPrev = [bq(t-1) for bq in torso.q[3:]]
    torsoCurr = [bq(t) for bq in torso.q[3:]]
    torsoNext = [bq(t+1) for bq in torso.q[3:]]
    #torsoNextNext = [bq(t+2) for bq in torso.q[3:]]
    for i in range(3):
	vPrev = torsoCurr[i] - torsoPrev[i]
	vNext = torsoNext[i] - torsoCurr[i]
	#vNextNext = torsoNextNext[i] - torsoNext[i]
	aPrev = vNext - vPrev
	#aNext = vNextNext - vNext

	anim_locomote.add_objective('sum {t in sTimeSteps: t>pTimeBegin && t<pTimeEnd} ('+ampl((aPrev)**2)+')', 100000.0)

    '''headPointPrev = world_xf(torso.ep_a(), [bq(t-1) for bq in torso.q])
    headPointCurr = world_xf(torso.ep_a(), [bq(t) for bq in torso.q])
    headPointNext = world_xf(torso.ep_a(), [bq(t+1) for bq in torso.q])

    for i in range(3):
	vNext = headPointNext[i] - headPointCurr[i]
	vPrev = headPointCurr[i] - headPointPrev[i]
	a = vNext - vPrev
	anim_locomote.add_objective('sum {t in sTimeSteps: t>pTimeBegin && t<pTimeEnd} ('+ampl(a**2)+')', 200000.0)'''

    anim_locomote.add_character(character)
    anim_locomote.generate('.', 'ipopt')

    print "Exit"
    return

#this check is necessary for multiprocessing to work
if __name__ == '__main__':
    import sys
    print(sys.argv)
    print('Running Python '+sys.version)

    main()