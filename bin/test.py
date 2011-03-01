from __future__ import division

from optanim.animation import *
from optanim.character import *
from optanim.constraint import *
from optanim.constraintplugins import *
from optanim.joints import *
from optanim.rigidbody import *

def main():
    #new character
    character = Character('Felix')

    #define some bodies
    torso = RigidBody("bto", 40.91, 0.52)
    character.add_body(torso)
    thigh_left = RigidBody("btl", 10.34, 0.61)
    character.add_body(thigh_left)
    calf_left = RigidBody("bcl", 5.05, 0.44)
    character.add_body(calf_left)
    thigh_right = RigidBody("btr", 10.34, 0.61)
    character.add_body(thigh_right)
    calf_right = RigidBody("bcr", 5.05, 0.44)
    character.add_body(calf_right)

    #define some powered revolute joints
    joint_hip_left = JointRevolute("jhl", torso, torso.ep_b(), thigh_left, [0, thigh_left.Length / 2.0], [-1.5, 0.1], 500)
    character.add_joint(joint_hip_left)
    joint_knee_left = JointRevolute("jkl", thigh_left, thigh_left.ep_b(), calf_left, calf_left.ep_a(), [0, 2.5], 500)
    character.add_joint(joint_knee_left)
    joint_hip_right = JointRevolute("jhr", torso, torso.ep_b(), thigh_right, [0, thigh_right.Length / 2.0], [-1.5, 0.1], 500)
    character.add_joint(joint_hip_right)
    joint_knee_right = JointRevolute("jkr", thigh_right, thigh_right.ep_b(), calf_right, calf_right.ep_a(), [0, 2.5], 500)
    character.add_joint(joint_knee_right)

    #define an (unpowered) contact joint for each foot
    #(character may push against ground plane with these points)
    joint_foot_left = JointContact("jfl", calf_left, calf_left.ep_b(), Friction=0.5)
    character.add_joint(joint_foot_left)
    joint_foot_right = JointContact("jfr", calf_right, calf_right.ep_b(), Friction=0.5)
    character.add_joint(joint_foot_right)

    #create an animimation
    anim_locomote = AnimationSpec(Name='locomote', FPS=50)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    '''anim_locomote.set_length(0.47528643)
    anim_locomote.set_contact_times({
    	joint_foot_left:[(0.640389, 0.12789167)],	#start contact at 0.0, lasting for 0.5
    	joint_foot_right:[(0.99982742, 0.10499423)]	#start contact at 0.5, lasting for 0.5
    })'''
    anim_locomote.set_length(0.7667)
    anim_locomote.set_contact_times({
    	joint_foot_left:[(0.1, 0.1)],	#start contact at 0.0, lasting for 0.5
    	joint_foot_right:[(0.6, 0.1)]	#start contact at 0.5, lasting for 0.5
    })

    #loop constraint with 3 movement speeds: 1.4 m/s (walk), 3.1 m/s (jog), 6.0 m/s (run)
    anim_locomote.add_param_constraint([ConstraintPluginLoop([x, 0, 0]) for x in [3.0]])

    #stay above ground plane
    anim_locomote.add_constraint(ConstraintPluginGroundPlane())

    #this is a test of a parameterized constraint
    #"start within +/-1 units of X, where X is 2,4,6,8"
    #paramConstraint = [Constraint("startAt"+str(x), c=(torso.q[0](t)-x)**2,ub=1**2,timeRange='t=0') for x in range(2,10,2)]
    #paramConstraint = [Constraint("startAtOrigin", c=(torso.q[0](t)-0)**2,ub=1**2,timeRange='t=0'), '']	#example of an on/off parameterized constraint
    #anim_locomote.add_param_constraint(paramConstraint)
    anim_locomote.add_constraint(Constraint("startAtOrigin", c=torso.q[0](t) ** 2, ub=1 ** 2, TimeRange='t = 0')) #start within 1 unit of origin

    #minimize torso rotation
    #anim_locomote.add_objective('sum {t in sTimeSteps} (' +
			    #str(torso.q[2]) + '[t]**2)', 100000.0)

    #minimize torques
    #we divide by time so animations of different lengths can be compared fairly
    anim_locomote.add_objective('(sum {t in sTimeSteps} (' +
	str(joint_hip_left.f[2]) + '[t]**2 + ' +
	str(joint_knee_left.f[2]) + '[t]**2 + ' +
	str(joint_hip_right.f[2]) + '[t]**2 + ' +
	str(joint_knee_right.f[2]) + '[t]**2)) / ((pTimeEnd+1)**2)', 1.0)

    anim_locomote.add_character(character)
    anim_locomote.generate('.', 'ipopt')
    #print(character.get_exporter("somefile.bvh"))
    print "Exit"
    return

#this check is necessary for multiprocessing to work
if __name__ == '__main__':
    import sys
    print(sys.argv)
    print('Running Python '+sys.version)
    #import optanim
    #help(optanim.animation)
    main()