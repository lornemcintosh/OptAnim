from __future__ import division

from optanim.animation import *
from optanim.character import *
from optanim.constraint import *
from optanim.constraintplugins import *
from optanim.joints import *
from optanim.rigidbody import *

def main():
    #new character
    character = Character('Aneta')
    #height=164cm
    #mass=51kg
    #bust=82cm
    #waist=67cm
    #hips=91cm

    #define some bodies in character space
    #x+- is character front, back (ventral, dorsal)
    #y+- is character up, down (cranial, caudal)
    #z+- is character right, left (lateral, lateral)

    torso = RigidBody("bto", 26.05, [0.21, 0.52, 0.27])
    character.add_body(torso)
    thigh_left = RigidBody("btl", 6.41, [0.16, 0.42, 0.16])
    character.add_body(thigh_left)
    calf_left = RigidBody("bcl", 3.13, [0.11, 0.43, 0.11])
    character.add_body(calf_left)
    thigh_right = RigidBody("btr", 6.41, [0.16, 0.42, 0.16])
    character.add_body(thigh_right)
    calf_right = RigidBody("bcr", 3.13, [0.11, 0.43, 0.11])
    character.add_body(calf_right)

    #define some powered revolute joints
    '''joint_hip_left = JointRevolute("jhl", torso, [0.0, torso.ep_b()[1], 0.0], thigh_left, thigh_left.ep_a(), [[-0.7, 1.4], [-0.5, 0.7], [-1.5, 0.1]], 2500)
    character.add_joint(joint_hip_left)
    joint_knee_left = JointRevolute("jkl", thigh_left, thigh_left.ep_b(), calf_left, calf_left.ep_a(), [[0,0], [0,0], [0, 2.5]], 2500)
    character.add_joint(joint_knee_left)'''

    joint_hip_left = JointRevolute("jhl", torso, [0.0, torso.ep_b()[1], -0.1], thigh_left, thigh_left.ep_a(), [[-0.8, 0.4], [-0.5, 0.7], [-1.5, 0.1]], 300)
    character.add_joint(joint_hip_left)
    joint_knee_left = JointRevolute("jkl", thigh_left, thigh_left.ep_b(), calf_left, calf_left.ep_a(), [[0,0], [0,0], [0, 2.8]], 300)
    character.add_joint(joint_knee_left)
    joint_hip_right = JointRevolute("jhr", torso, [0.0, torso.ep_b()[1], 0.1], thigh_right, thigh_right.ep_a(), [[-0.4, 0.8], [-0.7, 0.5], [-1.5, 0.1]], 300)
    character.add_joint(joint_hip_right)
    joint_knee_right = JointRevolute("jkr", thigh_right, thigh_right.ep_b(), calf_right, calf_right.ep_a(), [[0,0], [0,0], [0, 2.8]], 300)
    character.add_joint(joint_knee_right)

    #define an (unpowered) contact joint for each foot
    #(character may push against ground plane with these points)
    joint_foot_left = JointContact("jfl", calf_left, calf_left.ep_b(), Friction=0.5)
    character.add_joint(joint_foot_left)
    joint_foot_right = JointContact("jfr", calf_right, calf_right.ep_b(), Friction=0.5)
    character.add_joint(joint_foot_right)

    #create an animimation
    anim_locomote = AnimationSpec(Name='locomote', FPS=22)

    #specify anim length and contact joints timings (i.e. footsteps)
    #contact timings given as a fraction of the total animation length
    anim_locomote.set_length(1.11698)
    anim_locomote.set_contact_times({
    	joint_foot_left:[(0.37056432, 0.46671992)],	#contact starting at x, lasting for y
    	joint_foot_right:[(0.83310183, 0.52637255)]	#contact starting at x, lasting for y
    })

    #loop constraint with 3 movement speeds: 1.4 m/s (walk), 3.1 m/s (jog), 6.0 m/s (run)
    anim_locomote.add_param_constraint([ConstraintPluginLoop([x, 0, 0, 0, 0, 0]) for x in [1.4, 3.1, 6.0]])

    #stay above ground plane
    anim_locomote.add_constraint(ConstraintPluginGroundPlane())

    #this is a test of a parameterized constraint
    #"start within +/-1 units of X, where X is 2,4,6,8"
    #paramConstraint = [Constraint("startAt"+str(x), c=(torso.q[0](t)-x)**2,ub=1**2,timeRange='t=0') for x in range(2,10,2)]
    #paramConstraint = [Constraint("startAtOrigin", c=(torso.q[0](t)-0)**2,ub=1**2,timeRange='t=0'), '']	#example of an on/off parameterized constraint
    #anim_locomote.add_param_constraint(paramConstraint)
    anim_locomote.add_constraint(Constraint("startAtOrigin", c=torso.q[0](t) ** 2 + torso.q[2](t) ** 2, ub=1 ** 2, TimeRange='t = 0')) #start within 1 unit of origin

    #minimize torques
    #we divide by time so animations of different lengths can be compared fairly
    anim_locomote.add_objective('(sum {t in sTimeSteps} (' +
	str(joint_hip_left.f[3]) + '[t]**2 + ' +
	str(joint_hip_left.f[4]) + '[t]**2 + ' +
	str(joint_hip_left.f[5]) + '[t]**2 + ' +
	str(joint_knee_left.f[5]) + '[t]**2 + ' +

	str(joint_hip_right.f[3]) + '[t]**2 + ' +
	str(joint_hip_right.f[4]) + '[t]**2 + ' +
	str(joint_hip_right.f[5]) + '[t]**2 + ' +
	str(joint_knee_right.f[5]) + '[t]**2' +
	')) / ((pTimeEnd+1)**2)', 1.0)

    #minimize rotation of torso around y-axis (heading) from forwards
    anim_locomote.add_objective('(sum {t in sTimeSteps} (' +
	str(torso.q[4]) + '[t]**2' +
	')) / ((pTimeEnd+1)**2)', 5000.0)

    #anim_locomote.add_objective('(sum {t in sTimeSteps} (' +
	#world_xf(torso.ep_a(), [bq(t) for bq in torso.q])[1], 1.0)
    
    '''anim_locomote.add_objective('(sum {t in sTimeSteps} (' +
	str(joint_foot_left.f[1]) + '[t]**2 + ' +
	str(joint_foot_right.f[1]) + '[t]**2)) / ((pTimeEnd+1)**2)', 4.0)'''

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