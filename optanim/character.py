from __future__ import division
import math
import sympy

from constraint import *
from joints import *
from utils import *

class Character(object):
    '''Represents a physically simulated character (composed of bodies and joints).'''
    def __init__(self, Name):
	'''Constructor, creates an empty character (no bodies or joints)'''
	self.Name = Name
	self.BodyList = []
	self.JointList = []

    def add_body(self, body):
	self.BodyList.append(body)

    def add_joint(self, joint):
	self.JointList.append(joint)
	#for convenience add some links to parent/child bodies
	#this assumes bodyA is "up" to the root
	if isinstance(joint, JointRevolute):
	    joint.BodyA.add_child(joint.BodyB)
	    joint.BodyB.set_parent(joint.BodyA)

    def get_joints_contact(self):
	'''Returns a list of this characters contact joints'''
	retList = []
        for j in self.JointList:
	    if isinstance(j, JointContact):
		retList.append(j)
	return retList

    def get_newtonian_constraints(self, name, tPrev=t-1, tCurr=t, tNext=t+1, tRange='pTimeBegin < t < pTimeEnd', offset=[0,0,0]):
	#timestep length
	pH = sympy.Symbol("pH")

	#gravity vector
	#TODO: find a better place for gravity
	g = sympy.Matrix([[0, -9.81, 0]])

	#make the state vector q
	qList = []
	for body in self.BodyList:
	    qList.extend(body.q)
	q = sympy.Matrix(qList).T

	#make the mass vector m
	mList = []
	for body in self.BodyList:
	    mList.extend(body.Mass)
	m = sympy.Matrix(mList).T

	#make the joint force vector jf
	jfList = []
	for joint in self.JointList:
	    jfList.extend(joint.f)
	jf = sympy.Matrix(jfList).T

	#setup joint constraint forces
	jcList = []
	for joint in self.JointList:
	    jcList.extend(joint.get_state_constraints())

	jc = sympy.Matrix([x.c for x in jcList])  #take only constraint (not bounds)

	#----------------------------------------------------
	#TODO: FIXME: this is dumb, but jacobian doesn't work with functions,
	# so we take off the (t), and then add it back afterwards
	for i in q:
	    jc = jc.subs(i(t), i)

	#jacobian
	J = jc.jacobian(q)
	Jlam = J.T * jf.T

	#add the (t) back to each time-dependant variable
	for i in q:
	    Jlam = Jlam.subs(i, i(tCurr))
	for i in jf:
	    #note: we do tNext here so that the forces on frame t
	    #will explain movement between frame t-1 and frame t
	    Jlam = Jlam.subs(i, i(tNext))
	#----------------------------------------------------

	constraints = []
	for i, x in enumerate(q):
	    a = (x(tNext) - 2 * x(tCurr) + x(tPrev) + offset[i % 3]) / (pH ** 2)
	    fm = g[i % 3] + (Jlam[i] / m[i])
	    afm = ConstraintEq(name + str(i), a, fm, tRange)
	    constraints.append(afm)

	return constraints

    def get_model(self):
	model = ''

	#write state variables
	for body in self.BodyList:
	    for q in body.q:
		model += ('var %s {sTimeSteps};\n' % q)
	model += '\n'
	
	#write joint force variables
	for joint in self.JointList:
	    for f in joint.f:
		model += ('var %s {sTimeSteps};\n' % f)
	model += '\n'
	
	#write newtonian constraints
	newtonList = self.get_newtonian_constraints('AFM')
	for c in newtonList:
	    model += (str(c))
	model += '\n'

	#joint constraints
	for joint in self.JointList:
	    for i, eq in enumerate(joint.get_state_constraints() + joint.get_force_constraints()):
		model += (str(eq))
	    model += '\n'

	return model

    def get_bvh_hierarchy(self, root, level, rootoffset):
	ret = ''
	tab = '\t' * level
	if level == 0:
	    #special case for root
	    ret += '%sROOT %s\n' % (tab, root.Name)
	    ret += '%s{\n' % tab
	    level += 1; tab = '\t' * level;
	    ret += '%sOFFSET\t%f\t%f\t%f\n' % tuple([tab] + rootoffset)
	    ret += '%sCHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\n' % tab
	else:
	    #regular case
	    ret += '%sJOINT %s\n' % (tab, root.Name)
	    ret += '%s{\n' % tab
	    level += 1; tab = '\t' * level;
	    ret += '%sOFFSET\t%f\t%f\t%f\n' % tuple([tab] + [0, 0, -root.Parent.Length])
	    ret += '%sCHANNELS 3 Zrotation Xrotation Yrotation\n' % tab
	if len(root.ChildList) > 0:
	    for child in root.ChildList:
		ret += self.get_bvh_hierarchy(child, level, rootoffset)
	else:
	    ret += '%sEnd Site\n' % tab
	    ret += '%s{\n' % tab
	    level += 1; tab = '\t' * level;
	    ret += '%sOFFSET\t%f\t%f\t%f\n' % tuple([tab] + [0, 0, -root.Length])
	    level -= 1; tab = '\t' * level;
	    ret += '%s}\n' % tab
	level -= 1; tab = '\t' * level;
	ret += '%s}\n' % tab
	return ret

    def get_bvh_motion(self, root, level, frame, data):
	ret = ''
	if level == 0:
	    #special case for root
	    #position of root
	    q2 = data[str(root.q[2])][frame]
	    q1 = data[str(root.q[1])][frame]
	    q0 = data[str(root.q[0])][frame]
	    x = (math.cos(q2 + (math.pi / 2.0)) * root.Length / 2.0) + q0
	    y = (math.sin(q2 + (math.pi / 2.0)) * root.Length / 2.0) + q1
	    ret += '%.8f %.8f %.8f ' % (0.0, x, y)

	    #rotation of root
	    r = q2 * (180.0 / math.pi)
	    ret += '%.8f %.8f %.8f ' % (0.0, r, 0.0)
	else:
	    #regular case
	    #rotation
	    q2 = data[str(root.q[2])][frame]
	    q2p = data[str(root.Parent.q[2])][frame]
	    r = (-q2p + q2) * (180.0 / math.pi)
	    ret += '%.8f %.8f %.8f ' % (0.0, r, 0.0)

	level += 1;
	for child in root.ChildList:
	    ret += self.get_bvh_motion(child, level, frame, data)
	level -= 1;

	return ret

    def export_bvh(self, animationData, frameCount, frameLength):
	ret = ''
	root = self.BodyList[0]

	#write hierarchy
	ret += 'HIERARCHY\n'
	ret += self.get_bvh_hierarchy(root, 0, [0, 0, 1.57])	#TODO: magic number

	#write motion
	ret += 'MOTION\n'
	ret += 'Frames: %i\n' % frameCount
	ret += 'Frame Time: %f\n' % frameLength
	for frame in range(0,frameCount):
	    ret += self.get_bvh_motion(root, 0, frame, animationData)
	    ret += '\n'
	ret += '\n'

	return ret