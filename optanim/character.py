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
	#for convenience add some links to parent/child bodies and joints
	#this assumes bodyA is "up" to the root; this must be considered when creating joints
	if isinstance(joint, JointRevolute):
	    joint.BodyA.add_child(joint.BodyB)
	    joint.BodyB.set_parent(joint.BodyA)
	    joint.BodyA.add_child_joint(joint)
	    joint.BodyB.set_parent_joint(joint)

    def get_joints_contact(self):
	'''Returns a list of this characters contact joints'''
	retList = []
        for j in self.JointList:
	    if isinstance(j, JointContact):
		retList.append(j)
	return retList

    def get_newtonian_constraints(self, name, tPrev=t-1, tCurr=t, tNext=t+1, tRange='pTimeBegin < t < pTimeEnd', offset=[0]*dof):
	#timestep length
	pH = sympy.Symbol("pH")

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
	    a = (x(tNext) - 2 * x(tCurr) + x(tPrev) + offset[i % dof]) / (pH ** 2)
	    fm = g[i % dof] + (Jlam[i] / m[i])
	    afm = ConstraintEq(name + str(i), a, fm, tRange)
	    constraints.append(afm)

	return constraints

    def get_model(self):
	model = ''

	#state variables
	for body in self.BodyList:
	    for q in body.q:
		model += ('var %s {sTimeSteps};\n' % q)
	model += '\n'
	
	#joint force variables
	for joint in self.JointList:
	    for f in joint.f:
		model += ('var %s {sTimeSteps};\n' % f)
	model += '\n'
	
	#newtonian constraints
	newtonList = self.get_newtonian_constraints('AFM')
	for c in newtonList:
	    model += (str(c))
	model += '\n'

	#joint constraints
	for joint in self.JointList:
	    for i, eq in enumerate(joint.get_state_constraints() + joint.get_force_constraints()):
		model += (str(eq))
	    model += '\n'

	#self-intersection constraints
	for bodyA in self.BodyList:
	    for bodyB in self.BodyList:
		if bodyA is not bodyB:
		    #endpoint A
		    '''
		    point = bodyB.ep_a()
		    point[1] -= 0.05
		    worldpoint = world_xf(point, [bq(t) for bq in bodyB.q])
		    eq = bodyA.get_intersection_constraint(worldpoint)
		    eq.Name += '_'+bodyB.Name+'_epa'
		    model += (str(eq))

		    #endpoint B
		    point = bodyB.ep_b()
		    point[1] += 0.05
		    worldpoint = world_xf(point, [bq(t) for bq in bodyB.q])
		    eq = bodyA.get_intersection_constraint(worldpoint)
		    eq.Name += '_'+bodyB.Name+'_epb'
		    model += (str(eq))
		    '''

		    #center point
		    eq = bodyA.get_intersection_constraint([bq(t) for bq in bodyB.q[:3]])
		    eq.Name += '_'+bodyB.Name+'_center'
		    model += (str(eq))
	    model += '\n'

	return model