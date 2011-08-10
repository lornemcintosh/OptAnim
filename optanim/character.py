from __future__ import division
import math
import sympy
import collections

from specifier import *
from joints import *
from utils import *

class Character(object):
    '''Represents a physically simulated character (composed of bodies and joints).'''
    def __init__(self, Name):
	'''Constructor, creates an empty character (no bodies or joints)'''
	self.Name = Name
	self.BodyList = []
	self.JointList = []
        self.SpecifierList = []
        self.DefaultRoot = None #this body will be used as the root by default

    def get_mass(self):
        return sum([x.Mass for x in self.BodyList])

    def traverse_dfs(self, root):
        '''Perform a depth-first traversal of this character's bodies starting from root, yielding parent/child/joint pairs'''
        if root not in self.BodyList:
            raise BaseException("Character does not contain the specified root body!")
        stack, enqueued = [(None, root)], set([root])
        while stack:
            parent, n = stack.pop()
            yield parent, n, self.get_joint_connecting_bodies(parent, n)
            new = set(self.get_bodies_connected_to_body(n)) - enqueued
            enqueued |= new
            stack.extend([(n, child) for child in new])

    def traverse_bfs(self, root):
        '''Perform a breadth-first traversal of this character's bodies starting from root, yielding parent/child/joint pairs'''
        if root not in self.BodyList:
            raise BaseException("Character does not contain the specified root body!")
        queue, enqueued = collections.deque([(None, root)]), set([root])
        while queue:
            parent, n = queue.popleft()
            yield parent, n, self.get_joint_connecting_bodies(parent, n)
            new = set(self.get_bodies_connected_to_body(n)) - enqueued
            enqueued |= new
            queue.extend([(n, child) for child in new])

    def get_joint_connecting_bodies(self, bodyA, bodyB):
        ret = []
        for joint in self.JointList:
            if not isinstance(joint, JointRevolute):
                continue
            elif((joint.BodyA == bodyA and joint.BodyB == bodyB) or \
                (joint.BodyA == bodyB and joint.BodyB == bodyA)):
                ret.append(joint)
        if len(ret) > 1:
            raise BaseException("Character has 2 joints connecting " + bodyA.Name + " and " + bodyB.Name + " together. We can't handle that.")
        elif len(ret) < 1:
            return None
        else:
            return ret[0]

    def get_bodies_connected_to_body(self, body):
        ret = []
        for joint in self.JointList:
            if not isinstance(joint, JointRevolute):
                continue
            elif(joint.BodyA == body):
                ret.append(joint.BodyB)
            elif(joint.BodyB == body):
                ret.append(joint.BodyA)
        return ret

    def set_default_root(self, body):
        if body in self.BodyList:
            self.DefaultRoot = body
        else:
            raise BaseException("The specified body is not part of this character!")

    def add_body(self, body):
	self.BodyList.append(body)

    def add_joint(self, joint):
	self.JointList.append(joint)

    def add_specifier(self, specifier):
        '''Adds a specifier to the character. These specifiers will
        automatically be added to any animation involving this character'''
	self.SpecifierList.append(specifier)

    def get_joints_contact(self):
	'''Returns a list of this characters contact joints'''
	retList = []
        for j in self.JointList:
	    if isinstance(j, JointContact):
		retList.append(j)
	return retList

    def get_newtonian_constraints(self, name, tPrev=t-1, tCurr=t, tNext=t+1, tRange='pTimeBegin < t < pTimeEnd',
	offset_func_prev=None, offset_dir_prev=0, offset_func_next=None, offset_dir_next=0):

	#make the state vector q
	qList = []
	for body in self.BodyList:
	    qList.extend(body.q)
	q = sympy.Matrix(qList).T

	#make the mass vector m
	mList = []
	for body in self.BodyList:
	    mList.extend(body.get_mass_vector())
	m = sympy.Matrix(mList).T

	Jlam = sympy.Matrix([0]*(len(self.BodyList)*dof))
	if self.JointList:
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
	    #TODO: FIXME: this is dumb, but jacobian() doesn't work with functions,
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

	for b,body in enumerate(self.BodyList):
	    #make lists for next, current, and previous q's
	    pPrev = [x(tPrev) for x in body.q]
	    pCurr = [x(tCurr) for x in body.q]
	    pNext = [x(tNext) for x in body.q]

	    #do offsets if necessary
	    if offset_func_prev is not None:
		pPrev = offset_func_prev(pPrev, offset_dir_prev)
	    if offset_func_next is not None:
		pNext = offset_func_next(pNext, offset_dir_next)

	    #calculate velocities (finite differences)
	    vPrev = [0]*dof; vNext = [0]*dof; vCentral = [0]*dof
	    for k,x in enumerate(body.q):
		vPrev[k] = (pCurr[k] - pPrev[k])/(1*pH)
		vNext[k] = (pNext[k] - pCurr[k])/(1*pH)
		vCentral[k] = (pNext[k] - pPrev[k])/(2*pH)

	    #calculate accelerations (finite differences)
	    aCentral = [0]*dof
	    for k,x in enumerate(body.q):
		aCentral[k] = (vNext[k] - vPrev[k])/(1*pH)

	    #translations (Newton's second law, f=m*a, a=f/m)
	    for k,x in enumerate(body.q[:3]):
		fm = g[k] + (Jlam[b*dof+k] / m[b*dof+k])
		afm = ConstraintEq(name + '_' + body.Name+ '_' + str(k), aCentral[k], fm, tRange)
		constraints.append(afm)

	    #rotations (Euler's equations for rigid body dynamics)
	    constraints.append(ConstraintEq(name + '_' + body.Name+ '_' + str(3),
		m[b*dof+3]*aCentral[3]+(m[b*dof+5] - m[b*dof+4])*vCentral[4]*vCentral[5], Jlam[b*dof+3], tRange))
	    constraints.append(ConstraintEq(name + '_' + body.Name+ '_' + str(4),
		m[b*dof+4]*aCentral[4]+(m[b*dof+3] - m[b*dof+5])*vCentral[5]*vCentral[3], Jlam[b*dof+4], tRange))
	    constraints.append(ConstraintEq(name + '_' + body.Name+ '_' + str(5),
		m[b*dof+5]*aCentral[5]+(m[b*dof+4] - m[b*dof+3])*vCentral[3]*vCentral[4], Jlam[b*dof+5], tRange))

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
		    '''
		    #endpoint A
		    point = bodyB.ep_a()
		    point[1] -= 0.01    #bring point inside just a little
		    worldpoint = sym_world_xf(point, [bq(t) for bq in bodyB.q])
		    eq = bodyA.get_intersection_constraint(worldpoint)
		    eq.Name += '_'+bodyB.Name+'_epa'
		    model += (str(eq))

		    #endpoint B
		    point = bodyB.ep_b()
		    point[1] += 0.01	#bring point inside just a little
		    worldpoint = sym_world_xf(point, [bq(t) for bq in bodyB.q])
		    eq = bodyA.get_intersection_constraint(worldpoint)
		    eq.Name += '_'+bodyB.Name+'_epb'
		    model += (str(eq))
		    '''

		    #center point
		    eq = bodyA.get_intersection_constraint([bq(t) for bq in bodyB.q[:3]], min(bodyB.Diameter))
		    eq.Name += '_'+bodyB.Name+'_center'
		    model += (str(eq))
	    model += '\n'

	return model