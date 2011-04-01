from __future__ import division

from constraint import *
from utils import *

class Joint(object):
    '''Base class for joints'''
    def __init__(self, Name):
	self.Name = Name
	self.f = []

    #constraints enforced by joint forces
    def get_state_constraints(self): abstract

    #constraints on the joint forces themselves
    def get_force_constraints(self): abstract


class JointRevolute(Joint):
    '''Represents a powered revolute joint.'''

    def __init__(self, Name, BodyA, PointA, BodyB, PointB, RotationLimits, TorqueLimit):
	'''Constructor'''
	Joint.__init__(self, Name)
	self.BodyA = BodyA
	self.BodyB = BodyB
	self.PointA = PointA
	self.PointB = PointB
	self.RotationLimits = RotationLimits
	self.TorqueLimit = TorqueLimit
	self.f = [
	    sympy.Symbol(Name + "_ftx"), #translational
	    sympy.Symbol(Name + "_fty"),
	    sympy.Symbol(Name + "_ftz"),

	    sympy.Symbol(Name + "_frx"), #rotational
	    sympy.Symbol(Name + "_fry"),
	    sympy.Symbol(Name + "_frz")]

	print 'new ' + str(self)
	
    def __str__(self):
	return 'JointRevolute "' + self.Name + '": connects ' + str(self.BodyA.Name) + \
	' at ' + str(self.PointA) + ' to ' + str(self.BodyB.Name) + ' at ' + str(self.PointB)

    def get_state_constraints(self):
	#constraints enforced by joint forces
	retList = []
	worldpointA = world_xf(self.PointA, [bq(t) for bq in self.BodyA.q])
	worldpointB = world_xf(self.PointB, [bq(t) for bq in self.BodyB.q])

	retList.append(ConstraintEq(self.Name + "_tx", worldpointA[0] - worldpointB[0]))
	retList.append(ConstraintEq(self.Name + "_ty", worldpointA[1] - worldpointB[1]))
	retList.append(ConstraintEq(self.Name + "_tz", worldpointA[2] - worldpointB[2]))

	retList.append(Constraint(self.Name + "_rx", self.RotationLimits[0][0], self.BodyA.q[3](t) - self.BodyB.q[3](t), self.RotationLimits[0][1]))
	retList.append(Constraint(self.Name + "_ry", self.RotationLimits[1][0], self.BodyA.q[4](t) - self.BodyB.q[4](t), self.RotationLimits[1][1]))
	retList.append(Constraint(self.Name + "_rz", self.RotationLimits[2][0], self.BodyA.q[5](t) - self.BodyB.q[5](t), self.RotationLimits[2][1]))
	return retList

    def get_force_constraints(self):
	#constraints on the joint forces themselves
	retList = []
	#total torque (on all axes that can move) must be less than the "muscles" limit
	#on axes that can't move, we leave torque unconstrained
	expr = 0
	for i in range(3):
	    if self.RotationLimits[i][0] != self.RotationLimits[i][1]:
		expr += self.f[i+3](t)**2
	if expr != 0:
	    retList.append(Constraint(self.Name + "_f", lb=expr, c=self.TorqueLimit ** 2))
	return retList


class JointContact(Joint):
    '''Represents a (possibly temporary) contact joint.'''

    def __init__(self, Name, Body, Point, Friction):
	'''Constructor'''
	Joint.__init__(self, Name)
	self.Body = Body
	self.Point = Point
	self.Friction = Friction
	self.f = [
	    sympy.Symbol(Name + "_fx"), #translational
	    sympy.Symbol(Name + "_fy"),
	    sympy.Symbol(Name + "_fz"),
	    
	    sympy.Symbol(Name + "_fry")] #rotational

	print 'new '+str(self)
    
    def __str__(self):
	return 'JointContact "' + self.Name + '": connects ' + str(self.Body.Name) + ' at ' + str(self.Point) + ' to ground plane'

    def get_state_constraints(self):
	#constraints enforced by joint forces
	retList = []
	tRangeOn = 't in sTimeSteps_' + self.Name + 'On'

	#'zero velocity' constraints
	worldpoint_t0 = world_xf(self.Point, [bq(t) for bq in self.Body.q])
	worldpoint_t1 = world_xf(self.Point, [bq(t-1) for bq in self.Body.q])
	retList.append(ConstraintEq(self.Name + '_state_x', worldpoint_t0[0], worldpoint_t1[0], TimeRange=tRangeOn + ' && t>pTimeBegin'))
	retList.append(ConstraintEq(self.Name + '_state_y', worldpoint_t0[1], TimeRange=tRangeOn))
	retList.append(ConstraintEq(self.Name + '_state_z', worldpoint_t0[2], worldpoint_t1[2], TimeRange=tRangeOn + ' && t>pTimeBegin'))

	#'zero angular velocity' constraint
	#NB: this only works because our rotation order is YXZ (y-first)
	retList.append(ConstraintEq(self.Name + '_state_ry', self.Body.q[4](t), self.Body.q[4](t-1), TimeRange=tRangeOn + ' && t>pTimeBegin'))
	return retList

    def get_force_constraints(self):
	#constraints on the joint forces themselves
	retList = []
	tRangeOn = 't in sTimeSteps_' + self.Name + 'On'
	tRangeOff = 't in sTimeSteps_' + self.Name + 'Off'

	#friction constraint: "stay within static friction cone"
	#(fx**2 + fz**2) < (fy+u)**2
	retList.append(Constraint(self.Name + '_force_friction', c=self.f[0](t)**2 + self.f[2](t)**2, ub=(self.f[1](t) * self.Friction)**2, TimeRange=tRangeOn))

	#contact force may only push (not pull)
	retList.append(Constraint(self.Name + '_force_push', lb=0, c=self.f[1](t), TimeRange=tRangeOn))

	#when joint is off, no force
	for force in self.f:
	    retList.append(ConstraintEq(force.name + '_Off', a=force(t), TimeRange=tRangeOff))

	return retList