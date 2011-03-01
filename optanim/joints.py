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
	    sympy.Symbol(Name + "_fx"),
	    sympy.Symbol(Name + "_fy"),
	    sympy.Symbol(Name + "_fr")]
	print 'new revolute joint "' + Name + '": connects ' + str(BodyA.Name) + \
	    ' at ' + str(PointA) + ' to ' + str(BodyB.Name) + ' at ' + str(PointB)

    def get_state_constraints(self):
	#constraints enforced by joint forces
	retList = []
	worldpointA = world_xf(self.PointA, [bq(t) for bq in self.BodyA.q])
	worldpointB = world_xf(self.PointB, [bq(t) for bq in self.BodyB.q])
	retList.append(ConstraintEq(self.Name + "_x", worldpointA[0] - worldpointB[0]))
	retList.append(ConstraintEq(self.Name + "_y", worldpointA[1] - worldpointB[1]))
	retList.append(Constraint(self.Name + "_r", self.RotationLimits[0], self.BodyA.q[2](t) - self.BodyB.q[2](t), self.RotationLimits[1]))
	return retList

    def get_force_constraints(self):
	#constraints on the joint forces themselves
	retList = []
	retList.append(Constraint(self.Name + "_f", lb=self.f[2](t) ** 2, c=self.TorqueLimit ** 2))
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
	    sympy.Symbol(Name + "_fx"),
	    sympy.Symbol(Name + "_fy")]
	print 'new contact joint "' + Name + '": connects ' + str(Body.Name) + ' at ' + str(Point) + ' to ground plane'

    def get_state_constraints(self):
	#constraints enforced by joint forces
	retList = []
	tRangeOn = 't in sTimeSteps_' + self.Name + 'On'

	worldpoint_t0 = world_xf(self.Point, [bq(t) for bq in self.Body.q])
	worldpoint_t1 = world_xf(self.Point, [bq(t-1) for bq in self.Body.q])
	retList.append(ConstraintEq(self.Name + '_state_x', worldpoint_t0[0], worldpoint_t1[0], TimeRange=tRangeOn + ' && t>0'))
	retList.append(ConstraintEq(self.Name + '_state_y', worldpoint_t0[1], TimeRange=tRangeOn))
	return retList

    def get_force_constraints(self):
	#constraints on the joint forces themselves
	retList = []
	tRangeOn = 't in sTimeSteps_' + self.Name + 'On'
	tRangeOff = 't in sTimeSteps_' + self.Name + 'Off'

	#friction constraint: "stay within static friction cone"
	retList.append(Constraint(self.Name + '_force_friction', c=self.f[0](t) ** 2, ub=(self.f[1](t) * self.Friction) ** 2, TimeRange=tRangeOn))

	#contact force may only push (not pull)
	retList.append(Constraint(self.Name + '_force_push', lb=0, c=self.f[1](t), TimeRange=tRangeOn))

	#when joint is off, no force
	for force in self.f:
	    retList.append(ConstraintEq(force.name + '_Off', a=force(t), TimeRange=tRangeOff))

	return retList