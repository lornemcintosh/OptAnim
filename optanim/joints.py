# -------------------------------------------------------------------------
# Copyright (c) 2010-2012 Lorne McIntosh
#
# This file is part of OptAnim.
#
# OptAnim is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# OptAnim is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with OptAnim.  If not, see <http://www.gnu.org/licenses/>.
# -------------------------------------------------------------------------

'''
OptAnim, joints module
'''

from __future__ import division
import logging
from specifier import *
from utils import *

LOG = logging.getLogger(__name__)

class Joint(object):
	'''Base class for joints'''
	def __init__(self, Name):
		self.Name = Name
		self.f = []

	#constraints enforced by joint forces
	def get_state_constraints(self):
		raise NotImplementedError("You must override this method")

	#constraints on the joint forces themselves
	def get_force_constraints(self):
		raise NotImplementedError("You must override this method")


class JointRevolute(Joint):
	'''Represents a powered revolute/spherical joint.'''

	def __init__(self, Name, BodyA, PointA, BodyB, PointB, RotationLimits, TorqueLimit):
		'''Constructor'''
		Joint.__init__(self, Name)
		self.BodyA = BodyA
		self.BodyB = BodyB
		self.PointA = PointA
		self.PointB = PointB
		self.RotationLimits = RotationLimits
		self.TorqueLimit = TorqueLimit

		self.ftx = sympy.Symbol(Name + "_ftx")  #translational
		self.fty = sympy.Symbol(Name + "_fty")
		self.ftz = sympy.Symbol(Name + "_ftz")

		self.frx = sympy.Symbol(Name + "_frx")  #rotational
		self.fry = sympy.Symbol(Name + "_fry")
		self.frz = sympy.Symbol(Name + "_frz")

		self.f = [self.ftx, self.fty, self.ftz,
				  self.frx, self.fry, self.frz] #in a list for convenience

		LOG.debug('new ' + str(self))
		
	def __str__(self):
		return 'JointRevolute "' + self.Name + '": connects ' + str(self.BodyA.Name) + \
		' at ' + str(self.PointA) + ' to ' + str(self.BodyB.Name) + ' at ' + str(self.PointB)

	def get_state_constraints(self):
		#constraints enforced by joint forces
		retList = []
		worldpointA = sym_world_xf(self.PointA, [bq(t) for bq in self.BodyA.q])
		worldpointB = sym_world_xf(self.PointB, [bq(t) for bq in self.BodyB.q])

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
		expr = self.get_sumsqr_torque_expr(t)
		if expr != 0:
			retList.append(Constraint(self.Name + "_f", lb=expr, c=self.TorqueLimit ** 2))
		return retList

	def get_angle_expr(self, time):
		'''Returns a list of expressions for the joint angle on each euler axis at the given time'''
		retList = []
		for i in range(3):
			retList.append(self.BodyA.q[i + 3](time) - self.BodyB.q[i + 3](time))
		return retList

	def get_angle_velocity_expr(self, time):
		'''Returns a list of expressions for the joint angular velocity on each euler axis at the given time'''
		retList = []
		for i in range(3):
			retList.append((self.get_angle_expr(time+1)[i] - self.get_angle_expr(time-1)[i])/(2*pH))
		return retList

	def get_sumsqr_angle_velocity_expr(self, time):
		'''Returns an expression for the sum of the squared joint angular velocities on each euler axis at the given time'''
		expr = 0
		vel = self.get_angle_velocity_expr(time)
		for i in range(3):
			expr += vel[i]**2
		return expr

	def get_angle_acceleration_expr(self, time):
		'''Returns a list of expressions for the joint angular acceleration on each euler axis at the given time'''
		retList = []
		for i in range(3):
			retList.append((self.get_angle_expr(time+1)[i] - 2*self.get_angle_expr(time)[i] + self.get_angle_expr(time-1)[i])/(pH*pH))
		return retList

	def get_sumsqr_angle_acceleration_expr(self, time):
		'''Returns an expression for the sum of the squared joint angular accelerations on each euler axis at the given time'''
		expr = 0
		accel = self.get_angle_acceleration_expr(time)
		for i in range(3):
			expr += accel[i]**2
		return expr

	def get_sumsqr_torque_expr(self, time):
		'''Returns an expression for the sum of the squared "muscle" torque applied at the joint'''
		expr = 0
		if self.RotationLimits is None:
			for i in range(3):
				expr += self.f[i + 3](time) ** 2
		else:
			for i in range(3):
				#we assume that on axes that can't rotate, no muscle force is required to enforce it
				if self.RotationLimits[i][0] != self.RotationLimits[i][1]:
					expr += self.f[i + 3](time) ** 2
		return expr

class JointContact(Joint):
	'''Represents a (possibly temporary) contact joint.'''

	def __init__(self, Name, Body, Point, Friction):
		'''Constructor'''
		Joint.__init__(self, Name)
		self.Body = Body
		self.Point = Point
		self.Friction = Friction

		self.ftx = sympy.Symbol(Name + "_ftx") #translational
		self.fty = sympy.Symbol(Name + "_fty")
		self.ftz = sympy.Symbol(Name + "_ftz")

		self.fry = sympy.Symbol(Name + "_fry") #rotational

		self.f = [self.ftx, self.fty, self.ftz, self.fry] #in a list for convenience

		LOG.debug('new ' + str(self))
	
	def __str__(self):
		return 'JointContact "' + self.Name + '": connects ' + str(self.Body.Name) + ' at ' + str(self.Point) + ' to ground plane'

	def get_world_position_expr(self):
		worldpoint = sym_world_xf(self.Point, [bq(t) for bq in self.Body.q])
		return worldpoint;

	def get_state_constraints(self):
		#constraints enforced by joint forces
		retList = []
		tRangeOn = 't in sTimeSteps_' + self.Name + 'On'

		#'zero velocity' constraints
		worldpoint_t0 = sym_world_xf(self.Point, [bq(t) for bq in self.Body.q])
		worldpoint_t1 = sym_world_xf(self.Point, [bq(t-1) for bq in self.Body.q])
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
		#(fx**2 + fz**2) < (fy*u)**2
		retList.append(Constraint(self.Name + '_force_friction', c=self.f[0](t) ** 2 + self.f[2](t) ** 2, ub=(self.f[1](t) * self.Friction) ** 2, TimeRange=tRangeOn))

		#contact force may only push (not pull)
		retList.append(Constraint(self.Name + '_force_push', lb=0, c=self.f[1](t), TimeRange=tRangeOn))

		#when joint is off, no force
		for force in self.f:
			retList.append(ConstraintEq(force.name + '_Off', a=force(t), TimeRange=tRangeOff))

		return retList