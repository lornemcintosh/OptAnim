from __future__ import division
import sympy

from specifier import *
from utils import *

import logging
LOG = logging.getLogger(__name__)

class RigidBody(object):
    '''Represents a rigid body.'''

    def __init__(self, Id, Name, Mass, Diameter):
	'''Constructor'''

	self.Id = Id	    #just used for exporting to ogre format currently
	self.Name = Name
        self.Mass = Mass    #scalar mass
	self.Diameter = Diameter
	self.q = [
	    sympy.Symbol(Name + "_qtx"), #translational
	    sympy.Symbol(Name + "_qty"),
	    sympy.Symbol(Name + "_qtz"),

	    sympy.Symbol(Name + "_qrx"), #rotational
	    sympy.Symbol(Name + "_qry"),
	    sympy.Symbol(Name + "_qrz")]

	LOG.debug('new ' + str(self))

    def get_mass_vector(self):
        '''Returns mass vector (diagonals of a mass matrix) for this rigid body'''
        #unpack diameters as radii
        a,b,c = [x/2.0 for x in self.Diameter]
        
        return [self.Mass, self.Mass, self.Mass,    #translational mass
	    (self.Mass / 5.0) * (b**2+c**2),        #rotational mass moments of inertia (for a solid ellipsoid)
	    (self.Mass / 5.0) * (a**2+c**2),
	    (self.Mass / 5.0) * (a**2+b**2)]

    def __str__(self):
	return 'RigidBody "' + self.Name + '": diameter = ' + str(self.Diameter) + ', mass = ' + str(self.get_mass_vector())

    def ep_a(self):
	'''returns the position of endpoint A in body local coordinates'''
	return [0.0, self.Diameter[1] / 2.0, 0.0]

    def ep_b(self):
	'''returns the position of endpoint B in body local coordinates'''
	return [0.0, -self.Diameter[1] / 2.0, 0.0]

    def get_intersection_constraint(self, spherePoint, sphereDiameter=0.0):
	'''returns a constraint that ensures the given sphere (world coords) will
	not intersect/penetrate/collide with this body (ellipsoid) on any frame'''
	#unpack diameters as radii for convienience
	a,b,c = [x/2.0 for x in self.Diameter]
	r = sphereDiameter/2.0
	#transform point into body-local coordinates
	x,y,z = sym_world_xf(spherePoint, [bq(t) for bq in self.q], worldToLocal=True)
	return Constraint('NoIntersection_' + self.Name, 1, (x**2/(a+r)**2) + (y**2/(b+r)**2) + (z**2/(c+r)**2))

    def get_acceleration_expr(self, time):
        '''Returns a list of expressions for the acceleration of the q coordinates'''
        retList = []
        for i in range(dof):
            retList.append((self.q[i](time+1) - 2*self.q[i](time) + self.q[i](time-1))/(pH*pH))
        return retList