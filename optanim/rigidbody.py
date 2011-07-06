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

	#mass vector (diagonals of a mass matrix) for a solid ellipsoid
	a,b,c = [x/2.0 for x in Diameter] #unpack as radii
	self.Mass = [Mass, Mass, Mass,	#translational mass
	    (Mass / 5.0) * (b**2+c**2),	#rotational mass moments of inertia
	    (Mass / 5.0) * (a**2+c**2),
	    (Mass / 5.0) * (a**2+b**2)]

	self.Diameter = Diameter
	self.q = [
	    sympy.Symbol(Name + "_qtx"), #translational
	    sympy.Symbol(Name + "_qty"),
	    sympy.Symbol(Name + "_qtz"),

	    sympy.Symbol(Name + "_qrx"), #rotational
	    sympy.Symbol(Name + "_qry"),
	    sympy.Symbol(Name + "_qrz")]

	LOG.debug('new ' + str(self))

    def __str__(self):
	return 'RigidBody "' + self.Name + '": diameter = ' + str(self.Diameter) + ', mass = ' + str(self.Mass)

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