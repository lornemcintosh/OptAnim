from __future__ import division
import sympy

from utils import *

class RigidBody(object):
    '''Represents a rigid body.'''

    def __init__(self, Name, Mass, Length):
	'''Constructor'''
	self.Name = Name
	self.Mass = [Mass, Mass, 1.0 / 12.0 * Mass * Length ** 2.0]
	self.Length = Length
	self.q = [sympy.Symbol(Name + "_qx"), sympy.Symbol(Name + "_qy"), sympy.Symbol(Name + "_qr")]
	self.ChildList = []
	self.Parent = None
	print 'new rigid body "' + Name + '": length = ' + str(Length) + ', mass = ' + str(Mass)

    def add_child(self, body):
	self.ChildList.append(body)

    def set_parent(self, body):
	if(self.Parent is not None):
	    raise BaseException("already has parent assigned!")
	self.Parent = body

    def ep_a(self):
	'''returns the position of endpoint A in body local coordinates'''
	return [0.0, self.Length / 2.0]

    def ep_b(self):
	'''returns the position of endpoint B in body local coordinates'''
	return [0.0, -self.Length / 2.0]