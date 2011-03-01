from __future__ import division

from utils import *

class Constraint(object):
    '''Represents a constraint expression with lower and upper bounds'''
    def __init__(self, Name, lb=None, c=None, ub=None, TimeRange=''):
	self.Name = Name
	self.lb = lb	#lower bound
	self.c = c	#constrained expr
	self.ub = ub	#upper bound
	self.TimeRange = TimeRange  #e.g. 1 <= t <= 5

    def __repr__(self):
        return AmplPrinter().doprint(self)

    def __str__(self):
        return AmplPrinter().doprint(self)

    def _sympystr(self, p):
	#declaration:
	if(self.TimeRange != ''):
	    decl = 'subject to ' + self.Name + ' {t in sTimeSteps: ' + self.TimeRange + '}:\n\t';
	else:
	    decl = 'subject to ' + self.Name + ' {t in sTimeSteps}:\n\t';

	#equality
	if(self.lb == self.ub):
	    return decl + p.doprint(self.c) + ' == ' + p.doprint(self.lb) + ';\n'

	#inequality
	elif(self.lb is not None and self.c and self.ub is not None):
	    return decl + p.doprint(self.lb) + ' <= ' + p.doprint(self.c) + ' <= ' + p.doprint(self.ub) + ';\n'
	elif(self.lb is not None and self.c is not None):
	    return decl + p.doprint(self.lb) + ' <= ' + p.doprint(self.c) + ';\n'
	elif(self.c is not None and self.ub is not None):
	    return decl + p.doprint(self.c) + ' <= ' + p.doprint(self.ub) + ';\n'
	else:
	    raise "not a proper constraint!"

class ConstraintEq(Constraint):
    '''Convenience class for creating an equality
    constraint (lower and upper bounds are equal)'''
    def __init__(self, Name, a, b=0, TimeRange=''):
	Constraint.__init__(self, Name, lb=b, c=a, ub=b, TimeRange=TimeRange)