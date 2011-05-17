from __future__ import division

from utils import *

class Objective(object):
    '''Represents an objective expression, with an associated weighting'''
    def __init__(self, Name, Objective=None, Weight=1.0, TimeRange=None):
	self.Name = Name
	self.Objective = Objective	#objective expr
	self.TimeRange = TimeRange  #e.g. 1 <= t <= 5
        self.Weight = Weight

    def __repr__(self):
        return AmplPrinter().doprint(self)

    def __str__(self):
        return AmplPrinter().doprint(self)

    def _sympystr(self, p):
        s=''
        if self.TimeRange is not None:
            s = ': '+self.TimeRange
        return '\t' + p.doprint(self.Weight) + ' * sum {t in sTimeSteps'+s+'} (' + p.doprint(self.Objective) + ')'