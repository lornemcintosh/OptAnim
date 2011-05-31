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

    def get_objective_str(self):
        s=''
        if self.TimeRange is not None:
            s = ': '+self.TimeRange
        return 'sum {t in sTimeSteps'+s+'} (' + ampl(self.Objective) + ')'

    def get_weightedobjective_str(self):
        return  str(self.Weight) + ' * ' + self.get_objective_str()

    def _sympystr(self, p):
        return '\t(' + self.get_weightedobjective_str() + ')'

    def write_debug_str(self):
        return 'printf "Objective_'+self.Name+': %f * %f = %f\\n",'+str(self.Weight)+','+str(self.get_objective_str())+','+self.get_weightedobjective_str()+';\n'