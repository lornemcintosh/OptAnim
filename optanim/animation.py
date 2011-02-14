from __future__ import division

from constraint import *
from joints import *
from utils import *

class Animation(object):
    '''Represents a character animation. The same animation can generally be
    used for multiple characters - even if their topologies differ'''

    def __init__(self, Name, Timesteps, TimestepLength):
	'''Constructor'''
	self.Name = Name
	self.Timesteps = int(Timesteps)
	self.TimestepLength = float(TimestepLength)
	self.constraintList = []
	self.objectiveList = []
	self.pluginList = []

    def set_joint_timings(self, dict):
	self.jointTimeDict = dict

    def add_constraint(self, c):
	self.constraintList.append(c)

    def add_objective(self, obj, weight):
	self.objectiveList.append([obj, weight])

    def add_constraint_plugin(self, plugin):
	self.pluginList.append(plugin)

    def write(self, character, outdir, solver):
	file = openfile(os.path.join(outdir, character.name + '_' + self.Name + '.ampl'), 'w')
	file.write('param pAnimName symbolic := "%s";\n' % self.Name);
	file.write('param pi := atan(1.0)*4.0;\n')
	file.write('param pH = %f;\n' % self.TimestepLength)
	file.write('param pTimeBegin = 0;\n')
	file.write('param pTimeEnd = %i;\n' % (self.Timesteps-1))
	file.write('set sTimeSteps := pTimeBegin .. pTimeEnd;\n')
	file.write('\n')
	
	#write joint timing sets
	for j in character.jointList:
	    if isinstance(j, JointContact):  #TODO: refactor joints so we can test isinstance(j,ToggleableJoint) or something
		try:
		    set = self.jointTimeDict[j]
		    setstr = str(set).replace('[', '{')
		    setstr = setstr.replace(']', '}');
		    file.write('set sTimeSteps_%sOn := %s;\n' % (j.Name, setstr))
		    file.write('set sTimeSteps_%sOff := sTimeSteps diff sTimeSteps_%sOn;\n' % (j.Name, j.Name))
		    file.write('\n')
		except KeyError:
		    raise BaseException('Character "%s" has a temp joint "%s". You must specify timings for %s.' % (character.name, j.Name, j.Name))

	#add the character's physical constraints
	file.write('include ' + character.name + '.ampl;\n')
	file.write('\n')

	#write animation constraints
	for i, eq in enumerate(self.constraintList):
	    file.write(str(eq))

	#write plugin constraints
	for i, plugin in enumerate(self.pluginList):
	    for c in plugin.get_constraints(character):
		file.write(str(c))

	#write weighted objectives
	if len(self.objectiveList) > 0:
	    file.write('minimize objective:\n')
	    for i, obj in enumerate(self.objectiveList):
		file.write('\t' + str(obj[1]) + ' * ' + obj[0])
		if(i == len(self.objectiveList)-1):
		    file.write(';\n')
		else:
		    file.write(' +\n')

	file.write('\n')
	file.write('option reset_initial_guesses 1;\n')
	file.write('option show_stats 1;\n')
	file.write('option solver ' + solver + ';\n')
	file.write('solve;\n');
	file.write('\n')

	file.write('include ' + character.name + '-exporter.ampl;\n')
	file.write('\n')

	#file.write('display {j in 1.._nvars} (_varname[j],_var[j],_var[j].ub,_var[j].rc);\n')
	#file.write('display {j in 1.._nvars: _var[j].status = "pre"} _varname[j];\n');

	file.close()