from __future__ import division
import itertools

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
	self.paramConstraintList = []
	self.objectiveList = []
	self.pluginList = []
	self.characterList = []

    def add_character(self, character):
	self.characterList.append(character)

    def set_joint_timings(self, dict):
	self.jointTimeDict = dict

    def add_constraint(self, c):
	self.constraintList.append(c)

    def add_param_constraint(self, c):
	self.paramConstraintList.append(c)

    def add_objective(self, obj, weight):
	self.objectiveList.append([obj, weight])

    def add_constraint_plugin(self, plugin):
	self.pluginList.append(plugin)

    def write_header(self, character, file, solver):
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

    def write_constraints(self, character, file, solver):
	for i, eq in enumerate(self.constraintList):
	    file.write(str(eq))

	#write plugin constraints
	for i, plugin in enumerate(self.pluginList):
	    for c in plugin.get_constraints(character):
		file.write(str(c))

    def write_objective(self, character, file, solver):
	#write weighted objectives
	if len(self.objectiveList) > 0:
	    file.write('minimize objective:\n')
	    for i, obj in enumerate(self.objectiveList):
		file.write('\t' + str(obj[1]) + ' * ' + obj[0])
		if(i == len(self.objectiveList)-1):
		    file.write(';\n')
		else:
		    file.write(' +\n')

    def write_footer(self, character, file, solver):
	file.write('option reset_initial_guesses 1;\n')
	file.write('option show_stats 1;\n')
	file.write('option solver ' + solver + ';\n')
	file.write('solve;\n');
	file.write('\n')
	file.write('display solve_result;\n')
	file.write('display objective;\n')
	#file.write('display {j in 1.._nvars} (_varname[j],_var[j],_var[j].ub,_var[j].rc);\n')
	#file.write('display {j in 1.._nvars: _var[j].status = "pre"} _varname[j];\n');
	#file.write('include ' + character.name + '-exporter.ampl;\n')
	#file.write('\n')

    def generate(self, outdir, solver):
	print "Generating %s..." % self.Name

	#print a helpful message about the number of combinations
	combinations = len(self.characterList)
	for i, c in enumerate(self.paramConstraintList):
	    combinations *= len(c)
	print "There will be %i combinations" % combinations

	#for each combination of character and parameters
	for character in self.characterList:
	    for index, combination in enumerate(itertools.product(*self.paramConstraintList)):
		file = openfile(os.path.join(outdir, self.Name + '_' + character.name + '_' + str(index) + '.ampl'), 'w')
		self.write_header(character, file, solver)

		#include the character's physical constraints
		character.write_model(file)

		#write the parameterized constraints
		map(file.write, map(str,combination))

		#write the 'static' non-parameterized constraints
		self.write_constraints(character, file, solver)

		self.write_objective(character, file, solver)
		self.write_footer(character, file, solver)

		file.close()
		print '.',