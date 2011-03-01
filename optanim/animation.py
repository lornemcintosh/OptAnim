from __future__ import division
import itertools
import numpy
import re
import subprocess

import cma
from constraint import *
from constraintplugins import *
from joints import *
from utils import *

class AnimationSpec(object):
    '''Specifies a character animation (or set of parametric animations) to be
    made. The same animation can often be used for multiple characters - even if
    their morphologies differ'''

    def __init__(self, Name, Length=None, FPS=20):
	'''Constructor'''
	self.Name = Name
	self.Length = Length
	self.FPS = FPS
	self.ContactTimesDict = None
	self.ConstraintList = []
	self.ParamConstraintList = []
	self.ObjectiveList = []
	self.CharacterList = []

    def set_length(self, length):
	self.Length = float(length)

    def set_contact_times(self, dict):
	self.ContactTimesDict = dict
    
    def add_character(self, character):
	self.CharacterList.append(character)

    def add_constraint(self, c):
	'''Add a static constraint -- one that will apply to ALL animations
	in this animation set'''
	self.ConstraintList.append(c)

    def add_param_constraint(self, c):
	'''Add a set of parameterized constraints -- an animation will be
	produced for every unique combination of constraints from these sets'''
	self.ParamConstraintList.append(c)

    def add_objective(self, obj, weight):
	self.ObjectiveList.append([obj, weight])

    def generate(self, outdir, solver):
	print "Generating %s..." % self.Name

	#print a helpful message about the number of combinations
	combinations = len(self.CharacterList)
	for i, c in enumerate(self.ParamConstraintList):
	    combinations *= len(c)
	print "There will be %i combinations" % combinations

	#for each combination of character and parameters
	for character in self.CharacterList:
	    for index, combination in enumerate(itertools.product(*self.ParamConstraintList)):

		#create an animation instance
		animName = self.Name + "_" + character.Name + "_" + str(index)
		anim = Animation(animName, self.Length, self.FPS, character,
				 self.ConstraintList + list(combination), self.ObjectiveList, self.ContactTimesDict);
		anim.optimize(solver)
		if(anim.Solved):
		    print('Solved! Objective = ' + str(anim.ObjectiveValue))
		    print('Writing ' + animName + '.bvh')
		    file = open(animName + '.bvh', 'w')
		    file.write(anim.export_bvh())
		    file.close()

class Animation(object):
    '''Represents a specific character animation. The character and constraints
    etc. are set in stone. If solved, it also stores the optimization results (the
    animation data), and can write it to a BVH file'''

    def __init__(self, Name, Length, FPS, Character, Constraints, Objectives, ContactTimes):
	'''Constructor'''
	self.Name = Name
	self.Length = Length
	self.FPS = FPS
	self.Character = Character
	self.ConstraintList = Constraints
	self.ObjectiveList = Objectives
	self.ContactTimesDict = ContactTimes

	self.Solved = False
	self.ObjectiveValue = numpy.NaN
	self.SolutionValues = {}

    def get_frame_length(self):
	return float(1.0 / self.FPS)

    def get_frame_count(self):
	return int(round(self.Length * self.FPS))

    def export_bvh(self):
	if not self.Solved:
	    raise BaseException("Animation must be solved before export")
	#pass animation data to character, and let it write the bvh format
	bvh = self.Character.export_bvh(self.SolutionValues, self.get_frame_count(), self.get_frame_length())
	return bvh

    def _write_header(self):
	ret = ''
	ret += 'param pH = %f;\n' % self.get_frame_length()
	ret += 'param pTimeBegin = 0;\n'
	ret += 'param pTimeEnd = %i;\n' % (self.get_frame_count()-1)
	ret += 'set sTimeSteps := pTimeBegin .. pTimeEnd;\n'

	#write joint timing sets
	for j in self.Character.get_joints_contact():
	    try:
		footsteps = self.ContactTimesDict[j]
		contactSet = set()
		for step in footsteps:
		    startTime, intervalTime = [x * self.Length for x in step] #convert from fraction of length to real seconds
		    endTime = startTime + intervalTime
		    startFrame = int(round(startTime * self.FPS))
		    endFrame = int(round(endTime * self.FPS))
		    contactSet = contactSet | set([x % self.get_frame_count() for x in range(startFrame, endFrame)])	#loop

		contactStr = '{' + (', '.join(map(str, contactSet))) + '}'
		ret += 'set sTimeSteps_%sOn := %s;\n' % (j.Name, contactStr)
		ret += 'set sTimeSteps_%sOff := sTimeSteps diff sTimeSteps_%sOn;\n' % (j.Name, j.Name)
		ret += '\n'
	    except KeyError:
		raise BaseException('Character "%s" has a temp joint "%s". You must specify timings for %s.' % (self.Character.name, j.Name, j.Name))
	ret += '\n'
	return ret

    def _write_constraints(self):
	ret = ''
	for eq in self.ConstraintList:
	    #regular constraints
	    if isinstance(eq, Constraint):
		ret += str(eq)
	    #constraint plugins
	    if isinstance(eq, ConstraintPlugin):
		for c in eq.get_constraints(self, self.Character):
		    ret += str(c)
	return ret

    def _write_objective(self):
	ret = ''
	#write weighted objectives
	if len(self.ObjectiveList) > 0:
	    ret += 'minimize objective:\n'
	    for i, obj in enumerate(self.ObjectiveList):
		ret += '\t' + str(obj[1]) + ' * (' + obj[0] + ')'
		if(i == len(self.ObjectiveList)-1):
		    ret += ';\n'
		else:
		    ret += ' +\n'
	return ret

    def _write_footer(self, solver):
	ret = ''
	ret = 'option reset_initial_guesses 1;\n'
	#ret += 'option show_stats 1;\n'
	ret += 'option solver ' + solver + ';\n'
	ret += 'option ipopt_options \'print_level=0\';\n' #TODO: what about other solvers?
	ret += 'solve;\n'
	ret += '\n'

	ret += 'display solve_result;\n'
	ret += 'display objective;\n'
	ret += 'if solve_result = "solved" then{ display {j in 1.._nvars} (_varname[j],_var[j]); }\n'
	ret += 'exit;\n'
	return ret

    def _solve(self, solver, writeAMPL=False):
	'''This handles the 'inner' (spacetime) optimization. It assumes that
	length and contact timings are set. Use optimize() instead.'''

	#reset the solution
	self.Solved = False
	self.ObjectiveValue = numpy.NaN
	self.SolutionValues = {}

	amplcmd = ''
	amplcmd += self._write_header()
	amplcmd += self.Character.get_model()
	amplcmd += self._write_constraints()
	amplcmd += self._write_objective()
	amplcmd += self._write_footer(solver)

	#for debugging we'll write out the ampl file
	if writeAMPL:
	    file = open(self.Name + '.ampl', 'w')
	    file.write(amplcmd)
	    file.close()

	#solve it with ampl
	ampl = subprocess.Popen("ampl", stdin=subprocess.PIPE, stdout=subprocess.PIPE)
	amplresult = ampl.communicate(amplcmd) #blocking call

	#did it solve correctly?
	self.Solved = (" = solved" in amplresult[0])
	if self.Solved:
	    objectivematch = re.search("(?<=objective = )[0-9]*", amplresult[0])
	    self.ObjectiveValue = float(objectivematch.group(0))

	    #read the solution variables into a dict
	    #this assumes AMPL will output them in order of ascending indices
	    #myvar[0]... myvar[1]... myvar[2] etc.
	    pattern = "\d+\s+'(\w+)\[(\d+)]'\s+([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)"
	    matches = re.findall(pattern, amplresult[0])
	    for match in matches:
		if match[0] not in self.SolutionValues:
		    self.SolutionValues[match[0]] = [float(match[2])]
		else:
		    self.SolutionValues[match[0]].append(float(match[2]))
	#else:
	    #print(amplresult[0])
	return self.ObjectiveValue


    def optimize(self, solver):
	'''This handles the 'outer' optimization that's necessary to determine
	animation length and contact timings (if they are not explicitly provided).'''

	optLength = self.Length is None
	optContacts = self.ContactTimesDict is None

	if optLength or optContacts:
	    print("Starting CMA-ES optimization for %s..." % self.Name)

	    startPoint = []
	    lowerBounds = []
	    upperBounds = []
	    if optLength:
		startPoint.append(0.5)
		lowerBounds.append(self.get_frame_length()*3.0) #3 frame minimum
		upperBounds.append(1.0)
	    if optContacts:
		n = len(self.Character.get_joints_contact()) * 2
		startPoint.extend([0.5] * n)
		lowerBounds.extend([0.0] * n)
		upperBounds.extend([1.0] * n)

	    #optimize anim length and contact timings with CMA-ES
	    es = cma.CMAEvolutionStrategy(startPoint, 1.0 / 3.0,
		{'maxiter':100, 'bounds':[lowerBounds, upperBounds]})

	    # iterate until termination
	    while not es.stop:
		X = []
		fit = []
		for i in range(es.popsize):
		    curr_fit = numpy.NaN
		    while curr_fit is numpy.NaN:
			x = es.ask(1)[0]
			#print(x)
			if optLength:
			    self.Length = x[0] * 3 #TODO: handle scaling better
			if optContacts:
			    m = 1 if optLength else 0
			    self.ContactTimesDict = {}
			    for j, joint in enumerate(self.Character.get_joints_contact()):
				self.ContactTimesDict[joint] = [(x[j*2+0+m], x[j*2+1+m])]
			curr_fit = self._solve(solver)  #might return numpy.NaN
		    fit.append(curr_fit)
		    X.append(x)
		    print '. ',
		es.tell(X, fit)
		print ''
		es.printline(1)
	    print 'termination: ', es.stopdict
	    print(es.best[0])

	    #TODO: Because we don't bother saving the animation data, we have to
	    #solve the best one (again) to get it. This code is just a re-run
	    #from above, except it solves the best one found
	    if optLength:
		self.Length = es.best[0][0] * 3 #TODO: handle scaling better
	    if optContacts:
		m = 1 if optLength else 0
		self.ContactTimesDict = {}
		for j, joint in enumerate(self.Character.get_joints_contact()):
		    self.ContactTimesDict[joint] = [(es.best[0][j*2+0+m], es.best[0][j*2+1+m])]
	    return self._solve(solver, writeAMPL=True)

	else:
	    print("CMA-ES optimization unnecessary for %s. Solving..." % self.Name)
	    return self._solve(solver, writeAMPL=True)