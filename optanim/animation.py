from __future__ import division
import itertools
import numpy
import re
import subprocess
import time
import cma
from exporters import *
from constraint import *
from constraintplugins import *
from joints import *
from utils import *

class AnimationSpec(object):
    '''Specifies a character animation (or set of parametric animations) to be
    made. The same animation can often be used for multiple characters - even if
    their morphologies differ'''

    def __init__(self, Name, Length=None, FPS=25):
	'''Constructor'''
	self.Name = Name
	self.Length = Length
	self.FPS = FPS
	self.ContactTimesDict = None

	self.ConstraintList = []
	self.ParamConstraintList = []

	self.ObjectiveList = []
	self.ParamObjectiveList = []

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
	self.ObjectiveList.append((obj, weight))

    def add_param_objective(self, obj):
	'''Add a set of parameterized objectives -- an animation will be
	produced for every unique combination of objectives from these sets'''
	self.ParamObjectiveList.append(obj)

    def generate(self, outdir, solver):
	print "Generating %s..." % self.Name

	#print a helpful message about the number of combinations
	combinations = len(self.CharacterList)
	for c in self.ParamConstraintList:
	    combinations *= max(len(c), 1)
	for o in self.ParamObjectiveList:
	    combinations *= max(len(o), 1)
	print "There will be %i combinations" % combinations

	#for each combination of character and parameters
	for character in self.CharacterList:
	    for cindex, ccomb in enumerate(itertools.product(*self.ParamConstraintList)):
		for oindex, ocomb in enumerate(itertools.product(*self.ParamObjectiveList)):
		    #create an animation instance
		    animName =  character.Name + "_" + self.Name + "_" + str(cindex) + "_" + str(oindex)
		    anim = Animation(animName, self.Length, self.FPS, character,
			self.ConstraintList + list(itertools.chain.from_iterable(ccomb)), self.ObjectiveList + list(ocomb), self.ContactTimesDict);
		    start_time = float(time.time())
		    anim.optimize(solver)
		    elapsed_time = time.time() - start_time
		    if(anim.Solved):
			print('Solved! (took %f seconds, Objective = %f)' % (elapsed_time, anim.ObjectiveValue))

			filename = animName + '.bvh'
			print('Writing: %s,' % filename),
			file = open(filename, 'w')
			file.write(export_bvh(anim))
			file.close()

			filename = animName + '.flat.bvh'
			print('%s,' % filename),
			file = open(filename, 'w')
			file.write(export_bvh_flat(anim))
			file.close()

			filename = animName + '.skeleton.xml'
			print('%s' % filename)
			file = open(filename, 'w')
			file.write(export_ogre_skeleton_xml(anim))
			file.close()
		    else:
			print('Failed to solve! (took %f seconds)' % elapsed_time)
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

    '''def export_bvh(self):
	if not self.Solved:
	    raise BaseException("Animation must be solved before export")
	bvh = export_bvh(self)
	return bvh'''

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
		    #TODO: goofy 'double rounding' here is to avoid small floating-point errors; use decimal package instead?
		    intervalFrames = int(round(round(intervalTime * self.FPS, 1)))
		    startFrame = int(round(round(startTime * self.FPS, 1)))
		    endFrame = startFrame + intervalFrames
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
	    ret += 'minimize objective: (\n'
	    for i, obj in enumerate(self.ObjectiveList):
		ret += '\t' + str(obj[1]) + ' * (' + str(obj[0]) + ')'
		if(i == len(self.ObjectiveList)-1):
		    ret += ') / (pTimeEnd+1);\n'
		else:
		    ret += ' +\n'
	return ret

    def _write_footer(self, solver):
	ret = ''
	ret = 'option reset_initial_guesses 1;\n'
	#ret += 'option show_stats 1;\n'
	ret += 'option solver ' + solver + ';\n'
	ret += 'option ipopt_options \'max_iter=4000 print_level=0\';\n' #TODO: what about other solvers?
	ret += 'solve;\n'
	ret += '\n'

	ret += 'display solve_result;\n'
	ret += 'display objective;\n'

	#for interest we can output the values of individual objectives in the solution
	for i, obj in enumerate(self.ObjectiveList):
	    ret += 'printf "Objective%i: %f * %f = %f\\n",'+str(i)+','+str(obj[1])+','+str(obj[0])+','+str(obj[1])+' * '+str(obj[0])+';\n'

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

	amplresult = ''
	try:
	    #try to load cached solution
	    file = open(self.Name + '.amplsol', 'r')
	    amplresult = file.read();
	    file.close()
	except:
	    #couldn't load cached solution file, we'll have to solve it
	    #solve it with ampl
	    ampl = subprocess.Popen("ampl", stdin=subprocess.PIPE, stdout=subprocess.PIPE)
	    amplresult = ampl.communicate(amplcmd)[0] #blocking call

	    #cache solution to a file
	    file = open(self.Name + '.amplsol', 'w')
	    file.write(amplresult)
	    file.close()

	#did it solve correctly?
	self.Solved = (" = solved" in amplresult)
	if self.Solved:
	    objectivematch = re.search("(?<=objective = )"+regex_float, amplresult)
	    self.ObjectiveValue = float(objectivematch.group(0))

	    #read the solution variables into a dict
	    #this assumes AMPL will output them in order of ascending indices
	    #myvar[0]... myvar[1]... myvar[2] etc.
	    pattern = "\d+\s+'(\w+)\[(\d+)]'\s+"+regex_float
	    matches = re.findall(pattern, amplresult)
	    for match in matches:
		if match[0] not in self.SolutionValues:
		    self.SolutionValues[match[0]] = [float(match[2])]
		else:
		    self.SolutionValues[match[0]].append(float(match[2]))
	#else:
	    #print(amplresult)
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
		f = 1.0/len(self.Character.get_joints_contact())
		for j,joint in enumerate(self.Character.get_joints_contact()):
		    evenly = (j*f)+(f/2.0) #space the contacts evenly
		    startPoint.extend([evenly, 0.5])
		    lowerBounds.extend([0.0, 0.0])
		    upperBounds.extend([1.0, 1.0])

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
		    print '.',
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