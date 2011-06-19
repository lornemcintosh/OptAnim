from __future__ import division

import math
import copy
import itertools
import numpy
import re
import time

import cma
from exporters import *
from joints import *
from specifier import *
from threadpool import *
from utils import *

pool = ThreadPool()

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

	self.SpecifierList = []
	self.ParamSpecifierList = []

	self.CharacterList = []

        self.AnimationList = []

    def set_length(self, length):
	self.Length = float(length)

    def set_contact_times(self, dict):
	self.ContactTimesDict = dict

    def get_frame_length(self):
	return float(1.0 / self.FPS)

    def get_frame_count(self):
	return int(round(self.Length * self.FPS))
    
    def add_character(self, character):
	self.CharacterList.append(character)

    def add_specifier(self, spec):
	'''Add a static specifier -- one that will apply to ALL animations
	in this AnimationSpec'''
	self.SpecifierList.append(spec)

    def add_param_specifier(self, spec):
	'''Add a set of parameterized specifiers -- an animation will be
	produced for every unique combination of specifiers from these sets'''
	self.ParamSpecifierList.append(spec)

    def generate(self, solver='ipopt'):
	print "Generating %s..." % self.Name

	#print a helpful message about the number of combinations generated
	combinations = len(self.CharacterList)
	for spec in self.ParamSpecifierList:
	    combinations *= max(len(spec), 1)
	print "There will be %i combinations" % combinations

	#make an anim for each combination of characters/specifiers
	for character in self.CharacterList:
	    for index, comb in enumerate(itertools.product(*self.ParamSpecifierList)):
                #build out constraint and objective lists
                paramList = list(itertools.chain.from_iterable(comb))
                animSpecifierList = character.SpecifierList + self.SpecifierList + paramList

                #create an animation instance
                animName = character.Name + "_" + self.Name + "_" + str(index)
                anim = Animation(animName, self.Length, self.FPS, character,
                                 animSpecifierList, self.ContactTimesDict);
                self.AnimationList.append(anim)
                anim.optimize(solver)  #non-blocking

    def wait_for_results(self):
        '''Polls the animations and returns when they're all done'''
        alldone = False
        while(alldone is False):
            alldone = True
            for anim in self.AnimationList:
                if anim.Done is False:
                    alldone = False
                    time.sleep(1)
                    break

    def export(self, outdir):
        self.wait_for_results()
        for anim in self.AnimationList:
            if anim.Solved:
                anim.export(outdir)

class Animation(object):
    '''Represents a specific character animation. The character and constraints
    etc. are set in stone. If solved, it also stores the optimization results (the
    animation data)'''

    def __init__(self, Name, Length, FPS, Character, SpecifierList, ContactTimes):
	'''Constructor'''
	self.Name = Name
	self.Length = Length
	self.FPS = FPS
	self.Character = Character
	self.SpecifierList = SpecifierList
	self.ContactTimesDict = ContactTimes

	self.Done = False
        self.Solved = False
	self.ObjectiveValue = numpy.NaN
	self.AnimationData = {}
        self.CachedConstraintList = []
        self.CachedObjectiveList = []

    def __str__(self):
        return self.Name

    def get_frame_length(self):
	return float(1.0 / self.FPS)

    def get_frame_count(self):
	return int(round(self.Length * self.FPS))

    def get_contact_frames(self, joint):
        try:
            footsteps = self.ContactTimesDict[joint]
            contactSet = set()
            for step in footsteps:
                startTime, intervalTime = [x * self.Length for x in step] #convert from fraction of length to real seconds
                #TODO: goofy 'double rounding' here is to avoid small floating-point errors; use decimal package instead?
                intervalFrames = int(round(round(intervalTime * self.FPS, 1)))
                startFrame = int(round(round(startTime * self.FPS, 1)))
                endFrame = startFrame + intervalFrames
                contactSet = contactSet | set([x % self.get_frame_count() for x in range(startFrame, endFrame)])	#loop
            return contactSet

        except KeyError:
            raise BaseException('Character "%s" has a contact joint "%s". You must specify timings for %s.' % (self.Character.Name, joint.Name, joint.Name))

    def get_frame_slice(self, firstFrame, lastFrame):
        '''Returns a new Animation containing just the frames between firstFrame and lastFrame'''
        #take a 1 extra frame on each side
        firstFrame -= 1
        firstFrame = max(0, firstFrame)
        lastFrame += 1
        lastFrame = min(self.get_frame_count(), lastFrame)

        newName = self.Name + "_" + str(firstFrame) + "to" + str(lastFrame)
        newLength = (lastFrame-firstFrame+1)*self.get_frame_length()
        ret = Animation(newName, newLength, self.FPS, self.Character, None, None)
        
        #setup animation data
        ret.AnimationData = {}
        for body in ret.Character.BodyList:
            ret.AnimationData[str(body.Name)] = [None] * ret.get_frame_count()

        #copy slice of animation data from original
        for k, v in ret.AnimationData.items():
            ret.AnimationData[k] = self.AnimationData[k][firstFrame:lastFrame + 1]

        ret.Done = True
        ret.Solved = True
        
        return ret

    def animdata_resample(self, fps):
        ret = copy.deepcopy(self)   #TODO: get rid of this deepcopy (too memory hungry)
        ret.FPS = fps
        frameCount = ret.get_frame_count()

        #clear existing animation data
        ret.AnimationData = {}
        for body in ret.Character.BodyList:
            ret.AnimationData[str(body.Name)] = [None] * frameCount

        #do the resampling
        for frame in range(frameCount):
            time = frame / (frameCount-1)
            interpData = self.animdata_get_interpolated(time)
            for body in ret.Character.BodyList:
                ret.AnimationData[str(body.Name)][frame] = interpData[body.Name][0]

        return ret

    def animdata_get_interpolated(self, time):
        '''Returns the interpolated state at time, where time is 0 to 1'''
        assert(0.0 <= time <= 1.0)

        nframes = (len(self.AnimationData.items()[0][1])-1)
        frameA = int(math.floor(time * nframes))
        frameB = int(math.ceil(time * nframes))

        ret = copy.deepcopy(self.AnimationData)
        if frameA == frameB:
            for k, v in ret.items():
                ret[k] = ret[k][frameA:frameB + 1]
            #print str(time)+", "+str(frameA)+" == "+str(frameB)+", "+str(ret["C_torso"])
            return ret
        else:
            timeA = frameA / nframes
            timeB = frameB / nframes
            timeAB = (time - timeA) / (timeB - timeA)
            for k, v in ret.items():
                dataA = ret[k][frameA]
                dataB = ret[k][frameB]
                ret[k] = [num_q_lerp(dataA, dataB, timeAB)]
            #print str(time)+", "+str(frameA)+" != "+str(frameB)+", timeAB="+str(timeAB)+", "+str(ret["C_torso"])
            return ret

    def blend(self, other, weight, fps=25):
        print "Blending " + str(self.Name) + " and " + str(other.Name) + ". Weight = " + str(weight)

        #calculate length (in seconds) of new animation clip:
        #(formula from Safonova & Hodgins / Analyzing the Physical Correctness of Interpolated Human Motion)
        length = math.sqrt(math.pow(self.Length,2)*weight + math.pow(other.Length,2)*(1-weight))
        
        ret = Animation(str(self.Name) + "_and_" + str(other.Name), length, fps, self.Character, None, None)

        frameCount = ret.get_frame_count()

        for body in ret.Character.BodyList:
            ret.AnimationData[str(body.Name)] = [None] * frameCount

        for frame in range(frameCount):
            frameTime = frame / (frameCount-1)
            a = self.animdata_get_interpolated(frameTime)
            b = other.animdata_get_interpolated(frameTime)
            for body in ret.Character.BodyList:
                ret.AnimationData[str(body.Name)][frame] = num_q_lerp(a[str(body.Name)][0], b[str(body.Name)][0], weight)
        
        ret.Done = True
        ret.Solved = True

        return ret

    def _write_header(self):
	ret = ''
	ret += 'param pH = %f;\n' % self.get_frame_length()
	ret += 'param pTimeBegin = 0;\n'
	ret += 'param pTimeEnd = %i;\n' % (self.get_frame_count()-1)
	ret += 'set sTimeSteps := pTimeBegin .. pTimeEnd;\n'

	#write joint timing sets
	for j in self.Character.get_joints_contact():
            contactSet = self.get_contact_frames(j)
	    contactStr = '{' + (', '.join(map(str, contactSet))) + '}'
            ret += 'set sTimeSteps_%sOn := %s;\n' % (j.Name, contactStr)
            ret += 'set sTimeSteps_%sOff := sTimeSteps diff sTimeSteps_%sOn;\n' % (j.Name, j.Name)
            ret += '\n'
	ret += '\n'
	return ret

    def _write_specifiers(self):
        ret = ''

        #write constraints
	for eq in self.CachedConstraintList:
            ret += str(eq)

	#write weighted objectives
	if len(self.CachedObjectiveList) > 0:
	    ret += 'minimize objective: (\n'
	    for i, obj in enumerate(self.CachedObjectiveList):
                ret += str(obj)
		if(i == len(self.CachedObjectiveList)-1):
		    ret += ') / (pTimeEnd+1);\n' #we divide by time so animations of different lengths can be compared fairly
		else:
		    ret += ' +\n'
	return ret

    def _write_footer(self, solver):
	ret = ''
	ret = 'option reset_initial_guesses 1;\n'
	#ret += 'option show_stats 1;\n'
	ret += 'option solver ' + solver + ';\n'
	ret += 'option ipopt_options \'max_iter=10000 print_level=0\';\n' #TODO: what about other solvers? max_cpu_time=1200
        ret += 'option snopt_options \'meminc=10000000\';\n' #TODO: what about other solvers?
	ret += 'solve;\n'
	ret += '\n'

	ret += 'display solve_result;\n'

	if len(self.CachedObjectiveList) > 0:
	    ret += 'display objective;\n'

	    #for interest we can output the values of individual objectives in the solution
	    for i, obj in enumerate(self.CachedObjectiveList):
		ret += obj.write_debug_str()

	ret += 'if solve_result = "solved" then {\n'
        for frame in range(0, self.get_frame_count()):
            for body in self.Character.BodyList:
                varstr = ', '.join([str(body.q[x]) + '[' + str(frame) + ']' for x in range(0, dof)])
                fstr = ', '.join(['%f'] * dof)
                ret += '\tprintf "' + str(body.Name) + '[' + str(frame) + '] = ' + fstr + '\\n", ' + varstr + ';\n'
	ret += '}\nexit;\n'
	return ret

    def _solvedcallback(self, amplresult):
        self.Done = True

        #cache solution to a file
        file = open(self.Name + '.amplsol', 'w')
        file.write(amplresult)
        file.close()

	#did it solve correctly?
	self.Solved = ("solve_result = solved" in amplresult)
	if self.Solved:
	    if len(self.CachedObjectiveList) > 0:
		objectivematch = re.search("(?<=objective = )" + regex_float, amplresult)
		self.ObjectiveValue = float(objectivematch.group(0))

            #read the solution variables into a dict {indexed on body name}[frame][dof]
	    self.AnimationData = {}
            for body in self.Character.BodyList:
                self.AnimationData[str(body.Name)] = [None] * self.get_frame_count()
                for frame in range(0, self.get_frame_count()):
                    regex_float_str = ', '.join([regex_float] * dof)
                    pattern = str(body.Name) + "\[" + str(frame) + "\] = " + regex_float_str
                    match = re.findall(pattern, amplresult)[0]
                    q = [float(match[x * 2]) for x in range(dof)]
                    self.AnimationData[str(body.Name)][frame] = q

            #if looped, append an extra frame (identical to first frame, but offset)
	    for s in self.SpecifierList:
		if isinstance(s, SpecifierPluginLoop):
                    for frame in range(0, 1):   #duplicate first 1 frame
                        for b in self.Character.BodyList:
                            q = self.AnimationData[str(b.Name)][frame]
                            q = s.get_offset(q, 1) #apply offset
                            q = map(float, q)
                            self.AnimationData[str(b.Name)].append(q) #append extra frame

            print('%s solved! (Objective = %f)' % (self.Name, self.ObjectiveValue))

            self.export('.')    #export immediately so we can see the results

        else:
            print('%s failed!' % self.Name)

    def export(self, outdir):
        if self.Solved is False:
            raise BaseException('Animation is not solved. Cannot export!')

        filename = outdir + "\\" + self.Name + '.bvh'
        print('Writing: %s,' % filename),
        file = openfile(filename, 'w')
        file.write(export_bvh(self))
        file.close()

        '''
        filename = outdir + "\\" + self.Name + '.flat.bvh'
        print('%s,' % filename),
        file = openfile(filename, 'w')
        file.write(export_bvh_flat(self))
        file.close()
        '''

        filename = outdir + "\\" + self.Name + '.skeleton.xml'
        print('%s' % filename)
        file = openfile(filename, 'w')
        file.write(export_ogre_skeleton_xml(self))
        file.close()

    def _solve(self, solver, writeAMPL=False):
	'''This handles the 'inner' (spacetime) optimization. It assumes that
	length and contact timings are set. Use optimize() instead.'''

	#reset the solution
        self.Done = False
	self.Solved = False
	self.ObjectiveValue = numpy.NaN
	self.AnimationData = {}
        self.CachedConstraintList = []
        self.CachedObjectiveList = []

        #split specifiers into constraints and objectives for easier processing
        for s in self.SpecifierList:
            #regular constraints/objectives
            if isinstance(s, Constraint):
                self.CachedConstraintList.append(s)
            elif isinstance(s, Objective):
                self.CachedObjectiveList.append(s)

            #plugins
            elif isinstance(s, SpecifierPlugin):
                for c in s.get_specifiers(self, self.Character):
                    if isinstance(c, Constraint):
                        self.CachedConstraintList.append(c)
                    elif isinstance(c, Objective):
                        self.CachedObjectiveList.append(c)

        #generate the ampl model
        amplcmd = ''
        amplcmd += self._write_header()
        amplcmd += self.Character.get_model()   #character body & physical eq.
        amplcmd += self._write_specifiers()     #other constraints & objectives
        amplcmd += self._write_footer(solver)

        #for debugging purposes we'll write out the ampl file
        if writeAMPL:
            file = open(self.Name + '.ampl', 'w')
            file.write(amplcmd)
            file.close()

	try:
	    #try to load cached solution
	    file = open(self.Name + '.amplsol', 'r')
	    amplresult = file.read();
	    file.close()
            #pretend it solved, and use the callback
            self._solvedcallback(amplresult)

	except IOError:
	    #couldn't load cached solution file, we'll have to solve it with ampl
            #use the thread pool for this
            pool.add_job(amplsolve, args=[amplcmd], return_callback=self._solvedcallback)

    def optimize(self, solver):
	'''This handles the 'outer' optimization that's necessary to determine
	animation length and contact timings (if they are not explicitly provided).'''

	optLength = self.Length is None
	optContacts = self.ContactTimesDict is None and len(self.Character.get_joints_contact()) > 0

	if optLength or optContacts:
	    print("Starting CMA-ES optimization for %s..." % self.Name)

	    startPoint = []
	    lowerBounds = []
	    upperBounds = []
	    if optLength:
		startPoint.append(0.5)
		lowerBounds.append(self.get_frame_length() * 3.0) #3 frame minimum
		upperBounds.append(1.0)
	    if optContacts:
		f = 1.0 / len(self.Character.get_joints_contact())
		for j, joint in enumerate(self.Character.get_joints_contact()):
		    evenly = (j * f) + (f / 2.0) #space the contacts evenly
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
				self.ContactTimesDict[joint] = [(x[j * 2 + 0 + m], x[j * 2 + 1 + m])]
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
		    self.ContactTimesDict[joint] = [(es.best[0][j * 2 + 0 + m], es.best[0][j * 2 + 1 + m])]
	    return self._solve(solver, writeAMPL=True)

	else:
	    print("CMA-ES optimization unnecessary for %s. Solving..." % self.Name)
	    return self._solve(solver, writeAMPL=True)
