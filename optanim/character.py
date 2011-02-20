from __future__ import division
import os
import sympy

from constraint import *
from joints import *
from utils import *

class Character(object):
    '''Represents a physically simulated character (composed of bodies and joints).'''
    def __init__(self, name):
	'''Constructor, creates an empty character (no bodies or joints)'''
	self.name = name
	self.bodyList = []
	self.jointList = []

    def add_body(self, body):
	self.bodyList.append(body)

    def add_joint(self, joint):
	self.jointList.append(joint)
	#for convenience add some links to parent/child bodies
	#this assumes bodyA is "up" to the root
	if isinstance(joint, JointRevolute):
	    joint.BodyA.add_child(joint.BodyB)
	    joint.BodyB.set_parent(joint.BodyA)
	
    def get_newtonian_constraints(self, name, tPrev=t-1, tCurr=t, tNext=t+1, tRange='pTimeBegin < t < pTimeEnd', offset=[0,0,0]):
	#timestep length
	pH = sympy.Symbol("pH")

	#gravity vector
	#TODO: find a better place for gravity
	g = sympy.Matrix([[0, -9.81, 0]])

	#make the state vector q
	qList = []
	for body in self.bodyList:
	    qList.extend(body.q)
	q = sympy.Matrix(qList).T

	#make the mass vector m
	mList = []
	for body in self.bodyList:
	    mList.extend(body.Mass)
	m = sympy.Matrix(mList).T

	#make the joint force vector jf
	jfList = []
	for joint in self.jointList:
	    jfList.extend(joint.f)
	jf = sympy.Matrix(jfList).T

	#setup joint constraint forces
	jcList = []
	for joint in self.jointList:
	    jcList.extend(joint.get_state_constraints())

	jc = sympy.Matrix([x.c for x in jcList])  #take only constraint (not bounds)

	#----------------------------------------------------
	#TODO: FIXME: this is dumb, but jacobian doesn't work with functions,
	# so we take off the (t), and then add it back afterwards
	for i in q:
	    jc = jc.subs(i(t), i)

	#jacobian
	J = jc.jacobian(q)
	Jlam = J.T * jf.T

	#add the (t) back to each time-dependant variable
	for i in q:
	    Jlam = Jlam.subs(i, i(tCurr))
	for i in jf:
	    #note: we do tNext here so that the forces on frame t
	    #will explain movement between frame t-1 and frame t
	    Jlam = Jlam.subs(i, i(tNext))
	#----------------------------------------------------

	constraints = []
	for i, x in enumerate(q):
	    a = (x(tNext) - 2 * x(tCurr) + x(tPrev) + offset[i % 3]) / (pH ** 2)
	    fm = g[i % 3] + (Jlam[i] / m[i])
	    afm = ConstraintEq(name + str(i), a, fm, tRange)
	    constraints.append(afm)

	return constraints

    def get_model(self):
	model = ''

	#write state variables
	for body in self.bodyList:
	    for q in body.q:
		model += ('var %s {sTimeSteps};\n' % q)
	model += '\n'
	
	#write joint force variables
	for joint in self.jointList:
	    for f in joint.f:
		model += ('var %s {sTimeSteps};\n' % f)
	model += '\n'
	
	#write newtonian constraints
	newtonList = self.get_newtonian_constraints('AFM')
	for c in newtonList:
	    model += (str(c))
	model += '\n'

	#joint constraints
	for joint in self.jointList:
	    for i, eq in enumerate(joint.get_state_constraints() + joint.get_force_constraints()):
		model += (str(eq))
	    model += '\n'

	return model

    def write_bvh_hierarchy(self, file, root, level, rootoffset):
	tab = '\\t' * level
	if level == 0:
	    #special case for root
	    file.write('printf "%sROOT %s\\n" > (filename);\n' % (tab, root.Name));
	    file.write('printf "%s{\\n" > (filename);\n' % tab);
	    level += 1; tab = '\\t' * level;
	    file.write('printf "%sOFFSET\\t%f\\t%f\\t%f\\n" > (filename);\n' % tuple([tab] + rootoffset));
	    file.write('printf "%sCHANNELS 6 Xposition Yposition Zposition Zrotation Xrotation Yrotation\\n" > (filename);\n' % tab);
	else:
	    #regular case
	    file.write('printf "%sJOINT %s\\n" > (filename);\n' % (tab, root.Name));
	    file.write('printf "%s{\\n" > (filename);\n' % tab);
	    level += 1; tab = '\\t' * level;
	    file.write('printf "%sOFFSET\\t%f\\t%f\\t%f\\n" > (filename);\n' % tuple([tab] + [0, 0, -root.parent.Length]));
	    file.write('printf "%sCHANNELS 3 Zrotation Xrotation Yrotation\\n" > (filename);\n' % tab);
	if len(root.childList) > 0:
	    for child in root.childList:
		self.write_bvh_hierarchy(file, child, level, rootoffset)
	else:
	    file.write('printf "%sEnd Site\\n" > (filename);\n' % tab);
	    file.write('printf "%s{\\n" > (filename);\n' % tab);
	    level += 1; tab = '\\t' * level;
	    file.write('printf "%sOFFSET\\t%f\\t%f\\t%f\\n" > (filename);\n' % tuple([tab] + [0, 0, -root.Length]));
	    level -= 1; tab = '\\t' * level;
	    file.write('printf "%s}\\n" > (filename);\n' % tab);
	level -= 1; tab = '\\t' * level;
	file.write('printf "%s}\\n" > (filename);\n' % tab);
	return

    def write_bvh_motion(self, file, root, level):
	if level == 0:
	    #special case for root
	    file.write('\t#position of %s\n' % root.Name);
	    file.write('\tprintf "%f ", 0 > (filename);\n');
	    file.write('\tprintf "%f ", ' + str((sympy.cos(root.q[2](t) + (sympy.pi / 2.0)) * root.Length / 2) + root.q[0](t)).replace("(t)", "[t]") + ' > (filename);\n');
	    file.write('\tprintf "%f ", ' + str((sympy.sin(root.q[2](t) + (sympy.pi / 2.0)) * root.Length / 2) + root.q[1](t)).replace("(t)", "[t]") + ' > (filename);\n');
	    file.write('\n');

	    file.write('\t#rotation of %s\n' % root.Name);
	    file.write('\tprintf "%f ", 0 > (filename);\n');
	    file.write('\tprintf "%f ", ' + str((root.q[2](t)) * (180.0 / sympy.pi)).replace("(t)", "[t]") + ' > (filename);\n');
	    file.write('\tprintf "%f ", 0 > (filename);\n');
	    file.write('\n');
	else:
	    #regular case
	    file.write('\t#rotation of %s\n' % root.Name);
	    file.write('\tprintf "%f ", 0 > (filename);\n');
	    file.write('\tprintf "%f ", ' + str((-root.parent.q[2](t) + root.q[2](t)) * (180.0 / sympy.pi)).replace("(t)", "[t]") + ' > (filename);\n');
	    file.write('\tprintf "%f ", 0 > (filename);\n');
	    file.write('\n');

	level += 1;
	for child in root.childList:
	    self.write_bvh_motion(file, child, level)
	level -= 1;

	return

    def write_exporter(self, outdir):
	file = openfile(os.path.join(outdir, self.name + '-exporter.ampl'), 'w')
	file.write('param filename symbolic := "%s_" & pAnimName & ".bvh";\n' % self.name)

	#start BVH format:
	root = self.bodyList[0]
	file.write('printf "HIERARCHY\\n" > (filename);\n');

	#write hierarchy
	self.write_bvh_hierarchy(file, root, 0, [0, 0, 1.57])

	file.write('printf "MOTION\\n" > (filename);\n');
	file.write('printf "Frames: %i\\n", pTimeEnd+1 > (filename);\n');
	file.write('printf "Frame Time: %f\\n", pH > (filename);\n');

	file.write('for {t in sTimeSteps}\n');
	file.write('{\n');

	#write motion
	self.write_bvh_motion(file, root, 0)

	#TODO: remove me: for debugging contact forces
	#file.write('\tprintf "--x--%f ", foot1_fx[t] > (filename);\n')
	#file.write('\tprintf "--y--%f ", foot1_fy[t] > (filename);\n')

	file.write('\tprintf "\\n" > (filename);\n');
	file.write('}\n');
	file.write('printf "\\n" > (filename);\n');

	return