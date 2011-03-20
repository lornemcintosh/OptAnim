from __future__ import division
import os
import sympy
import math

#degrees of freedom
dof = int(6) #3 translational + 3 rotational

#time
#TODO: find a better place for time symbol?
t = sympy.Symbol("t")

#timestep length
pH = sympy.Symbol("pH")

#gravity vector
#TODO: find a better place for gravity?
g = sympy.Matrix([[0, -9.81, 0, 0, 0, 0]])

#this regex pattern will match floats (ex. -234 or 1.1 or .3 or +4.23e-8)
regex_float = "([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)"

def quat_norm(quat):
    qw, qx, qy, qz = quat
    len = sympy.sqrt(qw ** 2 + qx ** 2 + qy ** 2 + qz ** 2)
    return [qw / len, qx / len, qy / len, qz / len]

def quat_to_axisangle(quat):
    qw, qx, qy, qz = quat
    if (qw > 1):
	qw, qx, qy, qz = quat_norm(quat)
    angle = 2 * sympy.acos(qw)
    s = sympy.sqrt(1-qw * qw)
    if (s < 0.0001):
	x = 1
	y = z = 0
    else:
	x = qx / s #normalise axis
	y = qy / s
	z = qz / s
    return [x, y, z, angle]

def matrix_to_quat(m):
    #agh! sympy is a mess, so this is using regular math libraries
    w = math.sqrt(max(0, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2.0;
    x = math.sqrt(max(0, 1 + m[0, 0] - m[1, 1] - m[2, 2])) / 2.0;
    y = math.sqrt(max(0, 1 - m[0, 0] + m[1, 1] - m[2, 2])) / 2.0;
    z = math.sqrt(max(0, 1 - m[0, 0] - m[1, 1] + m[2, 2])) / 2.0;

    x = math.copysign(x, m[2, 1] - m[1, 2])
    y = math.copysign(y, m[0, 2] - m[2, 0])
    z = math.copysign(z, m[1, 0] - m[0, 1])
    return [w, x, y, z]

def euler_to_matrix(euler):
    rx, ry, rz = euler
    X = sympy.Matrix([
		     [1,	0,	0],
		     [0,	sympy.cos(rx), -sympy.sin(rx)],
		     [0,	sympy.sin(rx), sympy.cos(rx)]
		     ])
    Y = sympy.Matrix([
		     [sympy.cos(ry), 0, sympy.sin(ry)],
		     [0, 1, 0],
		     [-sympy.sin(ry), 0, sympy.cos(ry)]
		     ])
    Z = sympy.Matrix([
		     [sympy.cos(rz),	-sympy.sin(rz),	0],
		     [sympy.sin(rz),	sympy.cos(rz),	0],
		     [0,	0,	1]
		     ])
    return X * Y * Z

def matrix_to_euler(mat):
    x = y = z = 0
    if float(-mat[0, 2]) != 1.0 and float(-mat[0, 2]) != -1.0:
	y = -sympy.asin(-mat[0, 2])
	x = sympy.atan2(-mat[1, 2] / sympy.cos(y), mat[2, 2] / sympy.cos(y))
	z = sympy.atan2(-mat[0, 1] / sympy.cos(y), mat[0, 0] / sympy.cos(y))
    else:
	z = 0
	if(float(-mat[0, 2]) == -1):
	    y = sympy.pi / 2.0
	    x = z + sympy.atan2(-mat[1, 0], -mat[2, 0])
	else:
	    y = -sympy.pi / 2.0
	    x = -z + sympy.atan2(mat[1, 0], mat[2, 0])
    return [x, y, z]

def world_xf(point, coords, worldToLocal=False):
    '''transforms a point from local to world coordinates'''
    trans = sympy.Matrix(coords[:3])
    p = sympy.Matrix([point[0], point[1], point[2]])

    if worldToLocal:
	euler = [-x for x in coords[3:]]
	euler.reverse()
	rotinv = euler_to_matrix(euler)
	return rotinv * (p - trans)	#world to local
    else:
	euler = coords[3:]
	rot = euler_to_matrix(euler)
	return (rot * p) + trans	#local to world
    

def openfile(filepath, arg):
    '''opens a file and creates the directory if necessary'''
    dir = os.path.dirname(filepath)
    if not os.path.isdir(dir):
	os.makedirs(dir)
    return open(filepath, arg)


class AmplPrinter(sympy.printing.str.StrPrinter):
    '''A printer for the AMPL format. For now this is just a hack to handle
    brackets properly and doesn't do anything too nifty'''
    
    def _print_Function(self, expr):	
	#TODO: HACK: we need to put [] around arguments of time-dependent variables,
	#but keep () around the arguments of normal functions like cos()
	#this heuristic is hacky... and will break in obvious cases like cos(6)
	if(len(expr.args) == 1 and not expr.args[0].is_Function):
	    #first argument is not a function; this might be a time-dep var
	    return expr.func.__name__ + "[%s]" % self.stringify(expr.args, ", ")
	else:
	    #first argument is a function; default behaviour
	    return sympy.printing.str.StrPrinter._print_Function(self, expr)

def ampl(expr, ** settings):
    return AmplPrinter(settings).doprint(expr)