from __future__ import division
import math
import os
import subprocess
import sympy

#for fast quat operations (good for manipulating animation
#data, but unsuitable for spacetime control expressions)
from cgkit import cgtypes

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


def num_q_lerp(qA, qB, weight):
    assert(True not in [(math.isnan(x) or math.isinf(x)) for x in qA+qB])
    eulerA, eulerB = qA[3:dof], qB[3:dof]
    quatA, quatB = num_euler_to_quat(eulerA), num_euler_to_quat(eulerB)
    posA, posB = qA[:3], qB[:3]
    newQuat = cgtypes.slerp(weight, quatA, quatB)
    newEuler = num_quat_to_euler(newQuat)
    newPos = vec3_lerp(posA, posB, weight)
    ret = newPos + newEuler
    return ret

def vec3_lerp(v1, v2, weight):
    v1x, v1y, v1z = v1
    v2x, v2y, v2z = v2
    dx = v2x-v1x
    dy = v2y-v1y
    dz = v2z-v1z
    dxt = dx * weight
    dyt = dy * weight
    dzt = dz * weight
    return [v1x + dxt, v1y + dyt, v1z + dzt]

def num_euler_to_quat(euler):
    '''Converts 3 euler angles XYZ to a quaternion (YXZ order)'''
    assert(True not in [(math.isnan(x) or math.isinf(x)) for x in euler])
    rx, ry, rz = euler
    q_xrot = cgtypes.quat().fromAngleAxis(rx, [1, 0, 0])
    q_yrot = cgtypes.quat().fromAngleAxis(ry, [0, 1, 0])
    q_zrot = cgtypes.quat().fromAngleAxis(rz, [0, 0, 1])
    return q_yrot * q_xrot * q_zrot #YXZ order

def num_quat_to_euler(quat):
    '''Converts a quaternion to 3 euler angles XYZ (YXZ order)'''
    assert(True not in [(math.isnan(x) or math.isinf(x)) for x in [quat.x, quat.y, quat.z, quat.w]])
    quat = quat.normalize()
    mat = quat.toMat3()
    euler = mat.toEulerYXZ() #YXZ order
    return list(euler)

def sym_euler_to_matrix(euler):
    '''(symbolic!) Converts 3 euler angles XYZ to a rotation matrix (YXZ order)'''
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
    #we use YXZ order (y-axis first) so that rotations around the y-axis can be done
    #by just changing the y euler angle (without needing to multiply matricies)
    return Y * X * Z

def sym_matrix_to_euler(mat):
    '''(symbolic!) Converts a rotation matrix (YXZ order) to 3 euler angles XYZ. Note that
    for simplicity this ignores the singularities.'''
    #XYZ
    #x = sympy.atan2(-mat[1, 2], mat[2, 2])
    #y = sympy.asin(mat[0, 2])
    #z = sympy.atan2(-mat[0, 1], mat[0, 0])

    #YXZ
    x = sympy.asin(-mat[1, 2])
    y = sympy.atan2(mat[0, 2], mat[2, 2])
    z = sympy.atan2(mat[1, 0], mat[1, 1])

    return [x, y, z]

def sym_world_xf(point, coords, worldToLocal=False):
    '''(symbolic!) Transforms a point from local to world coordinates'''
    trans = sympy.Matrix(coords[:3])
    p = sympy.Matrix([point[0], point[1], point[2]])

    if worldToLocal:
	euler = [-x for x in coords[3:]]
	euler.reverse()
	rotinv = sym_euler_to_matrix(euler)
	return rotinv * (p - trans)	#world to local
    else:
	euler = coords[3:]
	rot = sym_euler_to_matrix(euler)
	return (rot * p) + trans	#local to world

def num_world_xf(point, coords, worldToLocal=False):
    '''Transforms a point from local to world coordinates. This gives the same
    result as sym_world_xf, but is numerical (and thus much faster).'''
    assert(True not in [(math.isnan(x) or math.isinf(x)) for x in point+coords])

    quat = num_euler_to_quat(coords[3:dof])
    trans = cgtypes.vec3(coords[:3])
    p = cgtypes.vec3(point)

    ret = None
    if worldToLocal:
        quat = quat.inverse()
        ret = quat.rotateVec(p - trans) #world to local
    else:
        ret = quat.rotateVec(p) + trans #local to world
    return [ret.x, ret.y, ret.z]

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
	#this heuristic is hacky...
	funcList = ['sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'atan2']
	if (len(expr.args) == 1 and not expr.args[0].is_Function) and expr.func.__name__ not in funcList:
	    #first argument is not a function; this might be a time-dep var
	    return expr.func.__name__ + "[(%s+card(sTimeSteps)) mod card(sTimeSteps)]" % self.stringify(expr.args, ", ")    #TODO: looping hack (card() gets set length)
	else:
	    #first argument is a function; default behaviour
	    return sympy.printing.str.StrPrinter._print_Function(self, expr)

def ampl(expr, ** settings):
    return AmplPrinter(settings).doprint(expr)

def amplsolve(amplcmd):
    '''Runs ampl with the given command and returns the stdout output'''
    ampl = subprocess.Popen("ampl", stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    amplresult = ampl.communicate(amplcmd)[0] #blocking call
    return amplresult

def guess_contact_time(leg_length=1.0, speed=1.0):
    '''Calculates a reasonable contact (stance) time (s) given leg length (m) and movement speed (m/s)'''
    #formula from paper: TIME OF CONTACT AND STEP LENGTH...
    #http://jeb.biologists.org/content/203/2/221.full.pdf
    time_contact = (0.80 * leg_length ** 0.84) / (speed ** 0.87)
    return time_contact #in seconds