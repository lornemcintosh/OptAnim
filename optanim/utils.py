from __future__ import division
import os
import sympy

#time
#TODO: find a better place for time symbol?
t = sympy.Symbol("t")

def world_xf(localpoint, coords):
    '''transforms a point from local to world coordinates'''
    x, y, theta = coords
    m = sympy.Matrix([
		     [sympy.cos(theta), -sympy.sin(theta), x],
		     [sympy.sin(theta), sympy.cos(theta), y],
		     [0, 0, 1]
		     ])
    c = sympy.Matrix([localpoint[0], localpoint[1], 1])
    return m * c


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