#!/usr/bin/env python2.6
"""module cma implements the CMA-ES, Covariance Matrix Adaptation Evolution Strategy,
a stochastic optimizer for robust non-linear non-convex function minimization
 
CMA-ES searches for a minimizer (a solution x in R**n) of an
objective function f (cost function), such that f(x) is
minimal. Regarding f, only function values for candidate solutions
need to be available, gradients are not necessary. Even less
restrictive, only a passably reliably ranking of the candidate
solutions in each iteration is necessary. 

Two interfaces are provided: 

    function `fmin(func, x0, sigma0,...)`
        runs a complete minimization of the objective function func with CMA-ES. 

    class `CMAEvolutionStrategy`
        allows for minimization such that the control of the iteration 
        loop remains with the user. 

Used packages (only `numpy` is inevitable): 
    `numpy` (see barecmaes.py if numpy is not available) 
    `time` 
    `sys` 

    optionally used (by default) : 
        `pprint` (for pretty-print) 
        `matplotlib.pylab` (for `plotdata`)

    optionally used (by default not): 
        `pygsl`

:Example:
    import cma
    help(cma)  # this help message
    help(cma.fmin)
    
:See also: fmin(), Options, CMAEvolutionStrategy. 

:Author: Nikolaus Hansen, 2008-2011
:License: GPL 2 and 3

"""

from __future__ import division  # TODO: check unit test with and w/o, then N/2. can be changed
__version__ = "0.9.52 $Revision: 2525 $" 
#    $Date: 2011-02-06 03:00:02 +0100 (Sun, 06 Feb 2011) $
#    bash: svn propset svn:keywords 'Date Revision' cma.py

#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, version 2 or 3.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
# 

# to test: 
#   pyflakes cma.py   # finds bugs by static analysis
#   pychecker --limit 60 cma.py  # also executes, gives 60 warnings (all checked)
#   python cma.py -t  # executes implemented tests based on doctest
# to create a html documentation file: 
#    pydoc -w cma  # edit the header (remove local pointers) 
#    epydoc cma.py  # comes close to javadoc but does not find the 
#                   # links of function references etc 
#    doxygen needs @package cma as first line in the module docstring
#       some things like class attributes are not interpreted correctly
#    sphinx: doc style of doc.python.org, could not make it work

# TODO re-consider interface:
# read settable "options" from a (properties) file, see myproperties.py
# 
# typical parameters in scipy.optimize: disp, xtol, ftol, maxiter, maxfun, callback=None
#         maxfev, diag (A sequency of N positive entries that serve as 
#                 scale factors for the variables.)
#           full_output -- non-zero to return all optional outputs.
#   If xtol < 0.0, xtol is set to sqrt(machine_precision)
#    'infot -- a dictionary of optional outputs with the keys:
#                      'nfev': the number of function calls...
# 
#    see eg fmin_powell 
# typical returns
#        x, f, dictionary d 
#        (xopt, {fopt, gopt, Hopt, func_calls, grad_calls, warnflag}, <allvecs>)
#
# TODO: implement rank-lambda update (aCMA)
# TODO: implement constraints handling
# TODO: option full_output -- non-zero to return all optional outputs.
# TODO: check new options code
# TODO: utilize doctest, extend function unitdoctest 
# TODO: implement equal-fitness termination, covered by stagnation?
# TODO: apply style guide: no capitalizations!? 
# TODO: API: fmin -> fminimize or fmin
# TODO: check and test noisehandling
# TODO: check and test dispdata() 
# TODO: eigh(): thourough testing would not hurt
# TODO: implement cholesky_update(): rank-one update of a cholesky
#    matrix and its inverse. 
#
# TODO solve problem with show(): use testcma.py: graphic does not show up, but show()
#   is needed. There should be another function which does not enter interactive mode. 
#   
# TODO: import should rather be local in general!?
# TODO (later): implement readSignals from a file like properties file (to be called after tell())

import sys   # not really essential
import time  # not really essential
import numpy as np # arange, cos, size, eye, inf, dot, floor, outer, zeros, linalg.eigh, sort, argsort, random, ones,... 
from numpy import array, log, sqrt, sum, exp, inf  # to access the built-in sum fct:  __builtins__.sum or del sum removes the imported sum and recovers the shadowed 
try:
    import matplotlib.pylab as pylab  # also: use ipython -pylab
    savefig = pylab.savefig   # we would like to be able to use cma.savefig() etc
    close = pylab.close
    show = pylab.show
except:
    print '  could not import pylab, cma.plotdata() not available'
    
# __docformat__ = "reStructuredText"  # this hides some comments entirely 

# TODO: check scitools.easyviz and how big the adaptation would be

# why not package math? 
# sys.py3kwarning = True  # TODO: out-comment from version 2.6
# from __future__ import with_statement  # need in version 2.5, not used anyway

# changes:
# 11/02/05: work around a memory leak in numpy 
# 11/02/05: plotting routines improved 
# 10/10/17: cleaning up, now version 0.9.30 
# 10/10/17: bug-fix: return values of fmin now use phenotyp (relevant 
#           if input scaling_of_variables is given)
# 08/10/01: option evalparallel introduced,
#           bug-fix for scaling being a vector 
# 08/09/26: option CMAseparable becomes CMA_diagonal
# 08/10/18: some names change, test functions go into a class
# 08/10/24: more refactorizing
# 10/03/09: upper bound exp(min(1,...)) for step-size control


# TODO: this would define the visible interface
# __all__ = ['fmin', 'CMAEvolutionStrategy', 'plotdata', ...] 
#                                                                     


# emptysets = ('', (), [], {}) # array([]) does not work but also np.size(.) == 0  
# "x in emptyset" cannot be well replaced by "not x" 
# which is also True for array([]) and None, but also for 0 and False, and False for NaN

#____________________________________________________________
#____________________________________________________________
#
def unitdoctest():
    """ is used to describe test cases and might in future become helpful
    as an experimental tutorial as well.  
    
    A simple first overall test: 
        >>> import cma
        >>> res = cma.fmin(cma.fcts.elli, 3*[1], 1, CMA_diagonal=2, seed=1, verb_time=0)
        (3_w,7)-CMA-ES (mu_w=2.3,w_1=58%) in dimension 3 (seed=1)
           Covariance matrix is diagonal for 2 iterations (1/ccov=7.0)
        Iterat #Fevals   function value     axis ratio  sigma   minstd maxstd min:sec
            1       7 1.453161670768570e+04 1.2e+00 1.08e+00  1e+00  1e+00 
            2      14 3.281197961927600e+04 1.3e+00 1.22e+00  1e+00  2e+00 
            3      21 1.082851071704020e+04 1.3e+00 1.24e+00  1e+00  2e+00 
          100     700 8.544042012075362e+00 1.4e+02 3.18e-01  1e-03  2e-01 
          200    1400 5.691152415221861e-12 1.0e+03 3.82e-05  1e-09  1e-06 
          220    1540 3.890107746209078e-15 9.5e+02 4.56e-06  8e-11  7e-08 
        termination on tolfun : 1e-11
        final/bestever f-value = 3.89010774621e-15 2.52273602735e-15
        mean solution:  [ -4.63614606e-08  -3.42761465e-10   1.59957987e-11]
        std deviation: [  6.96066282e-08   2.28704425e-09   7.63875911e-11]
    
    Test on the Rosenbrock function with 3 restarts. The first trial only
    finds the local optimum, which happens in about 20% of the cases.
        >>> import cma
        >>> res = cma.fmin(cma.fcts.rosen, 4*[1],1, ftarget=1e-6, restarts=3, verb_time=0, verb_disp=500, seed=3)
        (4_w,8)-CMA-ES (mu_w=2.6,w_1=52%) in dimension 4 (seed=3)
        Iterat #Fevals   function value     axis ratio  sigma   minstd maxstd min:sec
            1       8 4.875315645656848e+01 1.0e+00 8.43e-01  8e-01  8e-01 
            2      16 1.662319948123120e+02 1.1e+00 7.67e-01  7e-01  8e-01 
            3      24 6.747063604799602e+01 1.2e+00 7.08e-01  6e-01  7e-01 
          184    1472 3.701428610430019e+00 4.3e+01 9.41e-07  3e-08  5e-08 
        termination on tolfun : 1e-11
        final/bestever f-value = 3.70142861043 3.70142861043
        mean solution:  [-0.77565922  0.61309336  0.38206284  0.14597202]
        std deviation: [  2.54211502e-08   3.88803698e-08   4.74481641e-08   3.64398108e-08]
        (8_w,16)-CMA-ES (mu_w=4.8,w_1=32%) in dimension 4 (seed=4)
        Iterat #Fevals   function value     axis ratio  sigma   minstd maxstd min:sec
            1    1489 2.011376859371495e+02 1.0e+00 8.90e-01  8e-01  9e-01 
            2    1505 4.157106647905128e+01 1.1e+00 8.02e-01  7e-01  7e-01 
            3    1521 3.548184889359060e+01 1.1e+00 1.02e+00  8e-01  1e+00 
          111    3249 6.831867555502181e-07 5.1e+01 2.62e-02  2e-04  2e-03 
        termination on ftarget : 1e-06
        final/bestever f-value = 6.8318675555e-07 1.18576673231e-07
        mean solution:  [ 0.99997004  0.99993938  0.99984868  0.99969505]
        std deviation: [ 0.00018973  0.00038006  0.00076479  0.00151402]

    Notice the different termination conditions. Termination on the target 
    function value ftarget prevents further restarts. 
    
    Test of scaling_of_variables option
        >>> import cma
        >>> opts = cma.Options()
        >>> opts['seed'] = 456
        >>> opts['verb_disp'] = 0
        >>> # rescaling of third variable: for searching in  roughly
        >>> #   x0 plus/minus 1e3*sigma0 (instead of plus/minus sigma0)
        >>> opts.scaling_of_variables = [1, 1, 1e3, 1]
        >>> res = cma.fmin(cma.fcts.rosen, 4 * [0.1], 0.1, **opts)
        termination on tolfun : 1e-11
        final/bestever f-value = 2.68096173031e-14 1.09714829146e-14
        mean solution:  [ 1.00000001  1.00000002  1.00000004  1.00000007]
        std deviation: [  3.00466854e-08   5.88400826e-08   1.18482371e-07   2.34837383e-07]
    
    The printed std deviations reflect the actual true value (not the one
    in the internal representation which would be different). 
    """
    
    pass

#____________________________________________________________
#____________________________________________________________
#
class DictClass(dict):
    """ a dictionary that understands dot-syntax of class attributes 
    for its key values, given they are strings, that is, 
    (dic['fieldname'] is dic.fieldname) is true.
    
    This tries to gather both convenience features from dict and class. 
    :Example:
        >>> import cma
        >>> dic = cma.DictClass()
        >>> dic.s = "an entry in dictionary dic with key 's'"
        >>> print dic['s']
        an entry in dictionary dic with key 's'
       
    :Note: 
        If need be this class might be removed. 
    """
       
    def __init__(self, **kw):
        dict.__init__(self, kw)
        self.__dict__ = self  # this self reference relies on cycle detecting gc
        
    # def __del__(self):  # use of del is (strongly) discouraged
    #     self.__dict__ = {}  # remove cyclic reference, TODO: does this make sense?


#____________________________________________________________
#____________________________________________________________
#
class Solution(np.ndarray):
    """this class is a stump, but functional and in use: Solution represents 
    a candidate solution (a vector of real numbers), for example returned by ask(). 
    The class inherits from numpy.ndarray and can therefore be used as 
    a numpy array. Additionally, the solution can be repaired based on domain boundaries. 
    The input args to further available methods are subject to change.  
    
    :Attributes:
        unrepaired -- ndarray, last solution before repair() was called, 
            or None
        isgeno -- whether the solution represents a genotyp (internal
            representation in CMAEvolutionStrategy)

    Later: add attributes for the f-value f and the nb of f-evaluations 
    evals? 
    
    :Example: 
        >>> import cma
        >>> x = cma.Solution([0, -2, 3])
        >>> x                         # is a float np.array
        Solution([ 0., -2.,  3.])
        >>> x.repair([0.5, None])     # set lower domain bound to 0.5
        Solution([ 0.5,  0.5,  3. ])
        >>> x                         # check that x was changed
        Solution([ 0.5,  0.5,  3. ])
        >>> x.unrepaired              # original values
        array([ 0., -2.,  3.])
        >>> x.isgeno                  # by default not the internal repr.
        False
        
    """
        
    def __init__(self, x, copy=False, geno=False): 
              # f=np.NaN, evals=np.NaN, ldom=None, udom=None):
        # by now __new__(self, x, copy=False, geno=False) has been
        # called implicitly, so there is nothing to do. 
        self.isgeno = geno  # overwrite values from x
        self.unrepaired = None
        return
        
    def __new__(thistype, data, copy=False, geno=False):
        """returns a new object instance of type thistype==cma.Solution""" 
        # static classmethod, implicitly called before __init__
        # thistype is always Solution!?
        # necessary, because numpy.ndarray behaves in part like a
        # built-in class (what a hack)
        
        # initialize self from argument data
        if type(data) == thistype and copy is False:
            return data
        self = np.array(data, dtype=np.float_, copy=copy).view(thistype)
        
        # initialize attributes (moved to init)
        if 11 < 3:  # disregard class of data and type(data) is not Solution: 
            self.isgeno = geno  
            self.unrepaired = None
        return self  # calls _array_finalize__, if available
        
    #~ def __array_finalize__(self, obj):  # only called if type(obj) is Solution?
        #~ # Purpose: assign additional attributes from initializing obj (copy constructor)
        #~ # CAVE: attribute unrepaired holds the last unrepaired solution, 
        #~ #       regardlessly whether solution was changed. unrepaired cannot
        #~ #       not be updated (repair is not bijective), but could be removed
        #~ # This is removed, because it is not very efficient.
        #~ # the effect: any new solution is not isgeno and has no unrepaired version
        #~ if type(obj) is Solution:  # copy attributes
            #~ self.unrepaired = getattr(obj, 'unrepaired', None)
            #~ self.isgeno = getattr(obj, 'isgeno', None)
        
    def get_geno(self, gp):
        """return solution in "internal" representation.
        Argument gp -- GenoPheno class instance (subject to future changes)
        """

        if self.isgeno:
            return self
        # gp.geno copies data
        return Solution(gp.geno(self), copy=False, geno=True) 
        
    def get_pheno(self, gp):
        """return solution in original representation. 
            TODO: consider name change
        Argument gp -- GenoPheno class instance (subject to future changes)
        """

        if not self.isgeno:
            return self
        # gp.geno copies data
        return Solution(gp.pheno(self), copy=False, geno=False)
        
    def repair(self, bounds, keep_unrepaired=True):
        """sets out-of-bounds components of the represented solution 
        on the bounds.  
        :Arguments:
            bounds -- can be None or [lb, ub], where lb and ub
                can be None or a scalar or a list or array of length len(self)
                and represent lower and upper domain bounds respectively. 
            keep_unrepaired -- False prevents to make a copy of the original
                value
        """
        # TODO: CPU(N,lam,iter=20,200,100): 3.3s of 8s for two bounds, 1.8s of 6.5s for one bound
        if self.isgeno:
            raise _Error('genotyp cannot be repaired')
        if bounds in (None, [None, None]):  # solely for effiency 
            if 11 < 2: 
                # this produces a memory leak (supposably a numpy bug, reported by Daan Sprunken)
                self.unrepaired = self.view(np.ndarray)  
            else: 
                # TODO: this leaves the object possibly in an inconsistent state when the self value changes
                self.unrepaired = None
                if keep_unrepaired: 
                    self.unrepaired = array(self)  # make a copy
        else:
            self.unrepaired = None
            if keep_unrepaired: 
                self.unrepaired = array(self)  # make a copy
            if bounds[0] is not None:
                if np.isscalar(bounds[0]):
                    for i in xrange(len(self)):
                        self[i] = max(bounds[0], self[i])
                else:
                    for i in xrange(len(self)):
                        self[i] = max(bounds[0][i], self[i])
            if bounds[1] is not None:
                if np.isscalar(bounds[1]):
                    for i in xrange(len(self)):
                        self[i] = min(bounds[1], self[i])
                else:
                    for i in xrange(len(self)):
                        self[i] = min(bounds[1][i], self[i])
            # gp.into_bounds(self, copy_never=True)  # don't copy, because of side effect
        return self  # convenience return

#____________________________________________________________
#____________________________________________________________
#
class BoundPenalty(object):
    """Computes the boundary penalty. Must be updated each iteration, 
    using the update method. 
    
    :Details: the penalty computes like sum(w[i] * (x[i]-xfeas[i])**2), 
        where xfeas is the closest feasible (in-bounds) solution from x.
        The weight w[i] should be updated during each iteration using
        the update method.  
        This class uses GenoPheno to access the domain boundary values, 
        but this might change in future.  
    """
    
    #____________________________________________________________
    #
    def __init__(self, bounds): 
        """:Arguments: 
            bounds -- can be None or bounds[0] and bounds[1] are lower 
                and upper domain boundaries, each is either None or a scalar
                or a list or array of appropriate size.
        """
        ##
        # bounds attribute reminds the domain boundary values
        self.bounds = None  
        
        if bounds is not None:  # make the update stand-alone
            self.bounds = bounds
        self.gamma = 1  # a very crude assumption
        self.weights_initialized = False  # gamma becomes a vector after initialization
        self.hist = []  # delta-f history
        
    #____________________________________________________________
    #
    def __call__(self, x, bounds=None):
        """returns the boundary violation penalty for x ,where x is a 
        single solution or a list or array of solutions. 
        If bounds is not None, the values in bounds are used, see __init__"""
        if x in (None, (), []):
            return x
        if bounds is None:
            bounds = self.bounds
        if bounds is None or bounds == [None, None]:
            return 0.0 if np.isscalar(x[0]) else [0.0] * len(x) # no penalty
            
        x = [x] if np.isscalar(x[0]) else x
        
        pen = []
        for xi in x:   
            if type(xi) is not Solution or xi.unrepaired is None:
                # CAVE: this does not work with already repaired values!!
                # CPU(N,lam,iter=20,200,100)?: 3s of 10s, array(xi): 1s (check again)
                # remark: one deep copy can be prevented by xold = xi first
                xi = Solution(xi, copy=True).repair(bounds, keep_unrepaired=True) 
            fac = 1  # exp(0.1 * (log(self.scal) - np.mean(self.scal)))
            pen.append(sum(self.gamma * ((xi - xi.unrepaired) / fac)**2) / len(xi)) 
            
        return pen if len(pen) > 1 else pen[0]
        
    #____________________________________________________________
    #
    def feasible_ratio(self, solutions):
        """counts for each coordinate the number of feasible values in
        solutions and returns an array of length len(solutions[0])
        with the ratios. 
        
        :Arguments:
            solutions -- a list or array of repaired Solution instances 
        """
        count = np.zeros(len(solutions[0]))
        for x in solutions:
            count += x.unrepaired == x
        return count / float(len(solutions))

    #____________________________________________________________
    #
    def update(self, function_values, es, bounds=None):
        """updates the weights for computing a boundary penalty
        :Arguments:
            function_values -- all function values of recent population 
                of solutions
            es -- CMAEvolutionStrategy object instance
        :Reference: Hansen et al 2009, A Method for Handling Uncertainty... 
            IEEE TEC, with addendum at http://www.lri.fr/~hansen/TEC2009online.pdf
        """
        if bounds is None:
            bounds = self.bounds 
        if bounds is None:  # no bounds ==> no penalty
            return len(function_values) * [0.0]  # case without voilations
        
        N = es.N
        ### prepare
        # compute vars = sigma**2 * C_ii
        vars = es.sigma**2 * array(N * [es.C] if np.isscalar(es.C) else (  # scalar case
                                es.C if np.isscalar(es.C[0]) else  # diagonal matrix case
                                [es.C[i][i] for i in xrange(N)]))  # full matrix case
        dmean = (es.mean - es.gp.into_bounds(es.mean)) / vars**0.5

        ### Store/update a history of delta fitness value
        fvals = sorted(function_values)
        l = 1 + len(fvals)
        val = fvals[3*l // 4] - fvals[l // 4] # exact interquartile range apart interpolation
        val = val / np.mean(vars)  # new: val is normalized with sigma of the same iteration
        # insert val in history 
        if np.isfinite(val) and val > 0:
            self.hist.insert(0, val)
        elif val == inf and len(self.hist > 1):
            self.hist.insert(0, max(self.hist))
        else:
            pass  # ignore 0 or nan values
        if len(self.hist) > 20 + (3*N) / es.popsize:
            self.hist.pop()
        
        ### prepare
        dfit = np.median(self.hist)  # median interquartile range
        damp = min(1, es.sp.mueff/10./N)

        ### set/update weights
        # Throw initialization error
        if len(self.hist) == 0:
            raise _Error('wrongful initialization, no feasible solution sampled')
        # initialize weights
        if (dmean.any() and (not self.weights_initialized or es.countiter == 2)):  # TODO
            self.gamma = array(N * [2*dfit])
            self.weights_initialized = True
        # update weights gamma
        if self.weights_initialized:
            edist = array(abs(dmean) - 3 * max(1, N**0.5/es.sp.mueff))
            if 1 < 3:  # this is better, around a factor of two  
                # increase single weights possibly with a faster rate than they can decrease 
                #     value unit of edst is std dev, 3==random walk of 9 steps
                self.gamma *= exp((edist>0) * np.tanh(edist/3) / 2.)**damp 
                # decrease all weights up to the same level to avoid single extremely small weights
                #    use a constant factor for pseudo-keeping invariance
                self.gamma[self.gamma > 5 * dfit] *= exp(-1./3)**damp
                #     self.gamma[idx] *= exp(5*dfit/self.gamma[idx] - 1)**(damp/3)
            elif 1 < 3 and (edist>0).any():  # previous method
                # CAVE: min was max in TEC 2009
                self.gamma[edist>0] *= 1.1**min(1, es.sp.mueff/10./N)
                # max fails on cigtab(N=12,bounds=[0.1,None]): 
                # self.gamma[edist>0] *= 1.1**max(1, es.sp.mueff/10./N) # this was a bug!?
                # self.gamma *= exp((edist>0) * np.tanh(edist))**min(1, es.sp.mueff/10./N) 
            else:  # alternative version, but not better
                r = self.feasible_ratio(solutions)  # has to be the averaged over N iterations
                self.gamma *= exp(np.max([N*[0], 0.3 - r], axis=0))**min(1, es.sp.mueff/10/N)

        ### return penalty
        # es.more_to_write = self.gamma if not np.isscalar(self.gamma) else N*[1]
        return self  # bound penalty values

#____________________________________________________________
#____________________________________________________________
#   
class GenoPheno(object):
    """Genotype-phenotype transformation for convenient scaling.
    
    Can be extended to any on-the-fly transformation of the problem.
    Check pheno() and geno() for that matter. 
     
    """

    def __init__(self, dim, scaling=None, typical_x=None, bounds=None):
        """:Arguments:
            scaling -- the diagonal of a scaling transformation matrix
            bounds -- (obsolete, might disappear) list with two elements, 
                lower and upper bounds both can be a scalar or a "vector" 
                of length dim or None
        """
        self.N = dim
        self.bounds = bounds
        if bounds:
            if len(bounds) != 2:
                raise _Error('len(bounds) must be 2 for lower and upper bounds')
            for i in (0,1):
                if bounds[i] is not None:
                    bounds[i] = array(dim * [bounds[i]] if np.isscalar(bounds[i]) else 
                                        [b for b in bounds[i]])

        if np.size(scaling) > 1 or scaling:  # 
            self.scales = scaling  # CAVE: is not a copy
        else:
            self.scales = 1

        if np.size(typical_x) > 1 or typical_x:
            self.typical_x = typical_x
        else:
            self.typical_x = 0
            
    def into_bounds(self, y, bounds=None, copy_never=False):
        """:Arguments:
            y -- phenotypic vector
        :Returns:
            y put into boundaries, is a copy iff y != into_bounds(y)
            
        :Note: this code is duplicated in Solution.repair and might
            disappear in future. 
        """
        bounds = bounds if bounds is not None else self.bounds
        if bounds in (None, [None, None]):
            return y
        if bounds[0] is not None:
            if copy_never:  # is rather slower
                for i in xrange(len(y)):
                    y[i] = max(bounds[0][i], y[i])
            else:
                y = np.max([bounds[0], y], axis=0)
        if bounds[1] is not None:
            if copy_never:
                for i in xrange(len(y)):
                    y[i] = min(bounds[1][i], y[i])
            else:
                y = np.min([bounds[1], y], axis=0)
        return y
        
    def pheno(self, x, copy=True, bounds=None):
        """ maps the genotypic input argument into the phenotypic space """
        y = array(x, copy=copy)  # make a copy, in case
        if self.scales != 1:  # just for efficiency
            y *= self.scales
            # print 'transformed to phenotyp'
        if self.typical_x != 0:
            y += self.typical_x

        # y = self.pheno_nonlinear(y)  # inverse of geno_nonlinear

        if bounds is not None:
            y = self.into_bounds(y, bounds)
        return y
        
    def geno(self, y, bounds=None):
        """ maps the phenotypic input argument into the genotypic space """
        x = array(y)  # makes always a copy

        # bounds = self.bounds if bounds is None else bounds
        if bounds is not None:  # map phenotyp into bounds first
            x = self.into_bounds(array(y), bounds)
        
        # x = self.geno_nonlinear(x)  # maybe either this or geno_affine_linear
        
        # affine-linear transformation
        if self.typical_x != 0:
            x = x - self.typical_x
        if self.scales != 1:  # just for efficiency
            x = x / self.scales  # for each element in y
            
        return x

#____________________________________________________________
#____________________________________________________________
# 
class HelperFunctions(object):
    """static convenience functions
    if the name is preceded with an "a", an numpy array is returned"""
    @staticmethod
    def apos(x, lower=0):
        """clips argument (scalar or array) from below at lower"""
        if lower == 0:
            return (x > 0) * x
        else: 
            return lower + (x > lower) * (x - lower)
#____________________________________________________________
#____________________________________________________________
# 
class Rotation(object):
    """Rotation class that implements an orthogonal linear transformation, 
    one for each dimension. Used to implement non-separable test functions. 

    :Example:
       >>> import numpy, cma
       >>> R = cma.Rotation()
       >>> R2 = cma.Rotation() # another rotation
       >>> x = numpy.array((1,2,3))
       >>> print R(R(x), inverse=1)
       [ 1.  2.  3.]
       
    """
    dicMatrices = {} 
    def __init__(self):
        self.dicMatrices = {} # otherwise there might be shared bases which is
                              # probably not what we want 
    def __call__(self, x, inverse=False): # function when calling an object
        """Rotates the input array x with a fixed rotation matrix
           (dicMatrices['str(len(x))'])
        """
        N = x.shape[0]  # can be an array or matrix, TODO: accept also a list of arrays?
        if not self.dicMatrices.has_key(str(N)): # create new N-basis for once and all
            B = np.random.randn(N, N)
            for i in xrange(N):
                for j in xrange(0, i):
                    B[i] -= np.dot(B[i], B[j]) * B[j]
                B[i] /= sum(B[i]**2)**0.5
            self.dicMatrices[str(N)] = B
        if inverse: 
            return np.dot(self.dicMatrices[str(N)].T, x)  # compute rotation
        else: 
            return np.dot(self.dicMatrices[str(N)], x)  # compute rotation

# Use rotate(x) to rotate x 
rotate = Rotation()  # should not be capitalized? 

#____________________________________________________________
#____________________________________________________________
# 
class FitnessFunctions(object):
    """ versatile container for test objective functions """

    def __init__(self):
        self.counter = 0  # number of calls or any other practical use
    
    def rot(self, x, fun, rot=1, args=()):
        """returns fun(rotation(x), *args), ie. fun applied to a rotated argument"""
        if len(np.shape(array(x))) > 1:  # parallelized
            res = []
            for x in x:
                res.append(self.rot(x, fun, rot, args))
            return res
    
        if rot:
            return fun(rotate(x, *args))
        else:
            return fun(x)
       
    def rand(self, x): 
        """Random test objective function"""
        return np.random.random(1)[0]
    
    def linear(self, x):
        return -x[0]
    
    def sphere(self, x):
        """Sphere (squared norm) test objective function"""
        # return np.random.rand(1)[0]**0 * sum(x**2) + 1 * np.random.rand(1)[0]
        return sum((x+0)**2) 

    def sectorsphere(self, x):
        """asymmetric Sphere (squared norm) test objective function"""
        return sum(x**2) + (1e6-1) * sum(x[x,0]**2)
    
    def cornersphere(self, x):
        """Sphere (squared norm) test objective function"""
        if any(x < 0):
            return np.NaN
        return sum(x**2)

    def normalSkew(self, f):
        N = np.random.randn(1)[0]**2
        if N < 1:
            N = f * N  # diminish blow up lower part
        return N
    
    def noiseC(self, x, func=sphere, fac=10, expon=0.8):
        f = func(self, x)
        N = np.random.randn(1)[0]/np.random.randn(1)[0]
        return max(1e-19, f + (float(fac)/len(x)) * f**expon * N)
    
    def noise(self, x, func=sphere, fac=10, expon=1):
        f = func(self, x)
        R = np.random.randn(1)[0]
        R = np.log10(f) + expon * abs(10-np.log10(f)) * np.random.rand(1)[0]
        # sig = float(fac)/float(len(x))
        # R = log(f) + 0.5*log(f) * random.randn(1)[0] 
        # return max(1e-19, f + sig * (f**np.log10(f)) * np.exp(R)) 
        # return max(1e-19, f * np.exp(sig * N / f**expon)) 
        # return max(1e-19, f * normalSkew(f**expon)**sig)
        return f + 10**R  # == f + f**(1+0.5*RN)
    
    def cigar(self, x, rot=0):
        """Cigar test objective function"""
        if rot:
            x = rotate(x)  
        x = [x] if np.isscalar(x[0]) else x  # scalar into list
        f = [x[0]**2 + 1e6 * sum(x[1:]**2) for x in x]
        return f if len(f) > 1 else f[0]  # 1-element-list into scalar
        
    def tablet(self, x, rot=0):
        """Tablet test objective function"""
        if rot:
            x = rotate(x)  
        x = [x] if np.isscalar(x[0]) else x  # scalar into list
        f = [1e6*x[0]**2 + sum(x[1:]**2) for x in x]
        return f if len(f) > 1 else f[0]  # 1-element-list into scalar

    def cigtab(self, y):
        """Cigtab test objective function"""
        X = [y] if np.isscalar(y[0]) else y
        f = [1e-4 * x[0]**2 + 1e4 * x[1]**2 + sum(x[2:]**2) for x in X]
        return f if len(f) > 1 else f[0]
        
    def twoaxes(self, y):
        """Cigtab test objective function"""
        X = [y] if np.isscalar(y[0]) else y
        N2 = len(X[0]) // 2
        f = [1e6 * sum(x[0:N2]**2) + sum(x[N2:]**2) for x in X]
        return f if len(f) > 1 else f[0]
        
    def elli(self, x, rot=0, xoffset=0, all=False):
        """Ellipsoid test objective function"""
        if not np.isscalar(x[0]):  # parallel evaluation
            return [self.elli(xi, rot) for xi in x]  # could save 20% overall
        if rot:
            x = rotate(x)  
        N = len(x)
        ftrue = sum((10**(3.*np.arange(N)/(N-1.))*(x+xoffset))**2)

        alpha = 0.49 + 1./N 
        beta = 1      
        felli = np.random.rand(1)[0]**beta * ftrue * \
                max(1, (10.**9 / (ftrue+1e-99))**(alpha*np.random.rand(1)[0]))
        # felli = ftrue + 1*np.random.randn(1)[0] / (1e-30 +
        #                                           np.abs(np.random.randn(1)[0]))**0
        if all:
            return (felli, ftrue)
        else:
            return ftrue
            # return felli
        
    def rosen(self, x):  
        """Rosenbrock test objective function"""
        x = [x] if np.isscalar(x[0]) else x  # scalar into list
        f = [sum(100.*(x[:-1]**2-x[1:])**2 + (1.-x[:-1])**2) for x in x]
        return f if len(f) > 1 else f[0]  # 1-element-list into scalar
        
    def diffpow(self, x, rot=0):
        """Diffpow test objective function"""
        N = len(x)
        if rot:
            x = rotate(x)  
        return sum(np.abs(x)**(2.+4.*np.arange(N)/(N-1.)))**0.5
        
    def ridge(self, x, expo=2):
        x = [x] if np.isscalar(x[0]) else x  # scalar into list
        f = [x[0] + 100*np.sum(x[1:]**2)**(expo/2.) for x in x]
        return f if len(f) > 1 else f[0]  # 1-element-list into scalar
        
    def flat(self,x):
        return 1 
        return 1 if np.random.rand(1) < 0.9 else 1.1
        return np.random.randint(1,30)
    def griewank(self, x):
        # was in [-600 600]
        x = (600./5) * x
        return 1 - np.prod(np.cos(x/sqrt(1.+np.arange(len(x))))) + sum(x**2)/4e3

    def rastrigin(self, x):
        """Rastrigin test objective function"""
        if not np.isscalar(x[0]):
            N = len(x[0])
            return [10*N + sum(xi**2 - 10*np.cos(2*np.pi*xi)) for xi in x]
            # return 10*N + sum(x**2 - 10*np.cos(2*np.pi*x), axis=1)
        N = len(x)
        return 10*N + sum(x**2 - 10*np.cos(2*np.pi*x))
        
    def schwefelmult(self, x, pen_fac = 1e4):
        """multimodal Schwefel function with domain -500..500"""
        y = [x] if np.isscalar(x[0]) else x
        N = len(y[0]) 
        f = array([418.9829*N - 1.27275661e-5*N - sum(x * np.sin(np.abs(x)**0.5))
                + pen_fac * sum((abs(x) > 500) * (abs(x) - 500)**2) for x in y])
        return f if len(f) > 1 else f[0]
        
    def optprob(self, x):
        n = np.arange(len(x)) + 1
        f = n * x * (1-x)**(n-1)
        return sum(1-f)
    
    def lincon(self, x, theta=0.01):
        """ridge like linear function with one linear constraint"""
        if x[0] < 0:
            return np.NaN
        return theta * x[1] + x[0]
        
fcts = FitnessFunctions()
         
#____________________________________________________________
#____________________________________________________________
# 
def defaultOptions(): 
    """Returns options/optional parameters dictionary for fmin() and for 
    the class constructor CMAEvolutionStrategy. 
       
    :Details: 
         Returns and Options object that is a dictionary. 
         Values are the default input arguments to fmin. 
         Options() or Options().defaults() also return the
         default options.

    :Example: 
      >>> import cma 
      >>> opts = cma.defaultOptions()  # only a wrapper for cma.Options()

    For more information on supported options class Options
  
    :Note: options restarts and evalparallel are ignored for
        the class CMAEvolutionStrategy. 

    :See also: fmin(), Options
    """
    return Options()  # assembles the keyword-arguments in a dictionary    

#____________________________________________________________
#____________________________________________________________
# 
def fmin(func, x0, sigma0=None, args=()
    # the follow string arguments are evaluated, besides filename
    , CMA_diagonal='0*100*N/sqrt(popsize)  # nb of iterations with diagonal covariance matrix, True for always ' # TODO 4/ccov_separable?
    , CMA_eigenmethod='0  # 0=numpy-s eigh, -1=pygsl, otherwise cma.eig (slower)'
    , CMA_mu='None  # parents selection parameter, default is popsize//2'
    , CMA_on='True  # False or 0 for no adaptation of the covariance matrix'
    , CMA_rankmu='True  # False or 0 for omitting rank-mu update of covariance matrix'
    , CMA_rankmualpha='0.3  # factor of rank-mu update if mu=1, subject to removal, default might change to 0.0 '
    , CMA_teststds='None  # factors for non-isotropic initial distr. mainly for test purpose, see scaling_...'
    , bounds='[None, None]  # lower (=bounds[0]) and upper domain boundaries, each a single scalar or a list/vector'
    , evalparallel='False  # in fmin(): func is called with a list of solutions, might be removed' 
    , ftarget='-inf  # target function value, minimization'
    , incpopsize='2  # in fmin(): multiplier for increasing popsize before each restart'
    , maxfevals='inf  # maximum number of function evaluations'
    , maxiter='long(1e3*N**2/sqrt(popsize))  # maximum number of iterations'
    , minstd='0  # minimal std in any coordinate direction, cave interference with tol*'
    , noise_reevalfrac=' 0.0  # 0.05, not yet working'
    , noise_eps='1e-7  # perturbation factor for reevalution, not yet used'
    , popsize='4+int(3*log(N))  # population size, AKA lambda, number of new solution per iteration'
    , restarts='0  # in fmin(): number of restarts' 
    , scaling_of_variables='None  # scale for each variable, sigma0 is interpreted w.r.t. this scale, in that effective_sigma0 = sigma0*scaling. Internally the variables are divided by scaling_of_variables and sigma is unchanged, default is ones(N).'
    , seed='None  # random number seed'
    , termination_callback='None  # in fmin(): a function returning True for termination, called after each iteration step and could be abused for side effects.'
    , tolfacupx='1e3  # termination when step-size increases by tolfacupx (diverges). In this case the initial step-size was chosen far too small or the objective function unintentionally indicates better solutions far away from the initial solution x0. '
    , tolfun='1e-11  # termination criterion: tolerance in function value, quite useful'
    , tolfunhist='1e-12  # termination criterion: tolerance in function value history'
    , tolstagnation='int(100 * N**1.5 / popsize)  # termination if no improvement over tolstagnation iterations'
    , tolx='1e-11  # termination criterion: tolerance in x-changes'
    , typical_x = 'None  # used with scaling_of_variables'
    , updatecovwait = 'None  # number of iterations without distribution update, name is subject to future changes' # TODO: rename: iterwaitupdatedistribution?
    , verb_append = '0  # initial evaluation counter, if >0 append (do not overwrite) output files' 
    , verb_disp = '100  # verbosity: display console output every verb_disp iteration'
    , verb_filenameprefix = 'outcmaes  # output filenames prefix'
    , verb_log = '1  # verbosity: write data to files every verb_log iteration, writing can be time critical on fast to evaluate functions'
    , verb_plot = '0  # in fmin(): plotdata() is called every verb_plot iteration'
    , verb_time = 'True  # output timings on console'
     ): 
    """cma.fmin -- functional interface to the stochastic optimizer CMA-ES 
                   for non-convex function minimization

    :Calling Sequences:
        fmin([],[]) -- returns all optional arguments, that is,
            all keyword arguments to fmin with their default values 
            in a dictionary.
        fmin(func, x0, sigma0) -- minimizes func starting
            at x0 with standard deviation sigma0 (step-size)
        fmin(func, x0, sigma0, ftarget=1e-5) -- minimizes func 
            up to target function value 1e-5
        fmin(func, x0, sigma0, args=('f',), **options) -- minimizes 
            func called with an additional argument 'f'. options
            is a dictionary with additional keyword arguments, e.g.
            delivered by Options(). 
        fmin(func, x0, sigma0, **{'ftarget':1e-5, 'popsize':40}) -- 
            the same as fmin(func, x0, sigma0, ftarget=1e-5, popsize=40)
        fmin(func, esobj) -- uses CMAEvolutionStrategy object
            instance esobj to optimize func

    :Arguments:
        func -- function to be minimized. Called as
            func(x,*args). x is a one-dimensional numpy.ndarray. func
            can return numpy.NaN (only if evalparallel argument is False),
            which is interpreted as outright rejection of solution x
            and invokes an immediate resampling and (re-)evaluation
            of a new solution not counting as function evaluation
        x0 -- list or numpy.ndarray, initial guess of minimum solution
              OR cma.CMAEvolutionStrategy object instance. In this case
              sigma0 can be ommitted.
        sigma0 -- scalar, initial standard deviation in each coordinate.
            sigma0 should be about 1/3 of the search domain width where the
            optimum is to be expected. The variables in func should be
            scaled such that they presumably have similar sensitivity.
            See also option scaling_of_variables. 

    :Keyword Arguments:
        args=() -- additional arguments for func
        
        For further keyword arguments see also class Options. 
        
    :Returns: res = (xopt, fopt, evals,
                     dict('out', 'stopdict', 'opts', 'cma'))
        xopt -- best evaluated solution equals to res[3]['cma'].best[0]
        fopt -- respective function value, equals to res[3]['cma'].best[1]
        evals -- number of conducted objective function evaluation
        out -- dictionary with final mean solution point, best solution
            point of the last iteration...
        opts -- actually appied options
        cma -- the class CMAEvolutionStrategy 
                                                                        
    :Notes:
        This function is an interface to the class CMAEvolutionStrategy. The
        class can be used when full control over the iteration loop of the
        optimizer is desired. 

    :Example:
        The following example calls fmin optimizing the Rosenbrock function 
        in 10-D with initial solution 0.1 and initial step-size 0.5. The
        options are specified for the usage with the doctest module.
        
        >>> import cma
        >>> options = {'CMA_diagonal':10, 'seed':1234, 'verb_time':0}
        >>>
        >>> res = cma.fmin(cma.fcts.rosen, [0.1] * 10, 0.5, **options)
        (5_w,10)-CMA-ES (mu_w=3.2,w_1=45%) in dimension 10 (seed=1234)
           Covariance matrix is diagonal for 10 iterations (1/ccov=29.0)
        Iterat #Fevals   function value     axis ratio  sigma   minstd maxstd min:sec
            1      10 1.264232686260072e+02 1.1e+00 4.40e-01  4e-01  4e-01 
            2      20 1.023929748193649e+02 1.1e+00 4.00e-01  4e-01  4e-01 
            3      30 1.214724267489674e+02 1.2e+00 3.70e-01  3e-01  4e-01 
          100    1000 6.366683525319511e+00 6.2e+00 2.49e-02  9e-03  3e-02 
          200    2000 3.347312410388666e+00 1.2e+01 4.52e-02  8e-03  4e-02 
          300    3000 1.027509686232270e+00 1.3e+01 2.85e-02  5e-03  2e-02 
          400    4000 1.279649321170636e-01 2.3e+01 3.53e-02  3e-03  3e-02 
          500    5000 4.302636076186532e-04 4.6e+01 4.78e-03  3e-04  5e-03 
          600    6000 6.943669235595049e-11 5.1e+01 5.41e-06  1e-07  4e-06 
          650    6500 5.557961334063003e-14 5.4e+01 1.88e-07  4e-09  1e-07 
        termination on tolfun : 1e-11
        final/bestever f-value = 5.55796133406e-14 2.62435631419e-14
        mean solution:  [ 1.          1.00000001  1.          1.          1.          1.00000001
          1.00000002  1.00000003] ...
        std deviation: [  3.91933872e-09   3.77927324e-09   4.00622859e-09   4.66059254e-09
           5.49661885e-09   7.43777452e-09   1.37972077e-08   2.60207658e-08] ...
        >>> 
        >>> print 'current solutions fitness f(', res[0], ') =', res[1] 
        current solutions fitness f( [ 1.  1.  1.  1.  1.  1.  1.  1.  1.  1.] ) = 2.62435631419e-14
        >>> 
        >>> print 'best solutions fitness =', res[3]['out']['best_f']
        best solutions fitness = 2.62435631419e-14
        
        The method 
            
            cma.plotdata();  
            
        (based on matplotlib.pylab) produces a plot of the run and
            
            cma.show()  
        
        shows the plot in a window. To continue you might need to 
        close the pop-up window. This behavior seems to disappear in 
        subsequent calls of cma.plotdata() and is avoided by using
        ipython with -pylab option. Finally
        
            cma.savefig('myfirstrun')  # savefig from matplotlib.pylab
        
        will save the figure in a png.

    :See also: CMAEvolutionStrategy, plotdata(), Options, scipy.optimize() 
    
    """ # style guides say there should be the above empty line

    try: # pass on KeyboardInterrupt
        opts = locals()  # collect all local variables (i.e. arguments) in a dictionary
        del opts['func'] # remove those without a default value 
        del opts['args']
        del opts['x0']      # is not optional, no default available
        del opts['sigma0']  # is not optional for the constructor CMAEvolutionStrategy
        if not func:  # return available options in a dictionary
            return Options(opts, True)  # these opts are by definition valid

        irun = 0
        # TODO: this is very ugly:
        incpopsize = Options({'incpopsize':incpopsize}).eval('incpopsize')  # evaled value
        restarts = Options({'restarts':restarts}).eval('restarts') 
        while 1:
            # recover from a CMA object
            if irun == 0 and isinstance(x0, CMAEvolutionStrategy):
                es = x0
                x0 = es.inputargs['x0']  # for the next restarts
                if sigma0 is None or not np.isscalar(array(sigma0)): 
                    sigma0 = es.inputargs['sigma0']  # for the next restarts
                # ignore further input args and keep original options
            else:  # default case
                es = CMAEvolutionStrategy(x0, sigma0, opts) 

            opts = es.opts  # processed ones, unambiguous
            if irun > 0:
                es.updateBest(x_prev, f_prev, ev_prev - es.countevals)
                
            while not es.stop:
                if opts['evalparallel']: 
                    X = es.ask()  # get list of new solutions
                    fit = func(X, *args)  # without copy
                else:
                    X, fit = es.ask_and_eval(func, args) # treats NaN with resampling

                es.tell(X, fit)  # prepare for next iteration
                
                es.printline()
                if es.opts['verb_log'] and es.opts['verb_plot'] and \
                    (es.countiter % max(es.opts['verb_plot'], es.opts['verb_log']) == 0 or es.stop):
                    plotdata(324)

                if termination_callback and termination_callback != str(termination_callback): 
                    es.callbackstop = termination_callback(es)

            # end while not es.stop

            mean_pheno = es.gp.pheno(es.mean, bounds=es.gp.bounds)
            if opts['evalparallel']:
                es.updateBest(mean_pheno, func((mean_pheno,), *args), 1)
            else:
                es.updateBest(mean_pheno, func(mean_pheno, *args), 1)
            es.countevals += 1

            # final message
            for k, v in es.stopdict.iteritems():
                print 'termination on', k, ':', v
            print 'final/bestever f-value =', es.out['recent_f'], es.out['best_f']
            if es.N < 9:
                print 'mean solution: ', es.gp.pheno(es.mean) 
                print 'std deviation:', es.sigma * sqrt(es.dC) * es.gp.scales
            else:
                print 'mean solution: ', es.gp.pheno(es.mean)[:8], '...' 
                print 'std deviation:', (es.sigma * sqrt(es.dC) * es.gp.scales)[:8], '...'

            irun += 1
            if irun > restarts or 'ftarget' in es._stopdict or 'maxfunevals' in es._stopdict:
                break
            x_prev = es.out['best_x']
            f_prev = es.out['best_f']
            ev_prev = es.out['best_evals']
            opts['verb_append'] = es.countevals
            opts['popsize'] = incpopsize * es.sp.popsize # TODO: use rather options? 
            opts['seed'] += 1

        # while irun

        return (es.out['best_x'].copy(), es.out['best_f'], es.countevals, 
                dict((('stopdict', es._stopdict.copy()) 
                      ,('mean', es.gp.pheno(es.mean)) 
                      ,('std', es.sigma * sqrt(es.dC) * es.gp.scales)
                      ,('out', es.out)
                      ,('opts', es.opts)  # last state of options 
                      ,('cma', es)
                      ,('inputargs', es.inputargs) 
                      ))
               )
                      # TODO refine output, can #args be flexible?
                      # is this well usable as it is now?
    except KeyboardInterrupt:  # Exception, e:
        if opts['verb_disp'] > 0:
            print ' outcomment last line of cma.fmin to restore KeyboardInterrupt exception e.g. for debugging'
        # raise  # in case we want the original behavior
        
# end fmin()

#____________________________________________________________
#____________________________________________________________
# 
class OOOptimizer(object):
    """base class for an oo optimizer interface, a stump so far"""
    def ask():
        pass
    def tell(solutions, function_values):
        pass
    def stop():
        pass
    # def solution():
    #     """return current favorite solution"""
    #     pass
    
#____________________________________________________________
#____________________________________________________________
# 
class CMAEvolutionStrategy(OOOptimizer):
    """CMA-ES stochastic optimizer class with ask-and-tell interface. 
    :Calling sequence:
        CMAEvolutionStrategy(x0, sigma0, opts)
        :Arguments:
            x0 -- initial solution, starting point.
            sigma0 -- initial standard deviation.  The problem
                variables should have been scaled, such that a single
                standard deviation on all variables is useful and the
                optimum is expected to lie within x0 +- 3*sigma0.
            opts -- options, a dictionary with optional settings, 
                see class Options. 
        :Returns: a class instance
    
    :Main interface / usage:
        The ask-and-tell interface is a generic OO interface for iterative
        optimization algorithms. In each iteration, 
            solutions = opt.ask() 
        is used to get new solutions and 
            opt.tell(solutions, func_values) 
        completes the iteration. Instead of ask(), here also 
            (solutions, func_values) = opt.ask_and_eval(func) 
        can be used. Besides for termination criteria, in CMA-ES only 
        the ranks of the func_values are relevant. 
    
    :Attributes:
        inputargs -- passed input arguments
        inopts -- passed options
        opts -- actually used options, some of them can be changed any 
            time, see class Options. 
        out -- a dictionary with useful output 
        stop -- bool, whether or not a termination criterion is satisfied
        stoplist, stopdict -- non-empty when a termination criterion is
            satisfied
    
    :Super-short example, with output shown:
        >>> import cma 
        >>> # construct an object instance in 4-D, sigma0=1
        >>> es = cma.CMAEvolutionStrategy(4 * [1], 1, {'seed':234})
        (4_w,8)-CMA-ES (mu_w=2.6,w_1=52%) in dimension 4 (seed=234)
        >>> 
        >>> # iterate until termination
        >>> while not es.stop: X = es.ask(); es.tell(X, [cma.fcts.elli(x) for x in X])
        >>>
        >>> cma.pprint((es.out['best_x'], es.out['best_f']))
        (Solution([ -5.84282220e-08,   2.98751778e-09,   2.49593562e-11,
                 1.72190341e-11]),
         4.6091082058308125e-15)
         
    :Example with increasing popsize (IPOP), output is not displayed:
        import cma

        options = {'popsize': 10}  # initial population size
        
        # restart with increasing population size (IPOP)
        while options['popsize'] < 500:  
            es = cma.CMAEvolutionStrategy('6 - 8 * np.random.rand(9)',  # 9-D
                                          5,         # initial std sigma0
                                          options)   # pass options 
            while not es.stop:
                X = es.ask()    # get list of new solutions
                fit = [cma.fcts.rastrigin(x) for x in X]  # evaluate each solution
                es.tell(X, fit) # besides for termination only the ranking in fit is used

                # display some output
                es.printline()  # uses option verb_disp with default 100

            print 'termination:', es.stopdict
            cma.pprint(es.best)
            
            # store the best-ever solution here? 

            # prepare next loop
            options['verb_append'] = es.countevals  # append data for plotting
            options['popsize'] *= 2  # increase popsize
            
            # show a plot    
            cma.plotdata()
            print '  *** if execution stalls close the figure window to continue ***'
            cma.show()  # see cma.plotdata() for details 
        
        On the Rastrigin function, usually after five restarts the global optimum 
        is located. 
        
    :Another example shows how to resume:
        import cma, pickle

        flg_resume = False  # set True to resume
        if not flg_resume:
            cma.Options().printme()  # let's see what options are available
            es = cma.CMAEvolutionStrategy(12 * [0.1],  # a new instance, 12-D
                                          0.5)         # initial std sigma0
        else:
            es = pickle.load(open('saved-cma-object.pkl')) 
             
        while not es.stop:
            X = es.ask()    # get list of new solutions
            fit = [cma.fcts.rosen(x) for x in X]  # evaluate each solution
            es.tell(X, fit) # besides for termination only the ranking in fit is used

            # display some output
            es.printline()  # uses option verb_disp with default 100
            
            # interrupt the loop: just for the demonstration of resume
            if es.countiter % 300 == 0:  
                # allows for later resume, comparatively time consuming 
                pickle.dump(es, open('saved-cma-object.pkl', 'w'))  
                print 'just for testing: loop discontinued'
                break  # just for testing purpose
                
        cma.pprint(es.out)
        cma.plotdata()
        cma.show()  # see cma.plotdata() for details 

    :See also: fmin(), plotdata(), ask(), tell(), ask_and_eval()
        
    """

    # __all__ = ()  # TODO this would be the interface

    #____________________________________________________________
    #____________________________________________________________
    @property  # read only attribute decorator for a method
    def stop(self):
        # this doc string is available via help cma.CMAEvolutionStrategy.stop
        """Termination status, either True of False. 
        See also stoplist, stopdict 
        """
        return self.stoplist != []

    #____________________________________________________________
    #____________________________________________________________
    @property  # read only attribute decorator for a method
    def stoplist(self):
        """List of satisfied termination criteria, empty if none
        """
        self._teststop()
        return self._stoplist[:] # returns a copy, such that it is not modified later

    #____________________________________________________________
    #____________________________________________________________
    @property  # read only attribute decorator for a method
    def stopdict(self):
        """Dictionary of satisfied termination criteria, empty if none
        """
        self._teststop()
        return self._stopdict.copy() 

    #____________________________________________________________
    #____________________________________________________________
      
    def _addstop(self, key, cond, val=None):
        if cond:
            self._stoplist.append(key)
            if key in self.opts.keys():
                val = self.opts[key]
            self._stopdict[key] = val
    
    #____________________________________________________________
    #____________________________________________________________
    def _teststop(self):
        """Test termination criteria and update respective lists
        see also stoplist and stopdict. 
        """
    #____________________________________________________________

        if self.countiter == self.stoplastiter:
            if self.countiter == 0:
                self._stoplist = []
                self._stopdict = {}
            return 

        self.stoplastiter = self.countiter

        # TODO: check all stoplist and dict
        self._stoplist = []
        self._stopdict = {}
        
        N = self.N
        opts = self.opts
        
        # fitness: generic criterion, user defined w/o default
        self._addstop('ftarget',
                     self.out['best_f'] < opts['ftarget'])
        # maxiter, maxfevals: generic criteria
        self._addstop('maxfevals',
                     self.countevals - 1 >= opts['maxfevals'])
        self._addstop('maxiter',
                     self.countiter >= opts['maxiter'])
        # tolx, tolfacupx: generic criteria
        # tolfun, tolfunhist (CEC:tolfun includes hist) 
        self._addstop('tolx',
                     all([self.sigma*xi < opts['tolx'] for xi in self.pc]) and \
                     all([self.sigma*xi < opts['tolx'] for xi in sqrt(self.dC)]))
        self._addstop('tolfacupx',
                     any([self.sigma * sig > self.sigma0 * opts['tolfacupx']
                          for sig in sqrt(self.dC)]))
        self._addstop('tolfun',
                     self.fit.fit[-1] - self.fit.fit[0] < opts['tolfun'] and \
                     max(self.fit.hist) - min(self.fit.hist) < opts['tolfun'])
        self._addstop('tolfunhist',
                     len(self.fit.hist) > 9 and \
                     max(self.fit.hist) - min(self.fit.hist) <  opts['tolfunhist'])

        # worst seen false positive: table N=80,lam=80, getting worse for fevals=35e3 \approx 50 * N**1.5 
        # but the median is not so much getting worse
        # / 5 reflects the sparsity of histbest/median
        # / 2 reflects the left and right part to be compared
        l = int(max(self.opts['tolstagnation'] / 5. / 2, len(self.fit.histbest) / 10));
        # TODO: the problem in the beginning is only with best ==> ???
        if 11 < 3:  # 
            print self.countiter, (self.opts['tolstagnation'], self.countiter > N * (5+100./self.popsize),  
                        len(self.fit.histbest) > 100,
                        np.median(self.fit.histmedian[:l]) >= np.median(self.fit.histmedian[l:2*l]),
                        np.median(self.fit.histbest[:l]) >= np.median(self.fit.histbest[l:2*l]))
        # equality should handle flat fitness
        self._addstop('tolstagnation', # leads sometimes early stop on ftablet, fcigtab, N>=50?
                    1 < 3 and self.opts['tolstagnation'] and self.countiter > N * (5+100./self.popsize) and  
                    len(self.fit.histbest) > 100 and 2*l < len(self.fit.histbest) and 
                    np.median(self.fit.histmedian[:l]) >= np.median(self.fit.histmedian[l:2*l]) and
                    np.median(self.fit.histbest[:l]) >= np.median(self.fit.histbest[l:2*l]))
        # iiinteger: stagnation termination can prevent to find the optimum

        if 11 < 3 and 2*l < len(self.fit.histbest):  # TODO: this might go wrong, because the nb of written columns changes
            tmp = np.array((-np.median(self.fit.histmedian[:l]) + np.median(self.fit.histmedian[l:2*l]),
                        -np.median(self.fit.histbest[:l]) + np.median(self.fit.histbest[l:2*l])))
            self.more_to_write = [(10**t if t < 0 else t + 1) for t in tmp] # the latter to get monotonicy

        # TODO: add option stoponequalfunvals and test here...

        if 1 < 3:
            # non-user defined, method specific
            # noeffectaxis (CEC: 0.1sigma), noeffectcoord (CEC:0.2sigma), conditioncov
            self._addstop('noeffectcoord',
                         any([self.mean[i] == self.mean[i] + 0.2*self.sigma*sqrt(self.dC[i])
                              for i in xrange(N)]))
            if opts['CMA_diagonal'] is not True and self.countiter > opts['CMA_diagonal']: 
                self._addstop('noeffectaxis',
                             all([self.mean[i] == self.mean[i] +
                                  0.1*self.sigma * self.D[self.countiter%N]
                                  * self.B[i,self.countiter%N] for i in xrange(N)]))
            self._addstop('conditioncov',
                         self.D[-1] > 1e7 * self.D[0], 1e14)  # TODO 

            self._addstop('callback',
                          self.callbackstop)  # termination_callback

        return self.stoplist != []

    #____________________________________________________________
    #____________________________________________________________
    @property
    def best(self):
        """Return (xbest, fbest) -- best solution so far evaluated together
              with its function value
        """

        return (self.out['best_x'], self.out['best_f']) 

    # @best.setter
    # def best(self, x, fun_value, evals=None):
    #     self.updateBest(x, fun_value, evals)
    #     return self.best
        
    #____________________________________________________________
    #____________________________________________________________
    @property  # read only attribute decorator for a method
    def popsize(self): # getter method for mean_x
        """number of samples per iteration returned by default ask() and
        expected by tell()
        """
        return self.sp.popsize
    
    @popsize.setter
    def popsize(self, p):
        """popsize cannot be set (this might change in future)
        """
        raise _Error("popsize cannot be changed (this might change in future)")        
        
    #____________________________________________________________
    #____________________________________________________________
    def ask(self, number=None, xmean=None, sigma_fac=1):
        """Get new candidate solutions, sampled from a multi-variate
           normal distribution. 
           
        :Arguments:
            number -- number of returned solutions, by default the
                population size popsize (AKA lambda).
            xmean -- distribution mean 
            sigma -- multiplier for internal sample width (standard
               deviation)
        :Returns:
            list of N-dimensional candidate solutions to be evaluated
        :Example:
            es = cma.CMAEvolutionStrategy([0,0,0,0], 0.3)
            while not es.stop and es.best[1] > 1e-6:  # my_desired_target_f_value
                X = es.ask()  # get list of new solutions
                fit = [cma.fcts.rosen(x) for x in X]  # call function rosen with each solution
                es.tell(X, fit)  # feed back fitness values
                
        """
        
        if self.countiter == 0:
            self.tic = time.clock()
        if number is None or number < 1:
            number = self.sp.popsize
        if xmean is None:
            xmean = self.mean  
        sigma = sigma_fac * self.sigma

        # update parameters for sampling the distribution
        #        fac  0      1      10
        # 150-D cigar: 
        #           50749  50464   50787
        # 200-D elli:               == 6.9
        #                  99900   101160
        #                 100995   103275 == 2% loss
        # 100-D elli:               == 6.9
        #                 363052   369325  < 2% loss
        #                 365075   365755

        # update distribution
        if self.sp.CMA_on and (
                (self.opts['updatecovwait'] is None and 
                 self.countiter >=
                     self.itereigenupdated + 1./(self.sp.c1+self.sp.cmu)/self.N/10
                 ) or
                (self.opts['updatecovwait'] is not None and
                 self.countiter > self.itereigenupdated + self.opts['updatecovwait']
                 )):
            self.updateBD()

        # sample distribution
        if self.flgtelldone:  # could be done in tell()!? 
            self.flgtelldone = False
            self.ary = [] 

        if True or self.sp.mu > 1:  # use always independent sampling
            # each row is a solution
            self.ary = np.dot(self.B, (self.D * np.random.standard_normal((number, self.N))).T).T
            pop = xmean + sigma * self.ary
            # N,lambda=20,200: overall CPU 7s vs 5s == 40% overhead, even without bounds!
            pop = [Solution(self.gp.pheno(x, copy=False), copy=False).repair(self.gp.bounds) for x in pop]  

        else: # use mirrors for mu=1 see Brockhoff et al PPSN 2010, needs to be tested  
            # append number candidate solutions   
            pop = []
            for k in xrange(number):
                if np.mod(len(self.ary),2) == 1:  # even individual
                    self.ary.append(-self.ary[-1])
                else: # regular sampling
                    self.ary.append(np.dot(self.B, self.D*np.random.randn(self.N)))
                pop.append(Solution(self.gp.pheno(xmean + sigma * self.ary[-1], copy=False), 
                                    copy=False).repair(self.gp.bounds))
        return pop

    #____________________________________________________________
    #____________________________________________________________
    # 
    def updateBD(self): 
        """ update internal variables to sampling the distribution with 
        covariance matrix C. This method is O(N^3), given C is not diagonal."""
        # itereigenupdated is always up-to-date in the diagonal case 
        # just double check here
        if self.itereigenupdated == self.countiter:
            return

        self.C = 0.5 * (self.C + self.C.T)
        # self.C = np.triu(self.C) + np.triu(self.C,1).T  # should work as well
        # self.D, self.B = eigh(self.C) # hermitian, ie symmetric C is assumed
        if self.opts['CMA_eigenmethod'] == -1:  
                   # pygsl
                   # easy to install (well, in Windows install gsl binaries first, 
                   # set system path to respective libgsl-0.dll (or cp the dll to
                   # python\DLLS ?), in unzipped pygsl edit
                   # gsl_dist/gsl_site_example.py into gsl_dist/gsl_site.py
                   # and run "python setup.py build" and "python setup.py install"
                   # in MINGW32)
            if 1 < 3:  # import pygsl on the fly
                try:
                    from pygsl.eigen import eigenvectors  # TODO efficient enough?
                except ImportError:
                    print 'WARNING: could not find pygsl.eigen module, either install pygsl \n' + \
                          '  or set option CMA_eigenmethod=1 (is much slower), option set to 1'
                    self.opts['CMA_eigenmethod'] = 0  # use 0 if 1 is too slow
                    
                if self.opts['CMA_eigenmethod'] == -1:
                    self.D, self.B = eigenvectors(self.C)

            else:  # assumes pygsl.eigen was imported above 
                self.D, self.B = pygsl.eigen.eigenvectors(self.C)

            idx = np.argsort(self.D)
            self.D = self.D[idx]
            self.B = self.B[:,idx] # self.B[i] is the i+1-th row and not an eigenvector
            
        elif self.opts['CMA_eigenmethod'] == 0: 
            # TODO: thoroughly test np.linalg.eigh 
            #       numpy.linalg.eig crashes in 200-D
            #       and EVecs with same EVals are not orthogonal 
            self.D, self.B = np.linalg.eigh(self.C)  # self.B[i] is a row and not an eigenvector
                                                     # self.B[:,i] is an eigenvector
            idx = np.argsort(self.D)
            self.D = self.D[idx]
            self.B = self.B[:,idx]
        else:  # is overall two;ten times slower in 10;20-D
            self.D, self.B = eig(self.C)  # def eig, see below
            idx = np.argsort(self.D)
            self.D = self.D[idx]
            self.B = self.B[:,idx]
            # assert(sum(self.D-DD) < 1e-6)
            # assert(sum(sum(np.dot(BB, BB.T)-np.eye(self.N))) < 1e-6)
            # assert(sum(sum(np.dot(BB * DD, BB.T) - self.C)) < 1e-6)
        
        # assert(all(self.B[self.countiter % self.N] == self.B[self.countiter % self.N,:]))
            
        if 11 < 3 and any(abs(sum(self.B[:,0:self.N-1] * self.B[:,1:], 0)) > 1e-6): 
            print 'B is not orthogonal'
            print self.D
            print sum(self.B[:,0:self.N-1] * self.B[:,1:], 0) 
        else:
            # TODO remove as it is O(N^3)
            # assert(sum(abs(self.C - np.dot(self.D * self.B,  self.B.T))) < N**2*1e-11) 
            pass
        self.D **= 0.5
        self.itereigenupdated = self.countiter

    #____________________________________________________________
    #____________________________________________________________
    # 
    def _updateCholesky(self, A, Ainv, p, alpha, beta): 
        """not yet implemented"""
        # BD is A, p is A*Normal(0,I) distributed
        # input is assumed to be numpy arrays
        # Ainv is needed to compute the evolution path
        # this might be a stump and is not tested

        # prepare
        alpha = float(alpha)
        beta = float(beta)
        y = np.dot(Ainv, p)
        y_sum = sum(y**2)

        # compute scalars
        tmp = sqrt(1 + beta * y_sum / alpha)
        fac = (sqrt(alpha) / sum(y**2)) * (tmp - 1)
        facinv = (1. / (sqrt(alpha) * sum(y**2))) * (1 - 1. / tmp)

        # update matrices
        A *= sqrt(alpha)
        A += np.outer(fac * p, y)
        Ainv /= sqrt(alpha)
        Ainv -= np.outer(facinv * y, np.dot(y.T, Ainv))

    #____________________________________________________________

    #____________________________________________________________
    # 
    def ask_and_eval(self, func, args=(), number=None, xmean=None, sigma=1):
        """Samples number solutions and evaluates them on func.
        Each solution s is resampled until func(s) not in (numpy.NaN, None).  
            
        :Arguments:
            func -- objective function
            args -- additional parameters for func
            number -- number of solutions to be sampled, by default
                population size popsize (AKA lambda)
            xmean -- list(!) of means for sampling the solutions,
                 last entry is recycled, use (x,) to pass a single vector
            sigma -- multiplier for sampling width, standard deviation
        :Returns: (X, fit)
            X -- list of solutions
            fit -- list of respective function values
        :Details:
            When func(x) returns NaN or None a new solution is sampled until
            func(x) not in (numpy.NaN, None).  The argument to func can be 
            freely modified within func.  
        :Example:
            import cma
            x0, sigma0 = 8*[10], 1  # 8-D
            es = cma.CMAEvolutionStrategy(x0, sigma0) 
            while not es.stop:
                X, fit = es.ask_and_eval(cma.fcts.elli)  # treats NaN with resampling
                es.tell(X, fit)  # pass on fitness values
                es.printline(10) # print every 10-th iteration
            print 'terminated on', es.stopdict
            
        """
        popsize = self.sp.popsize  
        if number is not None:
            popsize = number
        if xmean is None:
            xmean = (self.mean,)
        if np.size(np.shape(xmean)) < 2:
            raise _Error(' xmean argument must be a sequence, e.g. (my_mean, )')
        fit = []  # or np.NaN * np.empty(number)
        X = []
        for k in xrange(int(popsize)):
            nreject = -1
            f = np.NaN
            while f in (np.NaN, None):
                nreject += 1
                x = self.ask(1, xmean[min(k,len(xmean)-1)], sigma)[0] # get one solution vector
                f = func(x, *args)         # call func on solutions
                if nreject+1 % 1000 == 0:
                    print ' ', nreject, 'solutions rejected (f-value NaN) at iteration',
                    print self.countiter
            fit.append(f)
            X.append(x)
        return X, fit

    #____________________________________________________________
    #____________________________________________________________
    def readProperties(self):
        """reads dynamic parameters from property file (not implemented)
        """
        print 'not yet implemented'

    #____________________________________________________________
    #____________________________________________________________
    #____________________________________________________________
    #____________________________________________________________
    def __init__(self, x0, sigma0, inopts = {}):
        """see class CMAEvolutionStrategy. 
        """

        #____________________________________________________________
        self.inputargs = dict(locals()) # for the record
        del self.inputargs['self'] # otherwise the instance self has a cyclic reference
        self.inopts = inopts
        opts = Options(inopts).complement()  # Options() == fmin([],[]) == defaultOptions()
        if x0 == str(x0):
            x0 = eval(x0)
        self.x0 = x0
        self.mean = array(x0, copy=True)  # does not have column or row, is just 1-D
        if self.mean.ndim != 1:
            raise _Error('x0 must be 1-D array')
        
        self.N = self.mean.shape[0]
        N = self.N
        self.mean.resize(N) # 1-D array, not really necessary?!
        
        self.sigma0 = sigma0
        if isinstance(sigma0, str):  # TODO: no real need here (do rather in fmin)
            self.sigma0 = eval(sigma0)  # like '1./N' or 'np.random.rand(1)[0]+1e-2'
        if np.size(self.sigma0) != 1 or np.shape(self.sigma0): 
            raise _Error('input argument sigma0 must be (or evaluate to) a scalar')
        self.sigma = self.sigma0
        
        # extract/expand options 
        opts.evalall(locals())  # using only N
        self.opts = opts

        self.gp = GenoPheno(N, opts['scaling_of_variables'], opts['typical_x'], opts['bounds']) 
        self.boundPenalty = BoundPenalty(self.gp.bounds)
        s = self.gp.geno(self.mean)
        self.mean = self.gp.geno(self.mean, bounds=self.gp.bounds)
        if (self.mean != s).any():
            print 'WARNING: initial solution is out of the domain boundaries:'
            print '  x0   =', self.inputargs['x0']
            print '  ldom =', self.gp.bounds[0] 
            print '  udom =', self.gp.bounds[1] 
        self.fmean = np.NaN             # TODO name should change? prints nan (OK with matlab&octave)
        self.fmean_noise_free = 0.  # for output only
        
        self.sp = Parameters(N, opts)
        self.sp0 = self.sp

        # initialization of state variables
        self.countiter = 0
        self.countevals = 0
        self.ps = np.zeros(N)
        self.pc = np.zeros(N)

        stds = np.ones(N)
        if self.opts['CMA_teststds']:  # also 0 would not make sense
            stds = self.opts['CMA_teststds']
            if np.size(stds) != N:
                raise _Error('CMA_teststds option must have dimension = ' + str(N)) 
        if self.opts['CMA_diagonal']:  # is True or > 0
            # linear time and space complexity 
            self.B = array(1) # works fine with np.dot(self.B, anything) and self.B.T
            self.C = stds**2
            self.dC = self.C
        else:
            self.B = np.eye(N) # identity(N), do not from matlib import *, as eye is a matrix there
            # prevent equal eigenvals, a hack for np.linalg TODO:
            # self.C = np.diag(1 + 0.01*np.random.rand(N)) 
            self.C = np.diag(stds**2)
            self.dC = np.diag(self.C)
        self.D = stds

        self.flgtelldone = True
        self.itereigenupdated = self.countiter
        self.noiseS = 0  # noise "signal"
        self.hsiglist = [] 

        if not opts['seed']:
            np.random.seed()
            opts['seed'] = 1e9 * np.random.rand()
        opts['seed'] = int(opts['seed'])
        np.random.seed(opts['seed'])
        
        out = {}
        # out.best = DictClass()
        out['best_f'] = np.inf
        out['best_x'] = []
        out['best_evals'] = 0
        # out['hsigcount'] = 0
        out['termination'] = {}
        self.out = out
        
        self.const = DictClass()
        self.const.chiN = N**0.5*(1-1./(4.*N)+1./(21.*N**2)) # expectation of 
                                  #   ||N(0,I)|| == norm(randn(N,1))

        # attribute for stopping criteria in function stop 
        self.stoplastiter = 0 
        self.callbackstop = 0 
        self.fit = DictClass() 
        self.fit.fit = []   # not really necessary 
        self.fit.hist = []  # short history of best
        self.fit.histbest = []   # long history of best
        self.fit.histmedian = [] # long history of median

        if self.opts['verb_append'] > 0:
            self.countevals = self.opts['verb_append']
        self.more_to_write = []  #  N*[1]  # needed when writing takes place before setting

        # say hello
        if opts['verb_disp'] > 0:
            if self.sp.mu == 1:
                print '(%d,%d)-CMA-ES' % (self.sp.mu, self.sp.popsize),
            else:
                print '(%d_w,%d)-CMA-ES' % (self.sp.mu, self.sp.popsize), 
            print '(mu_w=%2.1f,w_1=%d%%)' % (self.sp.mueff, int(100*self.sp.weights[0])),
            print 'in dimension %d (seed=%d)' % (N, opts['seed']) # + func.__name__ 
            if opts['CMA_diagonal'] and self.sp.CMA_on:
                s = ''
                if opts['CMA_diagonal'] is not True:
                    s = ' for '
                    if opts['CMA_diagonal'] < np.inf:
                        s += str(int(opts['CMA_diagonal']))
                    else:
                        s += str(np.floor(opts['CMA_diagonal']))
                    s += ' iterations'
                    s += ' (1/ccov=' + str(round(1./(self.sp.c1+self.sp.cmu))) + ')'
                print '   Covariance matrix is diagonal' + s 

    #____________________________________________________________
    def tell(self, solutions, function_values, function_values_reevaluated=None, check_points=True):  
        """Pass objective function values to CMA-ES to prepare for next
        iteration. This core procedure of the CMA-ES algorithm updates 
        all state variables: two evolution paths, the distribution mean, 
        the covariance matrix and a step-size. 

        :Arguments:
           solutions -- list or array of candidate solution points (of
              type Solution or numpy.ndarray), most presumably before 
              delivered by method ask().
           function_values -- list or array of objective function values 
              corresponding to the respective points. Beside for termination 
              decisions, only the ranking of values in function_values
              is used.
           check_points -- if true, allows to savely pass Solutions that are 
              not necessarily generated using ask() 
        :Details: tell() updates the parameters of the multivariate
            normal search distribtion, namely covariance matrix and
            step-size and updates also the number of function evaluations
            countevals. 
        """
    #____________________________________________________________
    # TODO: consider an input argument that flags injected trust-worthy solutions (which means 
    #       that they can be treated "absolut" rather than "relative")
        if self.flgtelldone: 
            raise _Error('tell should only be called once per iteration')

        if self.countiter == 0 and self.opts['verb_log'] > 0 and not self.opts['verb_append']:
            self.writeHeaders()
            self.writeOutput()  # initial values for sigma etc
            
        lam = len(solutions)
        if lam != array(function_values).shape[0]:
            raise _Error('for each candidate solution '
                        + 'a function value must be provided')
        if lam < 3:  # mirrors: lam=2 would be fine
            raise _Error('population ' + str(lam) + ' is too small')
            
        ### prepare
        N = self.N
        sp = self.sp
        if 11 < 3 and lam != sp.popsize:  # turned off, because mu should stay constant  
            print 'WARNING: population size has changed, recomputing parameters'
            # TODO: when the population size changes, sigma
            #    should have been updated before
            self.sp.set(self.opts, lam)  # not really tested
        if lam < sp.mu:  # TODO: capture case mu=lam
            raise _Error('not enough solutions passed to function tell (mu>lambda)')
            
        assert sp is self.sp 
        
        self.countiter += 1  # >= 1 now
        self.countevals += sp.popsize
        flgseparable = self.opts['CMA_diagonal'] is True \
                       or self.countiter <= self.opts['CMA_diagonal'] 
        if not flgseparable and len(self.C.shape) == 1:  # C was diagonal ie 1-D
            # enter non-separable phase (no easy return from here)
            self.B = np.eye(N) # identity(N)
            self.C = np.diag(self.C)
            idx = np.argsort(self.D)
            self.D = self.D[idx]
            self.B = self.B[:,idx]

        ### manage fitness
        fit = self.fit  # make short cut
        
        # CPU for N,lam=20,200: this takes 10s vs 7s 
        fit.bndpen = self.boundPenalty.update(function_values, self)(solutions)
        # for testing: 
        # fit.bndpen = self.boundPenalty.update(function_values, self)([s.unrepaired for s in solutions])
        fit.idx = np.argsort(array(fit.bndpen) + array(function_values))
        fit.fit = array(function_values)[fit.idx]

        if 11 < 3: # qqq mirrored vectors with mu>1 would need special treatment
            # need to know whether 0,1 etc. are mirrors or 1,2 etc
            # but how? Maybe better in fmin ? 
            pass

        # fitness histories
        fit.hist.insert(0, fit.fit[0])
        # if len(self.fit.histbest) < 120+30*N/sp.popsize or  # does not help, as tablet in the beginning is the critical counter-case
        if ((self.countiter % 5) == 0):  # 20 percent of 1e5 gen.
            fit.histbest.insert(0, fit.fit[0])
            fit.histmedian.insert(0, np.median(fit.fit) if len(fit.fit<21) else fit.fit[int(self.popsize/2)])
        if len(fit.histbest) > 2e4: # 10 + 30*N/sp.popsize:
            fit.histbest.pop()
            fit.histmedian.pop()
        if len(fit.hist) > 10 + 30*N/sp.popsize:
            fit.hist.pop()

        pop = []  # create pop from input argument solutions
        for s in solutions:
            if type(s) is Solution and s.unrepaired is not None:
                pop.append(s.unrepaired)
            else:
                pop.append(s)
            pop[-1] = self.gp.geno(pop[-1])

        # compute new mean and sort pop
        mold = self.mean
        pop = array(pop, copy=False)[fit.idx] # only arrays can be multiple indexed

        # check and normalize each x - xold
        sigma_fac = 1
        if check_points:  # useful in case of injected solutions and/or adaptive encoding
            for k in xrange(sp.mu):  
                # l has modal value sqrt(N-1) and limit std sqrt(0.5) ==> 3*std==2.12132 
                l = self.mahalanobisNorm(pop[k]-mold) / self.sigma
                if l > sqrt(N-1) + 2:  # + 1./N ? 
                    pop[k] = mold + (pop[k]-mold) * (sqrt(N-1) + 2) / l
                # adapt also sigma: which are the trust-worthy/injected solutions? 
                if 11 < 3 and k == 0: 
                    sigma_fac = sigma_fac * exp(1*np.tanh((l**2/N-1)/2.))
                    # print sigma_fac
                    
        self.mean = mold + self.sp.kappa**-1 * \
                    (sum(sp.weights * array(pop[0:sp.mu]).T, 1) - mold)

        # get learning rate constants
        cc, c1, cmu = sp.cc, sp.c1, sp.cmu 
        if flgseparable:
            cc, c1, cmu = sp.cc_sep, sp.c1_sep, sp.cmu_sep
        
        # evolution paths
        self.ps = (1-sp.cs) * self.ps + \
                  (sqrt(sp.cs*(2-sp.cs)*sp.mueff)  / self.sigma) \
                  * np.dot(self.B, (1./self.D) * np.dot(self.B.T,
                                                      self.sp.kappa*(self.mean - mold)))

        # "hsig"
        hsig = sum(self.ps**2) / (1-(1-sp.cs)**(2*self.countiter)) / self.N < 2 + 4./(N+1)
        # adjust missing variance due to hsig, in 4-D with damps=1e99 and sig0 small  
        #       hsig leads to premature convergence of C otherwise
        hsiga = (1-hsig**2) * c1 * cc * (2-cc)  
        
        if 11 < 3:  # diagnostic data
            self.out['hsigcount'] += 1 - hsig
            if not hsig:
                self.hsiglist.append(self.countiter)
        if 11 < 3:  # diagnostic message
            if not hsig:  
                print self.countiter, ': hsig-stall'
        if 11 < 3:  # for testing purpose
            hsig = 1 # TODO: 
                     #       put correction term, but how? 
            if self.countiter == 1:
                print 'hsig=1'
        
        self.pc = (1-cc) * self.pc + \
                  hsig * sqrt(cc*(2-cc)*sp.mueff) \
                  * self.sp.kappa * (self.mean - mold) / self.sigma

        # covariance matrix adaptation 
        if sp.CMA_on:
            assert sp.c1 + sp.cmu < sp.mueff / N  # ??
            assert c1 + cmu <= 1
            
            # default full matrix case 
            if not flgseparable:
                Z = (pop[0:sp.mu] - mold) / self.sigma
                if 11 < 3:
                    # TODO: here optional the Suttorp update
                    
                    # CAVE: how to integrate the weights
                    self.itereigenupdated = self.countiter
                else:
                    Z = np.dot((cmu * sp.weights) * Z.T, Z)  # learning rate integrated
                    if 11 < 3: # ?3 to 5 times slower??
                        Z = np.zeros((N,N))
                        for k in xrange(sp.mu): 
                            z = (pop[k]-mold) 
                            Z += np.outer((cmu * sp.weights[k] / self.sigma**2) * z, z)
                    self.C = (1 - c1 - cmu + hsiga) * self.C + \
                             np.outer(c1 * self.pc, self.pc) + Z
                    self.dC = np.diag(self.C)  # for output and termination checking 

            else: # separable/diagonal linear case 
                assert(c1+cmu <= 1)
                Z = np.zeros(N)
                for k in xrange(sp.mu):
                    z = (pop[k]-mold) / self.sigma  # TODO see above
                    Z += sp.weights[k] * z * z  # is 1-D
                self.C = (1-c1-cmu+hsiga) * self.C + c1 * self.pc * self.pc + cmu * Z
                self.dC = self.C
                self.D = sqrt(self.C)  # C is a 1-D array
                self.itereigenupdated = self.countiter
                
        # step-size adaptation, adapt sigma
        self.sigma *= sigma_fac * \
                        np.exp(min(1, (sp.cs/sp.damps) *
                                (sqrt(sum(self.ps**2))/self.const.chiN - 1)))

        if self.sigma * min(self.dC)**0.5 < self.opts['minstd']:
            self.sigma = self.opts['minstd'] / min(self.dC)**0.5

        # TODO increase sigma in case of a plateau? 

        # Uncertainty noise measurement tobe-inserted

        # update output data
        # old: self.updateBest(self.gp.pheno(pop[0]), fit.fit[0], fit.idx[0]+1)
        self.updateBest(solutions[fit.idx[0]], fit.fit[0], fit.idx[0]+1)
        out = self.out
        # old: out['recent_x'] = self.gp.pheno(pop[0])
        out['recent_x'] = solutions[fit.idx[0]]
        out['recent_f'] = fit.fit[0]
        out['funevals'] = self.countevals
        out['iterations'] = self.countiter
        out['mean'] = self.gp.pheno(self.mean)  # TODO: rather genotyp?
        out['termination'] = self.stopdict

        # output
        if self.opts['verb_log'] > 0 and (self.countiter < 4 or
                                          self.countiter % self.opts['verb_log'] == 0):
            self.writeOutput(solutions[fit.idx[0]])
            
        self.flgtelldone = True

    # end tell()
        
    #____________________________________________________________
    #____________________________________________________________
    def feedForResume(self, X, function_values):
        """Given all "previous" candidate solutions and their respective 
        function values, the state of an CMAEvolutionStrategy object 
        can be reconstructed from this history. This is the purpose of 
        function feedForResume.  

        :Arguments:
           X -- (all) solution points in chronological order, phenotypic 
                representation. The number of points must be a multiple
                of popsize. 
           function_values -- respective objective function values

        :Details:
            feedForResume() can be called repeatedly with only parts of 
            the history. 
            feedForResume() feeds the history in popsize-chunks into tell()  
            The state of the random number generator might not be 
            reconstructed.        
        :Example:
            import cma
            
            # prepare 
            (x0, sigma0) = ... # initial values from previous trial
            X = ... # list of generated solutions from a previous trial
            f = ... # respective list of f-values
            
            # resume
            es = cma.CMAEvolutionStrategy(x0, sigma0)  
            es.feed(X, f)
            
            # continue with func as objective function
            while not es.stop:
               X = es.ask()
               es.tell(X, [func(x) for x in X])
        
        :Credits: to Dirk Bueche and Fabrice Marchal    
        :See also: class CMAEvolutionStrategy for a simple dump/load to resume
        """
    #------------------------------------------------------------
        if self.countiter > 0:
            print 'WARNING: feed should generally be used with a new object instance'
        if len(X) != len(function_values):
            raise _Error('number of solutions ' + str(len(X)) +
                ' and number function values ' +
                str(len(function_values))+' must not differ')
        popsize = self.sp.popsize
        if (len(X) % popsize) != 0:
            raise _Error('number of solutions ' + str(len(X)) +
                    ' must be a multiple of popsize (lambda) ' + 
                    str(popsize))
        for i in xrange(len(X) / popsize):
            # feed in chunks of size popsize
            self.ask()  # a fake ask, mainly for a conditioned calling of updateBD
                        # and secondary to get possibly the same random state
            self.tell(X[i*popsize:(i+1)*popsize], function_values[i*popsize:(i+1)*popsize])
            
    #____________________________________________________________
    #____________________________________________________________
    def updateBest(self, x, fun_value, evals=None):
        """Update best ever visited solution

        :Arguments:
           x -- solution point, phenotypic representation
           fun_value -- respective objective function value
           evals -- function evaluation number when x was evaluated
                in the iteration

        :Description:
           When fun_value is smaller than the function value of the
           so far stored best solution, the best solution is replaced. 

        :see also: best
        """
    #------------------------------------------------------------
        if evals is None:
            evals = self.sp.popsize+1
        
        if self.out['best_f'] > fun_value: 
            self.out['best_x'] = x.copy() 
            self.out['best_f'] = fun_value
            self.out['best_evals'] = self.countevals - self.sp.popsize + evals

    #____________________________________________________________
    #____________________________________________________________
    def mahalanobisNorm(self, x):
        """
        computes the Mahalanobis norm which is induced by the adapted covariance
        matrix C, disregarding sigma. 
         :Argument:
           a genotype solution point
         :Example:
           es = cma.CMAEvolutionStragegy(numpy.ones(10), 1)
           x1 = numpy.random.randn(10)
           x2 = numpy.random.randn(10)
           es.mahalanobisNorm(es.gp.geno(x2-x1))  # amounts initially to
                                                  # the euclidean distance
        """
    #------------------------------------------------------------
        return sqrt(sum((self.D**-1 * np.dot(self.B.T, x))**2))
        
    #____________________________________________________________
    #____________________________________________________________
    def printlineheader(self):
        """print annotation for printline()"""
        print 'Iterat #Fevals   function value     axis ratio  sigma   minstd maxstd min:sec'
        sys.stdout.flush()

    #____________________________________________________________
    #____________________________________________________________
    def printline(self, verb_disp=None):  # TODO: rather assign opt['verb_disp'] as default? 
        """prints some infos according to printlineheader()
        :Arguments:
            verb_disp -- print if iteration_counter % verb_disp == 0
        """
        if verb_disp is None:
            verb_disp = self.opts['verb_disp']
            
        # console display 
        if verb_disp:
            if 0.1*(self.countiter-1) % verb_disp == 0:
                self.printlineheader()
            if self.countiter > 0 and (self.stop or self.countiter < 4 
                              or self.countiter % verb_disp == 0): 
                print repr(self.countiter).rjust(5), repr(self.countevals).rjust(7), \
                      '%.15e' % (min(self.fit.fit)), \
                      '%4.1e' % (self.D.max()/self.D.min()), \
                      '%6.2e' % self.sigma, \
                      '%6.0e' % (self.sigma * sqrt(min(self.dC))), \
                      '%6.0e' % (self.sigma * sqrt(max(self.dC))),
                if self.opts['verb_time']:
                    toc = time.clock() - self.tic  
                    print str(int(toc//60))+':'+str(round(toc%60,1)),
                print ''  # a new line
                # if self.countiter < 4:
                sys.stdout.flush()

    #____________________________________________________________
    #____________________________________________________________
    def writeOutput(self, xrecent=None):
        """
        write output to files
         :Arguments:
           xrecent -- recent best x-vector (candidate solution, phenotype)
               is not really necessary. 
        """
    #------------------------------------------------------------
        if 1 < 3:
            if xrecent is None:
                try:
                    xrecent = self.out['recent_x']
                except:
                    pass
            # fit
            if self.countiter > 0:
                fit = self.fit.fit[0]
                if self.fmean_noise_free != 0:
                    fit = self.fmean_noise_free
                fn = self.opts['verb_filenameprefix'] + 'fit.dat'
                try: 
                    f = open(fn, 'a')
                    f.write(str(self.countiter) + ' ' 
                            + str(self.countevals) + ' '
                            + str(self.sigma) + ' '
                            + str(self.D.max()/self.D.min()) + ' '
                            + str(self.out['best_f']) + ' '
                            + '%.16e' % fit + ' '
                            + str(self.fit.fit[self.sp.popsize//2]) + ' '
                            + str(self.fit.fit[-1]) + ' '
                            # + str(self.sp.popsize) + ' '
                            # + str(10**self.noiseS) + ' '
                            # + str(self.sp.kappa) + ' '
                            + ' '.join(str(i) for i in self.more_to_write)
                            + '\n')
                    f.close()
                except (IOError, OSError):
                    if self.countiter == 1:
                        print 'could not open/write file', fn
                finally:
                    f.close()
                
            # axlen
            fn = self.opts['verb_filenameprefix'] + 'axlen.dat'
            try: 
                f = open(fn, 'a')
                f.write(str(self.countiter) + ' ' 
                        + str(self.countevals) + ' '
                        + str(self.sigma) + ' '
                        + str(self.D.max()) + ' '
                        + str(self.D.min()) + ' '
                        + ' '.join(map(str, self.D))
                        + '\n')
                f.close()
            except (IOError, OSError):
                if self.countiter == 1:
                    print 'could not open/write file', fn
            finally:
                f.close()

            # stddev
            fn = self.opts['verb_filenameprefix'] + 'stddev.dat'
            try: 
                f = open(fn, 'a')
                f.write(str(self.countiter) + ' ' 
                        + str(self.countevals) + ' '
                        + str(self.sigma) + ' '
                        + '0 0 '
                        + ' '.join(map(str, self.sigma*sqrt(self.dC)))
                        + '\n')
                f.close()
            except (IOError, OSError):
                if self.countiter == 1:
                    print 'could not open/write file', fn
            fn = self.opts['verb_filenameprefix'] + 'xmean.dat'
            try: 
                f = open(fn, 'a')
                if self.countevals < self.sp.popsize:
                    f.write('0 0 0 0 0 '
                            + ' '.join(map(str,
                                              # TODO should be optional the phenotyp?
                                              self.gp.geno(self.x0)))  
                            + '\n')
                else:
                    f.write(str(self.countiter) + ' ' 
                            + str(self.countevals) + ' '
                            # + str(self.sigma) + ' '
                            + '0 '
                            + str(self.fmean_noise_free) + ' '
                            + str(self.fmean) + ' '  # TODO: this does not make sense
                            # TODO should be optional the phenotyp?
                            + ' '.join(map(str, self.mean)) 
                            + '\n')
                f.close()
            except (IOError, OSError):
                if self.countiter == 1:
                    print 'could not open/write file', fn
            finally:
                f.close()

            fn = self.opts['verb_filenameprefix'] + 'xrecentbest.dat'
            if self.countiter > 0 and xrecent is not None:
                try: 
                    f = open(fn, 'a')
                    f.write(str(self.countiter) + ' ' 
                            + str(self.countevals) + ' '
                            + str(self.sigma) + ' '
                            + '0 '
                            + str(self.fit.fit[0]) + ' '
                            + ' '.join(map(str, xrecent))
                            + '\n')
                    f.close()
                except (IOError, OSError):
                    if self.countiter == 1:
                        print 'could not open/write file', fn
                finally:
                    f.close()

    #____________________________________________________________
    #____________________________________________________________
    def writeHeaders(self):
        """
        write headers to files, overwrites existing files
        """
    #------------------------------------------------------------
        if self.opts['verb_log'] == 0:
            return

        # write headers for output
        fn = self.opts['verb_filenameprefix'] + 'fit.dat'
        strseedtime = 'seed=%d, %s' % (self.opts['seed'], time.asctime())

        try: 
            f = open(fn, 'w')
            f.write('% # columns="iteration, evaluation, sigma, axis ratio, ' +
                    'bestever, best, median, worst objective function value, ' +
                    'further objective values of best", ' +
                    strseedtime + 
                    # strftime("%Y/%m/%d %H:%M:%S", localtime()) + # just asctime() would do
                    '\n')
            f.close()
        except (IOError, OSError):
            print 'could not open file', fn
        finally:
            f.close()

        fn = self.opts['verb_filenameprefix'] + 'axlen.dat'
        try: 
            f = open(fn, 'w')
            f.write('%  columns="iteration, evaluation, sigma, max axis length, ' +
                    ' min axis length, all principle axes lengths ' +
                    ' (sorted square roots of eigenvalues of C)", ' +
                    strseedtime + 
                    '\n')
            f.close()
        except (IOError, OSError):
            print 'could not open file', fn
        finally:
            f.close()
        fn = self.opts['verb_filenameprefix'] + 'stddev.dat'
        try: 
            f = open(fn, 'w')
            f.write('% # columns=["iteration, evaluation, sigma, void, void, ' +
                    ' stds==sigma*sqrt(diag(C))", ' +
                    strseedtime + 
                    '\n')
            f.close()
        except (IOError, OSError):
            print 'could not open file', fn
        finally:
            f.close()

        fn = self.opts['verb_filenameprefix'] + 'xmean.dat'
        try: 
            f = open(fn, 'w')
            f.write('% # columns="iteration, evaluation, void, void, void, xmean", ' +
                    strseedtime)
            f.write(' # scaling_of_variables: ')
            if np.size(self.gp.scales) > 1:
                f.write(' '.join(map(str, self.gp.scales)))
            else:
                f.write(str(self.gp.scales))
            f.write(', typical_x: ')
            if np.size(self.gp.typical_x) > 1:
                f.write(' '.join(map(str, self.gp.typical_x)))
            else:
                f.write(str(self.gp.typical_x))
            f.write('\n')
            f.close()
        except (IOError, OSError):
            print 'could not open file', fn
        finally:
            f.close()

        fn = self.opts['verb_filenameprefix'] + 'xrecentbest.dat'
        try: 
            f = open(fn, 'w')
            f.write('% # iter+eval+sigma+0+fitness+xbest, ' + 
                    strseedtime + 
                    '\n')
            f.close()
        except (IOError, OSError):
            print 'could not open file', fn
        finally:
            f.close()

    # end def writeHeaders

# end class cma 

#____________________________________________________________
#____________________________________________________________
#
class Options(DictClass):
    """Options() returns a dictionary with the available options and their
    default values for function fmin and for class CMAEvolutionStrategy.
    
    Options(opts) returns the subset of recognized options in opts.  
    
    Option values can be "written" in a string and, when passed to fmin
    or CMAEvolutionStrategy, are evaluated using "N" and "popsize" as 
    known values for dimension and population size (sample size, number 
    of new solutions per iteration). 

    :Attributes:
        defaults -- dictionary of defaults values, also defines
            the accepted (known) keywords
        settableOptionsList -- list of options that can be set at
            any time (not only be initialized)
    
    :Details:
        All Options are originally defined via the input arguments of 
        fmin()
        
    :Example and description of recognized options:
        >>> import cma
        >>> cma.Options().printme(81)  # linebreak after 81 chars
         CMA_diagonal=0*100*N/sqrt(popsize)  # nb of iterations with diagonal covariance
             matrix, True for always 
         CMA_eigenmethod=0  # 0=numpy-s eigh, -1=pygsl, otherwise cma.eig (slower)
         CMA_mu=None  # parents selection parameter, default is popsize//2
         CMA_on=True  # False or 0 for no adaptation of the covariance matrix
         CMA_rankmu=True  # False or 0 for omitting rank-mu update of covariance matrix
         CMA_rankmualpha=0.3  # factor of rank-mu update if mu=1, subject to removal,
             default might change to 0.0 
         CMA_teststds=None  # factors for non-isotropic initial distr. mainly for test
             purpose, see scaling_...
         bounds=[None, None]  # lower (=bounds[0]) and upper domain boundaries, each a
             single scalar or a list/vector
         evalparallel=False  # in fmin(): func is called with a list of solutions, might
             be removed
         ftarget=-inf  # target function value, minimization
         incpopsize=2  # in fmin(): multiplier for increasing popsize before each restart
         maxfevals=inf  # maximum number of function evaluations
         maxiter=long(1e3*N**2/sqrt(popsize))  # maximum number of iterations
         minstd=0  # minimal std in any coordinate direction, cave interference with tol*
         noise_eps=1e-7  # perturbation factor for reevalution, not yet used
         noise_reevalfrac= 0.0  # 0.05, not yet working
         popsize=4+int(3*log(N))  # population size, AKA lambda, number of new solution
             per iteration
         restarts=0  # in fmin(): number of restarts
         scaling_of_variables=None  # scale for each variable, sigma0 is interpreted
             w.r.t. this scale, in that effective_sigma0 = sigma0*scaling. Internally the
             variables are divided by scaling_of_variables and sigma is unchanged,
             default is ones(N).
         seed=None  # random number seed
         termination_callback=None  # in fmin(): a function returning True for
             termination, called after each iteration step and could be abused for side
             effects.
         tolfacupx=1e3  # termination when step-size increases by tolfacupx (diverges).
             In this case the initial step-size was chosen far too small or the objective
             function unintentionally indicates better solutions far away from the
             initial solution x0. 
         tolfun=1e-11  # termination criterion: tolerance in function value, quite useful
         tolfunhist=1e-12  # termination criterion: tolerance in function value history
         tolstagnation=int(100 * N**1.5 / popsize)  # termination if no improvement over
             tolstagnation iterations
         tolx=1e-11  # termination criterion: tolerance in x-changes
         typical_x=None  # used with scaling_of_variables
         updatecovwait=None  # number of iterations without distribution update, name is
             subject to future changes
         verb_append=0  # initial evaluation counter, if >0 append (do not overwrite)
             output files
         verb_disp=100  # verbosity: display console output every verb_disp iteration
         verb_filenameprefix=outcmaes  # output filenames prefix
         verb_log=1  # verbosity: write data to files every verb_log iteration, writing
             can be time critical on fast to evaluate functions
         verb_plot=0  # in fmin(): plotdata() is called every verb_plot iteration
         verb_time=True  # output timings on console

        For tolstagnation the median over the first and the second half 
        of at least tolstagnation iterations are compared for both, the
        per-iteration best and per-iteration median function value. 
        Some options, as mentioned, are only used with fmin. 

    :See also: fmin(), CMAEvolutionStrategy, Parameters
     
    """
    
    # @classmethod # self is the class, not the instance
    # @property 
    # def default(self):
    #     """returns all options with defaults"""
    #     return fmin([],[])
     
    @staticmethod
    def defaults():
        return fmin([],[])

    settableOptionsList = ('ftarget', 'maxfevals', 'maxiter', 'termination_callback', 
                        'tolfacupx', 'tolfun', 'tolfunhist', 'tolx', 
                        'updatecovwait', 'verb_disp', 'verb_log')
    
    def __init__(self, dic=None, unchecked=False):
        """does not complement with default options or settings"""
        # if not Options.defaults:  # this is different from self.defaults!!!
        #     Options.defaults = fmin([],[])
        if dic is None:
            dic = Options.defaults()
        DictClass.__init__(self, **dic)  # TODO: no idea whether this is right and suffices
        # super(Options, self).__init__ should be the same
        if not unchecked: 
            for key in self.keys():
                if key not in Options.defaults():
                    print 'Warning in cma.Options.__init__(): invalid key', key, 'popped'
                    self.pop(key)
        # self.evaluated = False  # would become an option entry
                        
    def init(self, dict_or_key, val=None, warn=True):
        """initialize one or several options. 
        
        :Arguments:
            dict_or_key -- either a dictionary or a key. In the latter
                case, val must be provided
            val -- value for key

        :Details:
            Only known keys are accepted. Known keys are in Options.defaults()

        """
        dic = dict_or_key if val is None else {dict_or_key:val} 
        dic = dict_or_key
        if val is not None: 
            dic = {dict_or_key:val}
        for key, val in dic.items():
            if key not in Options.defaults():

                # TODO: find a better solution? 
                if warn:
                    print 'Warning in cma.Options.init(): key ' + \
                        str(key) + ' ignored'
            else:
                self[key] = val
                
        return self
            
    def set(self, dic, val=None, warn=True):
        """set can assign settable options from Options.settableOptionsList(), 
        use init() for others
        :Arguments:
            dic -- either a dictionary or a key. In the latter
                case, val must be provided
            val -- value for key
            warn -- bool, print a warning if the option is not settable
                and therefore ommitted
                
        This method will be most probably used with the opts attribute of 
        a CMAEvolutionStrategy instance. 

        """
        if val is not None:  # dic is a key in this case
            dic = {dic:val}  # compose a dictionary
        for key, val in dic.items():
            if key in Options.settableOptionsList:
                self[key] = val
            elif warn:
                print 'Warning in cma.Options.set(): key ' + str(key) + ' ignored'
        return self  # to allow o = Options(o).set(new)
    
    def complement(self):
        """add all missing options with their default values"""
        
        for key in Options.defaults():
            if key not in self: 
                self[key] = Options.defaults()[key]
        return self
                
    def settable(self):
        """Returns subset of those options that are settable at any
        time. 
         
        All settable options are in settableOptionsList()
        """
        return Options([i for i in self.items() 
                                if i[0] in Options.settableOptionsList])

    def __call__(self, key, default=None, loc=None):
        """Evaluates and returns the option value on the fly
        
        :Details:
            Keys that contain 'filename' are not evaluated.
            For loc is None, the self-dict is used as environment
            TODO: this does not work, as N is necessary, but not
            an option. 
            
        :See also: eval(), evalall()
        
        """
        if loc is None: 
            loc = self  # TODO: this hack is not so useful: popsize could be there, but N is missing

        val = self[key]
        if val == str(val):
            val = val.split('#')[0].strip()  # remove comments
            if key.find('filename') < 0:
                val = eval(val, globals(), loc)
        # invoke default
        # TODO: val in ... fails with array type, because it is applied element wise!
        # elif val in (None,(),[],{}) and default is not None:  
        elif val is None and default is not None:  
            val = eval(str(default), globals(), loc)
        return val
        
    def eval(self, key, default=None, loc=None):
        """Evaluates and resets the specified option value in 
        environment loc

        
        :Details:
            Keys that contain 'filename' are not evaluated.
            For loc is None, the self-dict is used as environment
            
        :See also: evalall(), __call__
        
        """
        self[key] = self(key, default, loc)
        return self[key]
        
    def evalall(self, loc=None):
        """Evaluates all option values in environment loc. 
        
        :See also: eval()"""
        # TODO: this needs rather the parameter N instead of loc
        if 'N' in loc.keys():  # TODO: __init__ of CMA can be simplified
            popsize = self('popsize', Options.defaults()['popsize'], loc)
            for k in self.keys():
                self.eval(k, Options.defaults()[k], 
                            {'N':loc['N'], 'popsize':popsize})
        return self
    
    def printme(self, linebreak=80):
        for i in sorted(Options.defaults().items()): 
            s = str(i[0]) + '=' + str(i[1])
            a = s.split(' ')

            # print s in chunks
            l = ''  # start entire to the left
            while a:
                while a and len(l) + len(a[0]) < linebreak:
                    l += ' ' + a.pop(0)
                print l
                l = '    '  # tab for subsequent lines

#____________________________________________________________
#____________________________________________________________
class Parameters(DictClass):
    """strategy parameters like population size and learning rates
    
    :Note: contrary to Options, Parameters is not (yet) part of the 
        "user-interface" and subject to future changes (might become
        an object instead a DictClass)
    
    :Example: 
        >>> import cma
        >>> es = cma.CMAEvolutionStrategy(20 * [0.1], 1)
        (6_w,12)-CMA-ES (mu_w=3.7,w_1=40%) in dimension 20 (seed=504519190)
            # the seed is "random" by default
        >>>
        >>> type(es.sp)  # sp contains the strategy parameters
        <class 'cma.Parameters'>
        >>>
        >>> cma.pprint(es.sp)
        {'CMA_on': True,
         'N': 20,
         'c1': 0.004181139918745593,
         'c1_sep': 0.034327992810300939,
         'cc': 0.17176721127681213,
         'cc_sep': 0.25259494835857677,
         'cmu': 0.0085149624979034746,
         'cmu_sep': 0.057796356229390715,
         'cs': 0.21434997799189287,
         'damps': 1.2143499779918929,
         'kappa': 1,
         'mu': 6,
         'mu_f': 6.0,
         'mueff': 3.7294589343030671,
         'popsize': 12,
         'rankmualpha': 0.29999999999999999,
         'weights': array([ 0.40240294,  0.25338908,  0.16622156,  0.10437523,  0.05640348,
                0.01720771])}
        >>>
        >> es.sp == cma.Parameters(20, 12, cma.Options().evalall({'N':20}))
        True
    
    :See also: Options, CMAEvolutionStrategy
    
    """
    
    # TODO: why is the population size mandatory? It must be in opts anyway
    def __init__(self, N, opts, ccovfac=1, verbose=True):
        """Compute strategy parameters mainly depending on
        dimension and population size """
        
        #____________________________________________________________
        #____________________________________________________________
        DictClass.__init__(self)
        sp = self
        sp.N = N
        self.set(opts, ccovfac=ccovfac, verbose=verbose)
        
    def set(self, opts, popsize=None, ccovfac=1, verbose=True):
        #____________________________________________________________
        #____________________________________________________________
        # learning rates cone and cmu as a function
        # of the degrees of freedom df
        # TODO: go back to old method, for sake of clearness? 
        def cone(df, mu, N):
            # TODO: candidate for a @property
            return 1. / (df + 2.*sqrt(df) + float(mu)/N)

        def cmu(df, mu, alphamu):
            return (alphamu + mu - 2. + 1./mu) / (df + 4.*sqrt(df) + mu/2.)
            
        sp = self
        N = sp.N
        if popsize:
            opts.evalall({'N':N, 'popsize':popsize})
        else:
            popsize = opts.evalall({'N':N}).popsize  # the default popsize is computed in Options()
        sp.popsize = popsize 
        sp.mu_f = sp.popsize / 2.0  # float value of mu
        if opts['CMA_mu'] is not None: 
            sp.mu_f = opts['CMA_mu']
        sp.mu = int(sp.mu_f + 0.49999) # round down for x.5
        sp.weights = log(sp.mu_f+0.5) - log(1.+np.arange(sp.mu))
        sp.weights /= sum(sp.weights)
        sp.mueff = 1./sum(sp.weights**2)
        sp.cs = (sp.mueff+2)/(N+sp.mueff+3)
        sp.cc = (4. + sp.mueff/N) / (N + 4. + 2.*sp.mueff/N)  
        sp.cc_sep = (1+1./N + sp.mueff/N) / (N**0.5 + 1./N + 2.*sp.mueff/N) # \not\gg\cc
        sp.rankmualpha = opts['CMA_rankmualpha']
        # sp.rankmualpha = _evalOption(opts['CMA_rankmualpha'], 0.3)
        sp.c1 = ccovfac*min(1,sp.popsize/6)*cone((N**2+N)/2, sp.mueff, N) # 2. / ((N+1.3)**2 + sp.mucov)
        sp.c1_sep = ccovfac*cone(N, sp.mueff, N) # 2. / ((N+1.3)**2 + sp.mucov)
        if 11 < 3:
            sp.c1 = 0.
            print 'c1 is zero'
        if opts['CMA_rankmu'] != 0:  # also empty
            sp.cmu = min(1-sp.c1, ccovfac*cmu((N**2+N)/2, sp.mueff, sp.rankmualpha))
            sp.cmu_sep = min(1-sp.c1_sep, ccovfac*cmu(N, sp.mueff, sp.rankmualpha))
        else:
            sp.cmu = sp.cmu_sep = 0

        sp.CMA_on = sp.c1 + sp.cmu > 0  
        # print sp.c1_sep / sp.cc_sep

        if not opts['CMA_on'] and opts['CMA_on'] not in (None,[],(),''): 
            sp.CMA_on = False
            # sp.c1 = sp.cmu = sp.c1_sep = sp.cmu_sep = 0
        sp.damps = (1 + 2*max(0,sqrt((sp.mueff-1)/(N+1))-1)) + sp.cs 
        # sp.damps = 2. * sp.mueff/sp.popsize + 0.3 + sp.cs  # nicer future setting 
        if 11 < 3:
            sp.damps = 30 * sp.damps  # 1e99 # (1 + 2*max(0,sqrt((sp.mueff-1)/(N+1))-1)) + sp.cs; 
            sp.damps = 20 # 1. + 20 * sp.cs**-1  # 1e99 # (1 + 2*max(0,sqrt((sp.mueff-1)/(N+1))-1)) + sp.cs; 
            print 'damps is', sp.damps

        # TODO: rather replace with cm=1/kappa
        sp.kappa = 1  # 4-D, lam=16, rank1, kappa < 4 does not influence convergence rate
                      # in larger dim it does, 15-D with defaults, kappa=8 factor 2 
        if sp.kappa != 1:
            print '  kappa =', sp.kappa
        
        if verbose:
            if not sp.CMA_on:
                print 'covariance matrix adaptation turned off'
            if opts['CMA_mu'] != None: 
                print 'mu =', sp.mu_f

        # return self  # the constructor returns itself
        
    def printme(self):
        pprint(self)


#____________________________________________________________
#____________________________________________________________
# 
# C and B are arrays rather than matrices, because they are
# addressed via B[i][j], matrices can only be addressed via B[i,j] 

# tred2(N, B, diagD, offdiag);
# tql2(N, diagD, offdiag, B);


# Symmetric Householder reduction to tridiagonal form, translated from JAMA package.

def eig(C):
    """eigendecomposition of a symmetric matrix, much slower than numpy.linalg.eigh
    :Returns: (EVals, Basis) the eigenvalues and an orthonormal basis of 
        corresponding eigenvectors
    """

# class eig(object):
#     def __call__(self, C):

# Householder transformation of a symmetric matrix V into tridiagonal form. 
    # -> n             : dimension
    # -> V             : symmetric nxn-matrix
    # <- V             : orthogonal transformation matrix:
    #                    tridiag matrix == V * V_in * V^t
    # <- d             : diagonal
    # <- e[0..n-1]     : off diagonal (elements 1..n-1)

    # Symmetric tridiagonal QL algorithm, iterative 
    # Computes the eigensystem from a tridiagonal matrix in roughtly 3N^3 operations
    # -> n     : Dimension. 
    # -> d     : Diagonale of tridiagonal matrix. 
    # -> e[1..n-1] : off-diagonal, output from Householder
    # -> V     : matrix output von Householder
    # <- d     : eigenvalues
    # <- e     : garbage?
    # <- V     : basis of eigenvectors, according to d
    

    #  tred2(N, B, diagD, offdiag); B=C on input
    #  tql2(N, diagD, offdiag, B); 
    
    #  private void tred2 (int n, double V[][], double d[], double e[]) {
    def tred2 (n, V, d, e):
        #  This is derived from the Algol procedures tred2 by
        #  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
        #  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
        #  Fortran subroutine in EISPACK.

        num_opt = False  # factor 1.5 in 30-D

        for j in xrange(n): 
            d[j] = V[n-1][j] # d is output argument

        # Householder reduction to tridiagonal form.
    
        for i in xrange(n-1,0,-1):
          # Scale to avoid under/overflow.
          h = 0.0
          if not num_opt:
              scale = 0.0
              for k in xrange(i):
                  scale = scale + abs(d[k])
          else:
              scale = sum(abs(d[0:i]))

          if scale == 0.0:
              e[i] = d[i-1]
              for j in xrange(i):
                  d[j] = V[i-1][j]
                  V[i][j] = 0.0
                  V[j][i] = 0.0
          else:

             # Generate Householder vector.
             if not num_opt:
                 for k in xrange(i):
                     d[k] /= scale
                     h += d[k] * d[k]
             else:
                 d[:i] /= scale
                 h = np.dot(d[:i],d[:i])

             f = d[i-1]
             g = sqrt(h)

             if f > 0:
                g = -g

             e[i] = scale * g
             h = h - f * g
             d[i-1] = f - g
             if not num_opt:
                 for j in xrange(i):
                     e[j] = 0.0
             else:
                 e[:i] = 0.0 
    
             # Apply similarity transformation to remaining columns.
    
             for j in xrange(i): 
                 f = d[j]
                 V[j][i] = f
                 g = e[j] + V[j][j] * f
                 if not num_opt:
                     for k in xrange(j+1, i):
                         g += V[k][j] * d[k]
                         e[k] += V[k][j] * f
                     e[j] = g
                 else:
                     e[j+1:i] += V.T[j][j+1:i] * f
                     e[j] = g + np.dot(V.T[j][j+1:i],d[j+1:i])

             f = 0.0
             if not num_opt:
                 for j in xrange(i):
                     e[j] /= h
                     f += e[j] * d[j]
             else:
                 e[:i] /= h
                 f += np.dot(e[:i],d[:i]) 

             hh = f / (h + h)
             if not num_opt:
                 for j in xrange(i): 
                     e[j] -= hh * d[j]
             else:
                 e[:i] -= hh * d[:i]
                 
             for j in xrange(i): 
                 f = d[j]
                 g = e[j]
                 if not num_opt:
                     for k in xrange(j, i): 
                         V[k][j] -= (f * e[k] + g * d[k])
                 else:
                     V.T[j][j:i] -= (f * e[j:i] + g * d[j:i])
                     
                 d[j] = V[i-1][j]
                 V[i][j] = 0.0

          d[i] = h
          # end for i--
    
        # Accumulate transformations.

        for i in xrange(n-1): 
           V[n-1][i] = V[i][i]
           V[i][i] = 1.0
           h = d[i+1]
           if h != 0.0:
              if not num_opt:
                  for k in xrange(i+1): 
                      d[k] = V[k][i+1] / h
              else:
                  d[:i+1] = V.T[i+1][:i+1] / h
                  
              for j in xrange(i+1): 
                  if not num_opt:
                      g = 0.0
                      for k in xrange(i+1): 
                          g += V[k][i+1] * V[k][j]
                      for k in xrange(i+1): 
                          V[k][j] -= g * d[k]
                  else:
                      g = np.dot(V.T[i+1][0:i+1], V.T[j][0:i+1])
                      V.T[j][:i+1] -= g * d[:i+1]

           if not num_opt:
               for k in xrange(i+1): 
                   V[k][i+1] = 0.0
           else:
               V.T[i+1][:i+1] = 0.0
               

        if not num_opt:
            for j in xrange(n): 
                d[j] = V[n-1][j]
                V[n-1][j] = 0.0
        else:
            d[:n] = V[n-1][:n]
            V[n-1][:n] = 0.0

        V[n-1][n-1] = 1.0
        e[0] = 0.0


    # Symmetric tridiagonal QL algorithm, taken from JAMA package.
    # private void tql2 (int n, double d[], double e[], double V[][]) {
    # needs roughly 3N^3 operations
    def tql2 (n, d, e, V):

        #  This is derived from the Algol procedures tql2, by
        #  Bowdler, Martin, Reinsch, and Wilkinson, Handbook for
        #  Auto. Comp., Vol.ii-Linear Algebra, and the corresponding
        #  Fortran subroutine in EISPACK.
    
        num_opt = False

        if not num_opt:
            for i in xrange(1,n): # (int i = 1; i < n; i++): 
                e[i-1] = e[i]
        else:
            e[0:n-1] = e[1:n]
        e[n-1] = 0.0
    
        f = 0.0
        tst1 = 0.0
        eps = 2.0**-52.0
        for l in xrange(n): # (int l = 0; l < n; l++) {

           # Find small subdiagonal element
    
           tst1 = max(tst1,abs(d[l]) + abs(e[l]))
           m = l
           while m < n: 
               if abs(e[m]) <= eps*tst1:
                   break
               m += 1

           # If m == l, d[l] is an eigenvalue,
           # otherwise, iterate.
    
           if m > l:
               iiter = 0
               while 1: # do {
                   iiter += 1  # (Could check iteration count here.)
    
                   # Compute implicit shift
    
                   g = d[l]
                   p = (d[l+1] - g) / (2.0 * e[l])
                   r = (p**2 + 1)**0.5  # hypot(p,1.0)
                   if p < 0:
                       r = -r

                   d[l] = e[l] / (p + r)
                   d[l+1] = e[l] * (p + r)
                   dl1 = d[l+1]
                   h = g - d[l]
                   if not num_opt: 
                       for i in xrange(l+2, n):
                           d[i] -= h
                   else:
                       d[l+2:n] -= h

                   f = f + h
    
                   # Implicit QL transformation.
    
                   p = d[m]
                   c = 1.0
                   c2 = c
                   c3 = c
                   el1 = e[l+1]
                   s = 0.0
                   s2 = 0.0

                   # hh = V.T[0].copy()  # only with num_opt
                   for i in xrange(m-1, l-1, -1): # (int i = m-1; i >= l; i--) {
                       c3 = c2
                       c2 = c
                       s2 = s
                       g = c * e[i]
                       h = c * p
                       r = (p**2 + e[i]**2)**0.5  # hypot(p,e[i])
                       e[i+1] = s * r
                       s = e[i] / r
                       c = p / r
                       p = c * d[i] - s * g
                       d[i+1] = h + s * (c * g + s * d[i])
    
                       # Accumulate transformation.
    
                       if not num_opt: # overall factor 3 in 30-D
                           for k in xrange(n): # (int k = 0; k < n; k++) {
                               h = V[k][i+1]
                               V[k][i+1] = s * V[k][i] + c * h
                               V[k][i] = c * V[k][i] - s * h
                       else: # about 20% faster in 10-D
                           hh = V.T[i+1].copy()
                           # hh[:] = V.T[i+1][:]
                           V.T[i+1] = s * V.T[i] + c * hh
                           V.T[i] = c * V.T[i] - s * hh
                           # V.T[i] *= c
                           # V.T[i] -= s * hh


                   p = -s * s2 * c3 * el1 * e[l] / dl1
                   e[l] = s * p
                   d[l] = c * p
    
                   # Check for convergence.
                   if abs(e[l]) <= eps*tst1:
                       break
             # } while (Math.abs(e[l]) > eps*tst1);

           d[l] = d[l] + f
           e[l] = 0.0
      

        # Sort eigenvalues and corresponding vectors.
        if 11 < 3:
            for i in xrange(n-1): # (int i = 0; i < n-1; i++) {
                k = i
                p = d[i]
                for j in xrange(i+1, n): # (int j = i+1; j < n; j++) {
                    if d[j] < p: # NH find smallest k>i
                        k = j
                        p = d[j]

                if k != i: 
                    d[k] = d[i] # swap k and i 
                    d[i] = p   
                    for j in xrange(n): # (int j = 0; j < n; j++) {
                        p = V[j][i]
                        V[j][i] = V[j][k]
                        V[j][k] = p
    # tql2

    N = len(C[0])
    if 11 < 3:
        V = np.array([x[:] for x in C])  # copy each "row"
        N = V[0].size
        d = np.zeros(N)
        e = np.zeros(N)
    else:
        V = [[x[i] for i in xrange(N)] for x in C]  # copy each "row"
        d = N * [0.]
        e = N * [0.]
        
    tred2(N, V, d, e)
    tql2(N, d, e, V)
    return (array(d), array(V))
    

#____________________________________________________________
#____________________________________________________________
# 
def downsampling(filename='outcmaes', factor=10):
    """
    rude downsampling of CMA-ES data file by factor, keeping the
    first factor entries. This function is a stump and subject
    to future changes. 
    :Arguments:
       filename: prefix of files used
       factor: downsampling factor
    :Output: filename+'down' files
    """
    for name in ('axlen','fit','stddev','xmean','xrecentbest'):
        f = open(filename+'down'+name+'.dat','w')
        iline = 0
        cwritten = 0
        for line in open(filename+name+'.dat'):
            if iline < factor or iline % factor == 0:
                f.write(line)
                cwritten += 1
            iline += 1
        f.close()
        print cwritten, 'lines written'
        

#____________________________________________________________
#____________________________________________________________
# 
def loaddata(filenameprefix='outcmaes'):
    """loads and returns data from files writen by class
    CMAEvolutionStrategy method writeOuput(), e.g. when using function
    fmin().

    :Arguments:
        filenameprefix -- filename prefix of data to be loaded (five files).
            By default 'outcmaes'. 
    :Returns:
        data dictionary: xrecent, xmean, f, D, std
    """
    if filenameprefix != str(filenameprefix):
        raise _Error('argument filenameprefix must be a string')
    
    dat = DictClass()
    dat.xrecent = _fileToMatrix(filenameprefix + 'xrecentbest.dat') 
    dat.xmean = _fileToMatrix(filenameprefix + 'xmean.dat')
    dat.std = _fileToMatrix(filenameprefix + 'stddev' + '.dat')
    # a hack to later write something into the last entry
    for key in ['xmean', 'xrecent', 'std']:
        dat.__dict__[key].append(dat.__dict__[key][-1])
        dat.__dict__[key] = array(dat.__dict__[key])
    dat.f = array(_fileToMatrix(filenameprefix + 'fit.dat'))
    dat.D = array(_fileToMatrix(filenameprefix + 'axlen' + '.dat'))
    return dat

#____________________________________________________________
#____________________________________________________________
# 
def dispdata(name=None, idx=np.r_[0:5,1e2:1e9:1e2,-10:0]):
    """displays data from files written by class CMAEvolutionStrategy in
    method writeOutput(). 
    
    :Arguments:
       `name` -- filename prefix
       `idx` -- indices corresponding to rows in the data file; by
           default the first five, then every 100-th, and the last
           10 rows. Too large index values are removed. 
    :Examples::
       import cma, numpy
       dat=cma.dispdata((),numpy.r_[0,-1])  # first and last
       dat=cma.dispdata((),numpy.r_[0:1e9:100,-1]) # every 100-th and last
       dat=cma.dispdata((),numpy.r_[0,-10:0]) # first and ten last
       dat=cma.dispdata((),numpy.r_[0:1e9:1e3,-10:0]) 
    """

    filenameprefix='outcmaes'
    if name == str(name):
        filenameprefix = name
        
    def printdatarow(dat, i):
        print '%5d' % int(dat.f[i,0]), '%6d' % int(dat.f[i,1]), '%.14e' % dat.f[i,5],
        print '%5.1e' % dat.f[i,3],
        print '%6.2e' % (dat.std[i,2] * max(dat.std[i,5:])), \
              '%6.2e' % (dat.std[i,2] * min(dat.std[i,5:])) 
    dat = loaddata(filenameprefix)
    ndata = dat.f.shape[0]

    # map index to iteration number, is difficult if not all iteration numbers exist
    # idx = idx[np.where(map(lambda x: x in dat.f[:,0], idx))[0]] # TODO: takes pretty long
    # otherwise: 
    idx = idx[idx<=ndata]
    idx = idx[-idx<=ndata]
    idxbest = np.argmin(dat.f[:,5])
    
    heading = 'Iterat Nfevals  function value    axis ratio maxstd   minstd'
    print heading
    for i in idx:
        printdatarow(dat, i)
    print heading
    printdatarow(dat, idxbest)
    

#____________________________________________________________
#____________________________________________________________
# 
def plotdivers(dat, iabscissa, foffset):
    """plots upper left subplot with fitness, sigma, etc. 
    :Arguments:
        iabscissa in (0,1): 0==versus fevals, 1==versus iteration
        foffset: offset to fitness for log-plot
    """
    from  matplotlib.pylab import semilogy, hold, plot, grid, \
             axis, title, text, xlabel
    hold(False)

    dfit = dat.f[:,5]-min(dat.f[:,5]) 
    dfit[dfit<1e-98] = np.NaN
        
    if dat.f.shape[1] > 7:
        # semilogy(dat.f[:, iabscissa], abs(dat.f[:,[6, 7, 10, 12]])+foffset,'-k')
        semilogy(dat.f[:, iabscissa], abs(dat.f[:,[6, 7]])+foffset,'-k')
        hold(True)

    # (larger indices): additional fitness data, for example constraints values
    if dat.f.shape[1] > 8:
        # dd = abs(dat.f[:,7:]) + 10*foffset
        # dd = np.where(dat.f[:,7:]==0, np.NaN, dd) # cannot be 
        semilogy(dat.f[:, iabscissa], np.abs(dat.f[:,8:]) + 10*foffset, 'm')
        hold(True)

    idx = np.where(dat.f[:,5]>1e-98)[0]  # positive values
    semilogy(dat.f[idx, iabscissa], dat.f[idx,5]+foffset, '.b')
    hold(True)
    grid(True)
    
    idx = np.where(dat.f[:,5] < -1e-98)  # negative values
    semilogy(dat.f[idx, iabscissa], abs(dat.f[idx,5])+foffset,'.r')
    
    semilogy(dat.f[:, iabscissa],abs(dat.f[:,5])+foffset,'-b')
    semilogy(dat.f[:, iabscissa], dfit, '-c')
    
    if 11 < 3:  # delta-fitness as points
        dfit = dat.f[1:, 5] - dat.f[:-1,5]  # should be negative usually
        semilogy(dat.f[1:,iabscissa],  # abs(fit(g) - fit(g-1))
            np.abs(dfit)+foffset, '.c')
        i = dfit > 0
        # print np.sum(i) / float(len(dat.f[1:,iabscissa]))
        semilogy(dat.f[1:,iabscissa][i],  # abs(fit(g) - fit(g-1))
            np.abs(dfit[i])+foffset, '.r')

    # overall minimum
    i = np.argmin(dat.f[:,5])
    semilogy(dat.f[i, iabscissa]*np.ones(2), dat.f[i,5]*np.ones(2), 'rd') 
    # semilogy(dat.f[-1, iabscissa]*np.ones(2), dat.f[-1,4]*np.ones(2), 'rd') 
    
    # AR and sigma
    semilogy(dat.f[:, iabscissa], dat.f[:,3], '-r') # AR
    semilogy(dat.f[:, iabscissa], dat.f[:,2],'-g') # sigma
    semilogy(dat.std[:-1, iabscissa], np.vstack([map(max, dat.std[:-1,5:]), map(min, dat.std[:-1,5:])]).T,
                 '-m', linewidth=2)
    text(dat.std[-2, iabscissa], max(dat.std[-2, 5:]), 'max std', fontsize=14)
    text(dat.std[-2, iabscissa], min(dat.std[-2, 5:]), 'min std', fontsize=14)
    ax = array(axis())
    # ax[1] = max(minxend, ax[1])
    axis(ax)
    text(ax[0], ax[2], # 10**(log10(ax[2])+0.05*(log10(ax[3])-log10(ax[2]))),
         '.f_recent=' + repr(dat.f[-1,5]) )
    
    # title('abs(f) (blue), f-min(f) (cyan), Sigma (green), Axis Ratio (red)')
    title('blue:abs(f), cyan:f-min(f), green:sigma, red:axis ratio')
    # pylab.xticks(xticklocs)

#____________________________________________________________
#____________________________________________________________
# 
def plotdata(fig=[], abscissa=1, iteridx=None, plot_mean=True,  # TODO: plot_mean default should be False
    foffset=1e-19, x_opt = None):
    """
    plots data from files written in method writeOutput() called from tell()
    :Arguments:
        fig -- filename or figure number, or both as a tuple (any order)
        abscissa -- 0==plot versus iteration count,
               1==plot versus function evaluation number
        iteridx -- iteration indices to plot
    :Returns: 
        plotted data 
    :Example:
       cma.plotdata();  # the optimization might be still 
                        # running in a different shell
       cma.show()  # to continue you might need to close the pop-up window 
                   # once and call cma.plotdata() again. 
                   # This behavior seems to disappear in subsequent 
                   # calls of cma.plotdata(). Also using ipython with -pylab
                   # option might help. 
       cma.savefig('fig325.png')
       cma.close()
       cma.downsampling()
       cma.plotdata((123, 'outcmaesdown'));
    """

    try: 
        from matplotlib.pylab import ioff, ion, draw, isinteractive
        # pylab: prodedural interface for matplotlib
        from  matplotlib.pylab import figure, subplot, semilogy, hold, plot, grid, \
             axis, title, text, xlabel
    except ImportError:
        print 'could not find matplotlib.pylab module, function plotdata() is not available'
    else: 
        pylab.rcParams['axes.formatter.limits'] = -3, 3
        pylab.rcParams['axes.grid'] = True
        
        filenameprefix='outcmaes'
        iabscissa = 1
        if fig == str(fig):
            filenameprefix = fig
        elif type(fig) is tuple:  # filename _and_ fignb given
            if type(fig[0]) is str:
                filenameprefix = fig[0]
                figure(fig[1])
            else:
                filenameprefix = fig[1]
                figure(fig[0])
        elif fig:
            figure(fig)
        else:
            figure(325)
        if abscissa in (0,1):
            iabscissa = abscissa
        interactive_status = isinteractive()
        ioff() # prevents immediate drawing
        
        dat = loaddata(filenameprefix)
        dat.x = dat.xrecent
        if len(dat.x) < 2:
            print 'not enough data to plot'
            return {}

        if plot_mean:
            dat.x = dat.xmean    # this is the genotyp
        if iteridx is not None:
            dat.f = dat.f[np.where(map(lambda x: x in iteridx, dat.f[:,0]))[0],:]
            dat.D = dat.D[np.where(map(lambda x: x in iteridx, dat.D[:,0]))[0],:]
            iteridx.append(dat.x[-1,1])  # last entry is artificial
            dat.x = dat.x[np.where(map(lambda x: x in iteridx, dat.x[:,0]))[0],:]
            dat.std = dat.std[np.where(map(lambda x: x in iteridx, dat.std[:,0]))[0],:]

        if iabscissa == 0:
            xlab = 'iterations' 
        elif iabscissa == 1:
            xlab = 'function evaluations' 
        
        # use fake last entry in x and std for line extension-annotation
        if dat.x.shape[1] < 100:
            minxend = int(1.06*dat.x[-2, iabscissa])
            # write y-values for individual annotation into dat.x
            dat.x[-1, iabscissa] = minxend  # TODO: should be ax[1]
            idx = np.argsort(dat.x[-2,5:])
            idx2 = np.argsort(idx)
            if x_opt is None: 
                dat.x[-1,5+idx] = np.linspace(np.min(dat.x[:,5:]), 
                            np.max(dat.x[:,5:]), dat.x.shape[1]-5)
            else: 
                dat.x[-1,5+idx] = np.logspace(np.log10(np.min(abs(dat.x[:,5:]))), 
                            np.log10(np.max(abs(dat.x[:,5:]))), dat.x.shape[1]-5)
        else:
            minxend = 0

        if len(dat.f) == 0:
            print 'nothing to plot'
            return
            
        # not in use anymore, see formatter above
        # xticklocs = np.arange(5) * np.round(minxend/4., -int(np.log10(minxend/4.)))
        
        # dfit(dfit<1e-98) = NaN;
        
        ioff() # turns update off

        # TODO: if abscissa==0 plot in chunks, ie loop over subsets where dat.f[:,0]==countiter is monotonous 

        subplot(2,2,1)
        plotdivers(dat, iabscissa, foffset)
        
        # TODO: modularize also the remaining subplots
        subplot(2,2,2)
        hold(False)
        if x_opt is not None:  # TODO: differentate neg and pos?
            semilogy(dat.x[:, iabscissa], abs(dat.x[:,5:]) - x_opt, '-')
        else:
            plot(dat.x[:, iabscissa], dat.x[:,5:],'-')
        hold(True)
        grid(True)
        ax = array(axis())
        # ax[1] = max(minxend, ax[1]) 
        axis(ax)
        ax[1] -= 1e-6 
        if dat.x.shape[1] < 100:
            yy = np.linspace(ax[2]+1e-6, ax[3]-1e-6, dat.x.shape[1]-5)
            yyl = np.sort(dat.x[-1,5:])
            idx = np.argsort(dat.x[-1,5:])
            idx2 = np.argsort(idx)
            if x_opt is not None:
                semilogy([dat.x[-1, iabscissa], ax[1]], [abs(dat.x[-1,5:]), yy[idx2]], 'k-') # line from last data point
                semilogy(np.dot(dat.x[-2, iabscissa],[1,1]), array([ax[2]+1e-6, ax[3]-1e-6]), 'k-')
            else:
                # plot([dat.x[-1, iabscissa], ax[1]], [dat.x[-1,5:], yy[idx2]], 'k-') # line from last data point
                plot(np.dot(dat.x[-2, iabscissa],[1,1]), array([ax[2]+1e-6, ax[3]-1e-6]), 'k-')
            # plot(array([dat.x[-1, iabscissa], ax[1]]),
            #      reshape(array([dat.x[-1,5:], yy[idx2]]).flatten(), (2,4)), '-k')
            for i in range(len(idx)):
                # TODOqqq: annotate phenotypic value!?
                # text(ax[1], yy[i], 'x(' + str(idx[i]) + ')=' + str(dat.x[-2,5+idx[i]]))
                text(dat.x[-1,iabscissa], dat.x[-1,5+i], 'x(' + str(idx[i]) + ')=' + str(dat.x[-2,5+idx[i]]))
                
        i = 2  # find smallest i where iteration count differs (in case the same row appears twice)
        while i < len(dat.f) and dat.f[-i][0] == dat.f[-1][0]:
            i += 1
        title('Object Variables (' + ('mean' if plot_mean else 'curr best') + 
                ', ' + str(dat.x.shape[1]-5) + '-D, popsize~' + 
                (str(int((dat.f[-1][1] - dat.f[-i][1]) / (dat.f[-1][0] - dat.f[-i][0]))) 
                    if len(dat.f.T[0]) > 1 and dat.f[-1][0] > dat.f[-i][0] else 'NA') 
                + ')')
        # pylab.xticks(xticklocs)
        
        subplot(2,2,3)
        hold(False)
        semilogy(dat.D[:, iabscissa], dat.D[:,5:], '-')
        hold(True)
        grid(True)
        ax = array(axis()) 
        # ax[1] = max(minxend, ax[1]) 
        axis(ax) 
        title('Scaling (All Main Axes)') 
        # pylab.xticks(xticklocs)
        xlabel(xlab) 
        
        subplot(2,2,4)
        hold(False)
        # remove sigma from stds (graphs become much better readible)
        dat.std[:,5:] = np.transpose(dat.std[:,5:].T / dat.std[:,2].T) 
        # ax = array(axis())
        # ax[1] = max(minxend, ax[1]) 
        # axis(ax)
        if 1 < 2 and dat.std.shape[1] < 100:
            # use fake last entry in x and std for line extension-annotation
            minxend = int(1.06*dat.x[-2, iabscissa])
            dat.std[-1, iabscissa] = minxend  # TODO: should be ax[1]
            idx = np.argsort(dat.std[-2,5:])
            idx2 = np.argsort(idx)
            dat.std[-1,5+idx] = np.logspace(np.log10(np.min(dat.std[:,5:])), 
                            np.log10(np.max(dat.std[:,5:])), dat.std.shape[1]-5)

            dat.std[-1, iabscissa] = minxend  # TODO: should be ax[1]
            yy = np.logspace(np.log10(ax[2]), np.log10(ax[3]), dat.std.shape[1]-5)
            yyl = np.sort(dat.std[-1,5:])
            idx = np.argsort(dat.std[-1,5:])
            idx2 = np.argsort(idx)
            plot(np.dot(dat.std[-2, iabscissa],[1,1]), array([ax[2]+1e-6, ax[3]-1e-6]), 'k-') # vertical separator
            hold(True)
            # plot([dat.std[-1, iabscissa], ax[1]], [dat.std[-1,5:], yy[idx2]], 'k-') # line from last data point
            for i in xrange(len(idx)):
                # text(ax[1], yy[i], ' '+str(idx[i]))
                text(dat.std[-1, iabscissa], dat.std[-1, 5+i], ' '+str(idx[i]))
        semilogy(dat.std[:, iabscissa], dat.std[:,5:], '-') 
        grid(True)
        title('Standard Deviations in All Coordinates')
        # pylab.xticks(xticklocs)
        xlabel(xlab)
        draw()  # does not suffice
        if interactive_status:
            ion()  # turns interactive mode on (again)
            draw()  
        # show() # turns interactive mode on
        # ioff() # turns interactive mode off
        
        # TODO: does not bring up the figure, show() is needed, but enters interactive mode
        
        # semilogy(dat.f[:, iabscissa], dat.f[:,6])
        return dat

#____________________________________________________________

def _fileToMatrix(file_name):
    """rudimentary method to read in data from a file removing the first line"""
    # TODO: np.loadtxt() might be an alternative
    #     try: 
    fil = open(file_name, 'r')
    fil.readline() # rudimentary, assume one comment line
    lineToRow = lambda line: map(float, line.split())
    res = map(lineToRow, fil.readlines())  
    fil.close()  # close file could be ommitted, reference
                 # counting should do during garbage collection, but...
    while res != [] and res[0] == []:  # remove further empty lines
        del res[0] 
    return res
    #     except:
    print 'could not read file', file_name

#____________________________________________________________
#____________________________________________________________
class _Error(Exception):
    """generic exception of cma module"""
    pass


#____________________________________________________________
#____________________________________________________________
# 

def pprint(something):
    """ nicely formated print """
    try:
        import pprint
        # generate an instance PrettyPrinter
        pprint.PrettyPrinter().pprint(something) 
    except ImportError:
        print 'could not use pprint module, will apply regular print'
        print something

#____________________________________________
#____________________________________________________________

def _test(module=None):  # None is fine when called from inside the module
    import doctest
    print doctest.testmod(module)  # this is sooo coool!

#____________________________________________________________
#____________________________________________________________
#
def main(argv=None):
    """provides a versatile interface to do some trials from the command line
    not really productive though, but check python cma.py -h ..."""

    if argv is None:
        argv = sys.argv

    # uncomment for unit test
    # _test() 

    # handle input arguments, getopt might be helpful ;-)
    if len(argv) >= 1:  # function and help
        if len(argv) == 1 or argv[1].startswith('-h') or argv[1].startswith('--help'):
            print 'usage: python cma.py [options | func dim sig0 [optkey optval][optkey optval]...]'
            print '  Pass func name to run fmin on func, where func is a function from cma.fcts.'  
            print '  Use options --fcts and --doc for more infos or start ipython. '
            print '  Use option -t to run the doctest, -t -v to get (much) verbosity.'
            print '  Example: python cma.py elli 10 1 '
            print '  Example: python cma.py --test'
            fun = None
        elif argv[1].startswith('-t') or argv[1].startswith('--test'):
            import doctest
            print 'doctest for cma.py: one failure to be expected'
            # if argv[1][2] == 'v':
            doctest.testmod(report=True)  # # this is sooo coool!
            fun = None
        elif argv[1] == '--doc':
            print __doc__
            print CMAEvolutionStrategy.__doc__
            print fmin.__doc__
            fun = None
        elif argv[1] == '--fcts':
            print 'List of valid function names:'
            print [d for d in dir(fcts) if not d.startswith('_')]
            fun = None
        elif len(argv) > 3:
            fun = eval('fcts.' + argv[1])
        else:
            print 'try -h option'
            fun = None
            
    if fun is not None:
        
        if len(argv) > 2:  # dimension
            x0 = np.ones(eval(argv[2]))
        if len(argv) > 3:  # sigma
            sig0 = eval(argv[3])

        opts = {}
        for i in xrange(5, len(argv), 2):
            opts[argv[i-1]] = eval(argv[i])

        # run fmin
        if fun is not None:
            tic = time.time()
            fmin(fun, x0, sig0, **opts)  # ftarget=1e-9, tolfacupx=1e9, verb_log=10)
            # plotdata()
            # print ' best function value ', res[2]['es'].best[1]
            print 'elapsed time [s]:', round(time.time() - tic, 2)


#____________________________________________________________
#____________________________________________________________
#
# mainly for testing purpose
# executed when called from an OS shell
if __name__ == "__main__":
    main()

