# This file was automatically generated by SWIG (http://www.swig.org).
# Version 3.0.12
#
# Do not make changes to this file unless you know what you are doing--modify
# the SWIG interface file instead.

from sys import version_info as _swig_python_version_info
if _swig_python_version_info >= (2, 7, 0):
    def swig_import_helper():
        import importlib
        pkg = __name__.rpartition('.')[0]
        mname = '.'.join((pkg, '_tracik')).lstrip('.')
        try:
            return importlib.import_module(mname)
        except ImportError:
            return importlib.import_module('_tracik')
    _tracik = swig_import_helper()
    del swig_import_helper
elif _swig_python_version_info >= (2, 6, 0):
    def swig_import_helper():
        from os.path import dirname
        import imp
        fp = None
        try:
            fp, pathname, description = imp.find_module('_tracik', [dirname(__file__)])
        except ImportError:
            import _tracik
            return _tracik
        try:
            _mod = imp.load_module('_tracik', fp, pathname, description)
        finally:
            if fp is not None:
                fp.close()
        return _mod
    _tracik = swig_import_helper()
    del swig_import_helper
else:
    import _tracik
del _swig_python_version_info

try:
    _swig_property = property
except NameError:
    pass  # Python < 2.2 doesn't have 'property'.

try:
    import builtins as __builtin__
except ImportError:
    import __builtin__

def _swig_setattr_nondynamic(self, class_type, name, value, static=1):
    if (name == "thisown"):
        return self.this.own(value)
    if (name == "this"):
        if type(value).__name__ == 'SwigPyObject':
            self.__dict__[name] = value
            return
    method = class_type.__swig_setmethods__.get(name, None)
    if method:
        return method(self, value)
    if (not static):
        if _newclass:
            object.__setattr__(self, name, value)
        else:
            self.__dict__[name] = value
    else:
        raise AttributeError("You cannot add attributes to %s" % self)


def _swig_setattr(self, class_type, name, value):
    return _swig_setattr_nondynamic(self, class_type, name, value, 0)


def _swig_getattr(self, class_type, name):
    if (name == "thisown"):
        return self.this.own()
    method = class_type.__swig_getmethods__.get(name, None)
    if method:
        return method(self)
    raise AttributeError("'%s' object has no attribute '%s'" % (class_type.__name__, name))


def _swig_repr(self):
    try:
        strthis = "proxy of " + self.this.__repr__()
    except __builtin__.Exception:
        strthis = ""
    return "<%s.%s; %s >" % (self.__class__.__module__, self.__class__.__name__, strthis,)

try:
    _object = object
    _newclass = 1
except __builtin__.Exception:
    class _object:
        pass
    _newclass = 0

class SwigPyIterator(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, SwigPyIterator, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, SwigPyIterator, name)

    def __init__(self, *args, **kwargs):
        raise AttributeError("No constructor defined - class is abstract")
    __repr__ = _swig_repr
    __swig_destroy__ = _tracik.delete_SwigPyIterator
    __del__ = lambda self: None

    def value(self):
        return _tracik.SwigPyIterator_value(self)

    def incr(self, n=1):
        return _tracik.SwigPyIterator_incr(self, n)

    def decr(self, n=1):
        return _tracik.SwigPyIterator_decr(self, n)

    def distance(self, x):
        return _tracik.SwigPyIterator_distance(self, x)

    def equal(self, x):
        return _tracik.SwigPyIterator_equal(self, x)

    def copy(self):
        return _tracik.SwigPyIterator_copy(self)

    def next(self):
        return _tracik.SwigPyIterator_next(self)

    def __next__(self):
        return _tracik.SwigPyIterator___next__(self)

    def previous(self):
        return _tracik.SwigPyIterator_previous(self)

    def advance(self, n):
        return _tracik.SwigPyIterator_advance(self, n)

    def __eq__(self, x):
        return _tracik.SwigPyIterator___eq__(self, x)

    def __ne__(self, x):
        return _tracik.SwigPyIterator___ne__(self, x)

    def __iadd__(self, n):
        return _tracik.SwigPyIterator___iadd__(self, n)

    def __isub__(self, n):
        return _tracik.SwigPyIterator___isub__(self, n)

    def __add__(self, n):
        return _tracik.SwigPyIterator___add__(self, n)

    def __sub__(self, *args):
        return _tracik.SwigPyIterator___sub__(self, *args)
    def __iter__(self):
        return self
SwigPyIterator_swigregister = _tracik.SwigPyIterator_swigregister
SwigPyIterator_swigregister(SwigPyIterator)

class IntVector(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, IntVector, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, IntVector, name)
    __repr__ = _swig_repr

    def iterator(self):
        return _tracik.IntVector_iterator(self)
    def __iter__(self):
        return self.iterator()

    def __nonzero__(self):
        return _tracik.IntVector___nonzero__(self)

    def __bool__(self):
        return _tracik.IntVector___bool__(self)

    def __len__(self):
        return _tracik.IntVector___len__(self)

    def __getslice__(self, i, j):
        return _tracik.IntVector___getslice__(self, i, j)

    def __setslice__(self, *args):
        return _tracik.IntVector___setslice__(self, *args)

    def __delslice__(self, i, j):
        return _tracik.IntVector___delslice__(self, i, j)

    def __delitem__(self, *args):
        return _tracik.IntVector___delitem__(self, *args)

    def __getitem__(self, *args):
        return _tracik.IntVector___getitem__(self, *args)

    def __setitem__(self, *args):
        return _tracik.IntVector___setitem__(self, *args)

    def pop(self):
        return _tracik.IntVector_pop(self)

    def append(self, x):
        return _tracik.IntVector_append(self, x)

    def empty(self):
        return _tracik.IntVector_empty(self)

    def size(self):
        return _tracik.IntVector_size(self)

    def swap(self, v):
        return _tracik.IntVector_swap(self, v)

    def begin(self):
        return _tracik.IntVector_begin(self)

    def end(self):
        return _tracik.IntVector_end(self)

    def rbegin(self):
        return _tracik.IntVector_rbegin(self)

    def rend(self):
        return _tracik.IntVector_rend(self)

    def clear(self):
        return _tracik.IntVector_clear(self)

    def get_allocator(self):
        return _tracik.IntVector_get_allocator(self)

    def pop_back(self):
        return _tracik.IntVector_pop_back(self)

    def erase(self, *args):
        return _tracik.IntVector_erase(self, *args)

    def __init__(self, *args):
        this = _tracik.new_IntVector(*args)
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this

    def push_back(self, x):
        return _tracik.IntVector_push_back(self, x)

    def front(self):
        return _tracik.IntVector_front(self)

    def back(self):
        return _tracik.IntVector_back(self)

    def assign(self, n, x):
        return _tracik.IntVector_assign(self, n, x)

    def resize(self, *args):
        return _tracik.IntVector_resize(self, *args)

    def insert(self, *args):
        return _tracik.IntVector_insert(self, *args)

    def reserve(self, n):
        return _tracik.IntVector_reserve(self, n)

    def capacity(self):
        return _tracik.IntVector_capacity(self)
    __swig_destroy__ = _tracik.delete_IntVector
    __del__ = lambda self: None
IntVector_swigregister = _tracik.IntVector_swigregister
IntVector_swigregister(IntVector)

class DoubleVector(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, DoubleVector, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, DoubleVector, name)
    __repr__ = _swig_repr

    def iterator(self):
        return _tracik.DoubleVector_iterator(self)
    def __iter__(self):
        return self.iterator()

    def __nonzero__(self):
        return _tracik.DoubleVector___nonzero__(self)

    def __bool__(self):
        return _tracik.DoubleVector___bool__(self)

    def __len__(self):
        return _tracik.DoubleVector___len__(self)

    def __getslice__(self, i, j):
        return _tracik.DoubleVector___getslice__(self, i, j)

    def __setslice__(self, *args):
        return _tracik.DoubleVector___setslice__(self, *args)

    def __delslice__(self, i, j):
        return _tracik.DoubleVector___delslice__(self, i, j)

    def __delitem__(self, *args):
        return _tracik.DoubleVector___delitem__(self, *args)

    def __getitem__(self, *args):
        return _tracik.DoubleVector___getitem__(self, *args)

    def __setitem__(self, *args):
        return _tracik.DoubleVector___setitem__(self, *args)

    def pop(self):
        return _tracik.DoubleVector_pop(self)

    def append(self, x):
        return _tracik.DoubleVector_append(self, x)

    def empty(self):
        return _tracik.DoubleVector_empty(self)

    def size(self):
        return _tracik.DoubleVector_size(self)

    def swap(self, v):
        return _tracik.DoubleVector_swap(self, v)

    def begin(self):
        return _tracik.DoubleVector_begin(self)

    def end(self):
        return _tracik.DoubleVector_end(self)

    def rbegin(self):
        return _tracik.DoubleVector_rbegin(self)

    def rend(self):
        return _tracik.DoubleVector_rend(self)

    def clear(self):
        return _tracik.DoubleVector_clear(self)

    def get_allocator(self):
        return _tracik.DoubleVector_get_allocator(self)

    def pop_back(self):
        return _tracik.DoubleVector_pop_back(self)

    def erase(self, *args):
        return _tracik.DoubleVector_erase(self, *args)

    def __init__(self, *args):
        this = _tracik.new_DoubleVector(*args)
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this

    def push_back(self, x):
        return _tracik.DoubleVector_push_back(self, x)

    def front(self):
        return _tracik.DoubleVector_front(self)

    def back(self):
        return _tracik.DoubleVector_back(self)

    def assign(self, n, x):
        return _tracik.DoubleVector_assign(self, n, x)

    def resize(self, *args):
        return _tracik.DoubleVector_resize(self, *args)

    def insert(self, *args):
        return _tracik.DoubleVector_insert(self, *args)

    def reserve(self, n):
        return _tracik.DoubleVector_reserve(self, n)

    def capacity(self):
        return _tracik.DoubleVector_capacity(self)
    __swig_destroy__ = _tracik.delete_DoubleVector
    __del__ = lambda self: None
DoubleVector_swigregister = _tracik.DoubleVector_swigregister
DoubleVector_swigregister(DoubleVector)

class StringVector(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, StringVector, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, StringVector, name)
    __repr__ = _swig_repr

    def iterator(self):
        return _tracik.StringVector_iterator(self)
    def __iter__(self):
        return self.iterator()

    def __nonzero__(self):
        return _tracik.StringVector___nonzero__(self)

    def __bool__(self):
        return _tracik.StringVector___bool__(self)

    def __len__(self):
        return _tracik.StringVector___len__(self)

    def __getslice__(self, i, j):
        return _tracik.StringVector___getslice__(self, i, j)

    def __setslice__(self, *args):
        return _tracik.StringVector___setslice__(self, *args)

    def __delslice__(self, i, j):
        return _tracik.StringVector___delslice__(self, i, j)

    def __delitem__(self, *args):
        return _tracik.StringVector___delitem__(self, *args)

    def __getitem__(self, *args):
        return _tracik.StringVector___getitem__(self, *args)

    def __setitem__(self, *args):
        return _tracik.StringVector___setitem__(self, *args)

    def pop(self):
        return _tracik.StringVector_pop(self)

    def append(self, x):
        return _tracik.StringVector_append(self, x)

    def empty(self):
        return _tracik.StringVector_empty(self)

    def size(self):
        return _tracik.StringVector_size(self)

    def swap(self, v):
        return _tracik.StringVector_swap(self, v)

    def begin(self):
        return _tracik.StringVector_begin(self)

    def end(self):
        return _tracik.StringVector_end(self)

    def rbegin(self):
        return _tracik.StringVector_rbegin(self)

    def rend(self):
        return _tracik.StringVector_rend(self)

    def clear(self):
        return _tracik.StringVector_clear(self)

    def get_allocator(self):
        return _tracik.StringVector_get_allocator(self)

    def pop_back(self):
        return _tracik.StringVector_pop_back(self)

    def erase(self, *args):
        return _tracik.StringVector_erase(self, *args)

    def __init__(self, *args):
        this = _tracik.new_StringVector(*args)
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this

    def push_back(self, x):
        return _tracik.StringVector_push_back(self, x)

    def front(self):
        return _tracik.StringVector_front(self)

    def back(self):
        return _tracik.StringVector_back(self)

    def assign(self, n, x):
        return _tracik.StringVector_assign(self, n, x)

    def resize(self, *args):
        return _tracik.StringVector_resize(self, *args)

    def insert(self, *args):
        return _tracik.StringVector_insert(self, *args)

    def reserve(self, n):
        return _tracik.StringVector_reserve(self, n)

    def capacity(self):
        return _tracik.StringVector_capacity(self)
    __swig_destroy__ = _tracik.delete_StringVector
    __del__ = lambda self: None
StringVector_swigregister = _tracik.StringVector_swigregister
StringVector_swigregister(StringVector)

class ConstCharVector(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, ConstCharVector, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, ConstCharVector, name)
    __repr__ = _swig_repr

    def iterator(self):
        return _tracik.ConstCharVector_iterator(self)
    def __iter__(self):
        return self.iterator()

    def __nonzero__(self):
        return _tracik.ConstCharVector___nonzero__(self)

    def __bool__(self):
        return _tracik.ConstCharVector___bool__(self)

    def __len__(self):
        return _tracik.ConstCharVector___len__(self)

    def __getslice__(self, i, j):
        return _tracik.ConstCharVector___getslice__(self, i, j)

    def __setslice__(self, *args):
        return _tracik.ConstCharVector___setslice__(self, *args)

    def __delslice__(self, i, j):
        return _tracik.ConstCharVector___delslice__(self, i, j)

    def __delitem__(self, *args):
        return _tracik.ConstCharVector___delitem__(self, *args)

    def __getitem__(self, *args):
        return _tracik.ConstCharVector___getitem__(self, *args)

    def __setitem__(self, *args):
        return _tracik.ConstCharVector___setitem__(self, *args)

    def pop(self):
        return _tracik.ConstCharVector_pop(self)

    def append(self, x):
        return _tracik.ConstCharVector_append(self, x)

    def empty(self):
        return _tracik.ConstCharVector_empty(self)

    def size(self):
        return _tracik.ConstCharVector_size(self)

    def swap(self, v):
        return _tracik.ConstCharVector_swap(self, v)

    def begin(self):
        return _tracik.ConstCharVector_begin(self)

    def end(self):
        return _tracik.ConstCharVector_end(self)

    def rbegin(self):
        return _tracik.ConstCharVector_rbegin(self)

    def rend(self):
        return _tracik.ConstCharVector_rend(self)

    def clear(self):
        return _tracik.ConstCharVector_clear(self)

    def get_allocator(self):
        return _tracik.ConstCharVector_get_allocator(self)

    def pop_back(self):
        return _tracik.ConstCharVector_pop_back(self)

    def erase(self, *args):
        return _tracik.ConstCharVector_erase(self, *args)

    def __init__(self, *args):
        this = _tracik.new_ConstCharVector(*args)
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this

    def push_back(self, x):
        return _tracik.ConstCharVector_push_back(self, x)

    def front(self):
        return _tracik.ConstCharVector_front(self)

    def back(self):
        return _tracik.ConstCharVector_back(self)

    def assign(self, n, x):
        return _tracik.ConstCharVector_assign(self, n, x)

    def resize(self, *args):
        return _tracik.ConstCharVector_resize(self, *args)

    def insert(self, *args):
        return _tracik.ConstCharVector_insert(self, *args)

    def reserve(self, n):
        return _tracik.ConstCharVector_reserve(self, n)

    def capacity(self):
        return _tracik.ConstCharVector_capacity(self)
    __swig_destroy__ = _tracik.delete_ConstCharVector
    __del__ = lambda self: None
ConstCharVector_swigregister = _tracik.ConstCharVector_swigregister
ConstCharVector_swigregister(ConstCharVector)

Speed = _tracik.Speed
Distance = _tracik.Distance
Manip1 = _tracik.Manip1
Manip2 = _tracik.Manip2
class TRAC_IK(_object):
    __swig_setmethods__ = {}
    __setattr__ = lambda self, name, value: _swig_setattr(self, TRAC_IK, name, value)
    __swig_getmethods__ = {}
    __getattr__ = lambda self, name: _swig_getattr(self, TRAC_IK, name)
    __repr__ = _swig_repr
    __swig_destroy__ = _tracik.delete_TRAC_IK
    __del__ = lambda self: None

    def getKDLChain(self, chain_):
        return _tracik.TRAC_IK_getKDLChain(self, chain_)

    def getKDLLimits(self, lb_, ub_):
        return _tracik.TRAC_IK_getKDLLimits(self, lb_, ub_)

    def getSolutions(self, *args):
        return _tracik.TRAC_IK_getSolutions(self, *args)
    if _newclass:
        JointErr = staticmethod(_tracik.TRAC_IK_JointErr)
    else:
        JointErr = _tracik.TRAC_IK_JointErr

    def SetSolveType(self, _type):
        return _tracik.TRAC_IK_SetSolveType(self, _type)

    def __init__(self, *args):
        this = _tracik.new_TRAC_IK(*args)
        try:
            self.this.append(this)
        except __builtin__.Exception:
            self.this = this

    def CartToJnt(self, *args):
        return _tracik.TRAC_IK_CartToJnt(self, *args)

    def getNrOfJointsInChain(self):
        return _tracik.TRAC_IK_getNrOfJointsInChain(self)

    def getJointNamesInChain(self, urdf_string):
        return _tracik.TRAC_IK_getJointNamesInChain(self, urdf_string)

    def getLinkNamesInChain(self):
        return _tracik.TRAC_IK_getLinkNamesInChain(self)

    def getLowerBoundLimits(self):
        return _tracik.TRAC_IK_getLowerBoundLimits(self)

    def getUpperBoundLimits(self):
        return _tracik.TRAC_IK_getUpperBoundLimits(self)

    def setKDLLimits(self, *args):
        return _tracik.TRAC_IK_setKDLLimits(self, *args)
TRAC_IK_swigregister = _tracik.TRAC_IK_swigregister
TRAC_IK_swigregister(TRAC_IK)

def TRAC_IK_JointErr(arr1, arr2):
    return _tracik.TRAC_IK_JointErr(arr1, arr2)
TRAC_IK_JointErr = _tracik.TRAC_IK_JointErr

# This file is compatible with both classic and new-style classes.


