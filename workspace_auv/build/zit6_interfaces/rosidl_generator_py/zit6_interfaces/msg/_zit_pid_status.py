# generated from rosidl_generator_py/resource/_idl.py.em
# with input from zit6_interfaces:msg/ZitPidStatus.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# Member 'pos_kp'
# Member 'pos_max_v'
# Member 'pos_max_a'
# Member 'pos_out_limit'
# Member 'vel_kp'
# Member 'vel_ki'
# Member 'vel_kd'
# Member 'vel_i_limit'
# Member 'vel_out_limit'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ZitPidStatus(type):
    """Metaclass of message 'ZitPidStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('zit6_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'zit6_interfaces.msg.ZitPidStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__zit_pid_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__zit_pid_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__zit_pid_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__zit_pid_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__zit_pid_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ZitPidStatus(metaclass=Metaclass_ZitPidStatus):
    """Message class 'ZitPidStatus'."""

    __slots__ = [
        '_pos_kp',
        '_pos_max_v',
        '_pos_max_a',
        '_pos_out_limit',
        '_vel_kp',
        '_vel_ki',
        '_vel_kd',
        '_vel_i_limit',
        '_vel_out_limit',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'pos_kp': 'float[4]',
        'pos_max_v': 'float[4]',
        'pos_max_a': 'float[4]',
        'pos_out_limit': 'float[4]',
        'vel_kp': 'float[4]',
        'vel_ki': 'float[4]',
        'vel_kd': 'float[4]',
        'vel_i_limit': 'float[4]',
        'vel_out_limit': 'float[4]',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('float'), 4),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        if 'pos_kp' not in kwargs:
            self.pos_kp = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.pos_kp = numpy.array(kwargs.get('pos_kp'), dtype=numpy.float32)
            assert self.pos_kp.shape == (4, )
        if 'pos_max_v' not in kwargs:
            self.pos_max_v = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.pos_max_v = numpy.array(kwargs.get('pos_max_v'), dtype=numpy.float32)
            assert self.pos_max_v.shape == (4, )
        if 'pos_max_a' not in kwargs:
            self.pos_max_a = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.pos_max_a = numpy.array(kwargs.get('pos_max_a'), dtype=numpy.float32)
            assert self.pos_max_a.shape == (4, )
        if 'pos_out_limit' not in kwargs:
            self.pos_out_limit = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.pos_out_limit = numpy.array(kwargs.get('pos_out_limit'), dtype=numpy.float32)
            assert self.pos_out_limit.shape == (4, )
        if 'vel_kp' not in kwargs:
            self.vel_kp = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.vel_kp = numpy.array(kwargs.get('vel_kp'), dtype=numpy.float32)
            assert self.vel_kp.shape == (4, )
        if 'vel_ki' not in kwargs:
            self.vel_ki = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.vel_ki = numpy.array(kwargs.get('vel_ki'), dtype=numpy.float32)
            assert self.vel_ki.shape == (4, )
        if 'vel_kd' not in kwargs:
            self.vel_kd = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.vel_kd = numpy.array(kwargs.get('vel_kd'), dtype=numpy.float32)
            assert self.vel_kd.shape == (4, )
        if 'vel_i_limit' not in kwargs:
            self.vel_i_limit = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.vel_i_limit = numpy.array(kwargs.get('vel_i_limit'), dtype=numpy.float32)
            assert self.vel_i_limit.shape == (4, )
        if 'vel_out_limit' not in kwargs:
            self.vel_out_limit = numpy.zeros(4, dtype=numpy.float32)
        else:
            self.vel_out_limit = numpy.array(kwargs.get('vel_out_limit'), dtype=numpy.float32)
            assert self.vel_out_limit.shape == (4, )

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if any(self.pos_kp != other.pos_kp):
            return False
        if any(self.pos_max_v != other.pos_max_v):
            return False
        if any(self.pos_max_a != other.pos_max_a):
            return False
        if any(self.pos_out_limit != other.pos_out_limit):
            return False
        if any(self.vel_kp != other.vel_kp):
            return False
        if any(self.vel_ki != other.vel_ki):
            return False
        if any(self.vel_kd != other.vel_kd):
            return False
        if any(self.vel_i_limit != other.vel_i_limit):
            return False
        if any(self.vel_out_limit != other.vel_out_limit):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pos_kp(self):
        """Message field 'pos_kp'."""
        return self._pos_kp

    @pos_kp.setter
    def pos_kp(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'pos_kp' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 4, \
                    "The 'pos_kp' numpy.ndarray() must have a size of 4"
                self._pos_kp = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'pos_kp' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._pos_kp = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def pos_max_v(self):
        """Message field 'pos_max_v'."""
        return self._pos_max_v

    @pos_max_v.setter
    def pos_max_v(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'pos_max_v' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 4, \
                    "The 'pos_max_v' numpy.ndarray() must have a size of 4"
                self._pos_max_v = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'pos_max_v' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._pos_max_v = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def pos_max_a(self):
        """Message field 'pos_max_a'."""
        return self._pos_max_a

    @pos_max_a.setter
    def pos_max_a(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'pos_max_a' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 4, \
                    "The 'pos_max_a' numpy.ndarray() must have a size of 4"
                self._pos_max_a = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'pos_max_a' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._pos_max_a = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def pos_out_limit(self):
        """Message field 'pos_out_limit'."""
        return self._pos_out_limit

    @pos_out_limit.setter
    def pos_out_limit(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'pos_out_limit' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 4, \
                    "The 'pos_out_limit' numpy.ndarray() must have a size of 4"
                self._pos_out_limit = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'pos_out_limit' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._pos_out_limit = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def vel_kp(self):
        """Message field 'vel_kp'."""
        return self._vel_kp

    @vel_kp.setter
    def vel_kp(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'vel_kp' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 4, \
                    "The 'vel_kp' numpy.ndarray() must have a size of 4"
                self._vel_kp = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'vel_kp' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._vel_kp = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def vel_ki(self):
        """Message field 'vel_ki'."""
        return self._vel_ki

    @vel_ki.setter
    def vel_ki(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'vel_ki' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 4, \
                    "The 'vel_ki' numpy.ndarray() must have a size of 4"
                self._vel_ki = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'vel_ki' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._vel_ki = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def vel_kd(self):
        """Message field 'vel_kd'."""
        return self._vel_kd

    @vel_kd.setter
    def vel_kd(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'vel_kd' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 4, \
                    "The 'vel_kd' numpy.ndarray() must have a size of 4"
                self._vel_kd = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'vel_kd' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._vel_kd = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def vel_i_limit(self):
        """Message field 'vel_i_limit'."""
        return self._vel_i_limit

    @vel_i_limit.setter
    def vel_i_limit(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'vel_i_limit' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 4, \
                    "The 'vel_i_limit' numpy.ndarray() must have a size of 4"
                self._vel_i_limit = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'vel_i_limit' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._vel_i_limit = numpy.array(value, dtype=numpy.float32)

    @builtins.property
    def vel_out_limit(self):
        """Message field 'vel_out_limit'."""
        return self._vel_out_limit

    @vel_out_limit.setter
    def vel_out_limit(self, value):
        if self._check_fields:
            if isinstance(value, numpy.ndarray):
                assert value.dtype == numpy.float32, \
                    "The 'vel_out_limit' numpy.ndarray() must have the dtype of 'numpy.float32'"
                assert value.size == 4, \
                    "The 'vel_out_limit' numpy.ndarray() must have a size of 4"
                self._vel_out_limit = value
                return
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 4 and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'vel_out_limit' field must be a set or sequence with length 4 and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._vel_out_limit = numpy.array(value, dtype=numpy.float32)
