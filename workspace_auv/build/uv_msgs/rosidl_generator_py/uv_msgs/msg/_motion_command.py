# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uv_msgs:msg/MotionCommand.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MotionCommand(type):
    """Metaclass of message 'MotionCommand'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MODE_POS_WORLD': 0,
        'MODE_POS_BODY': 1,
        'MODE_VEL_WORLD': 2,
        'MODE_VEL_BODY': 3,
        'MODE_FORCE_BODY': 4,
        'MODE_POS_WORLD_STEP': 5,
        'MODE_POS_BODY_STEP': 6,
        'MODE_VEL_WORLD_STEP': 7,
        'MODE_VEL_BODY_STEP': 8,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('uv_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uv_msgs.msg.MotionCommand')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__motion_command
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__motion_command
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__motion_command
            cls._TYPE_SUPPORT = module.type_support_msg__msg__motion_command
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__motion_command

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MODE_POS_WORLD': cls.__constants['MODE_POS_WORLD'],
            'MODE_POS_BODY': cls.__constants['MODE_POS_BODY'],
            'MODE_VEL_WORLD': cls.__constants['MODE_VEL_WORLD'],
            'MODE_VEL_BODY': cls.__constants['MODE_VEL_BODY'],
            'MODE_FORCE_BODY': cls.__constants['MODE_FORCE_BODY'],
            'MODE_POS_WORLD_STEP': cls.__constants['MODE_POS_WORLD_STEP'],
            'MODE_POS_BODY_STEP': cls.__constants['MODE_POS_BODY_STEP'],
            'MODE_VEL_WORLD_STEP': cls.__constants['MODE_VEL_WORLD_STEP'],
            'MODE_VEL_BODY_STEP': cls.__constants['MODE_VEL_BODY_STEP'],
        }

    @property
    def MODE_POS_WORLD(self):
        """Message constant 'MODE_POS_WORLD'."""
        return Metaclass_MotionCommand.__constants['MODE_POS_WORLD']

    @property
    def MODE_POS_BODY(self):
        """Message constant 'MODE_POS_BODY'."""
        return Metaclass_MotionCommand.__constants['MODE_POS_BODY']

    @property
    def MODE_VEL_WORLD(self):
        """Message constant 'MODE_VEL_WORLD'."""
        return Metaclass_MotionCommand.__constants['MODE_VEL_WORLD']

    @property
    def MODE_VEL_BODY(self):
        """Message constant 'MODE_VEL_BODY'."""
        return Metaclass_MotionCommand.__constants['MODE_VEL_BODY']

    @property
    def MODE_FORCE_BODY(self):
        """Message constant 'MODE_FORCE_BODY'."""
        return Metaclass_MotionCommand.__constants['MODE_FORCE_BODY']

    @property
    def MODE_POS_WORLD_STEP(self):
        """Message constant 'MODE_POS_WORLD_STEP'."""
        return Metaclass_MotionCommand.__constants['MODE_POS_WORLD_STEP']

    @property
    def MODE_POS_BODY_STEP(self):
        """Message constant 'MODE_POS_BODY_STEP'."""
        return Metaclass_MotionCommand.__constants['MODE_POS_BODY_STEP']

    @property
    def MODE_VEL_WORLD_STEP(self):
        """Message constant 'MODE_VEL_WORLD_STEP'."""
        return Metaclass_MotionCommand.__constants['MODE_VEL_WORLD_STEP']

    @property
    def MODE_VEL_BODY_STEP(self):
        """Message constant 'MODE_VEL_BODY_STEP'."""
        return Metaclass_MotionCommand.__constants['MODE_VEL_BODY_STEP']


class MotionCommand(metaclass=Metaclass_MotionCommand):
    """
    Message class 'MotionCommand'.

    Constants:
      MODE_POS_WORLD
      MODE_POS_BODY
      MODE_VEL_WORLD
      MODE_VEL_BODY
      MODE_FORCE_BODY
      MODE_POS_WORLD_STEP
      MODE_POS_BODY_STEP
      MODE_VEL_WORLD_STEP
      MODE_VEL_BODY_STEP
    """

    __slots__ = [
        '_mode',
        '_type_mask',
        '_x',
        '_y',
        '_z',
        '_yaw',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'mode': 'uint8',
        'type_mask': 'uint8',
        'x': 'float',
        'y': 'float',
        'z': 'float',
        'yaw': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.mode = kwargs.get('mode', int())
        self.type_mask = kwargs.get('type_mask', int())
        self.x = kwargs.get('x', float())
        self.y = kwargs.get('y', float())
        self.z = kwargs.get('z', float())
        self.yaw = kwargs.get('yaw', float())

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
        if self.mode != other.mode:
            return False
        if self.type_mask != other.type_mask:
            return False
        if self.x != other.x:
            return False
        if self.y != other.y:
            return False
        if self.z != other.z:
            return False
        if self.yaw != other.yaw:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

    @builtins.property
    def type_mask(self):
        """Message field 'type_mask'."""
        return self._type_mask

    @type_mask.setter
    def type_mask(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'type_mask' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'type_mask' field must be an unsigned integer in [0, 255]"
        self._type_mask = value

    @builtins.property
    def x(self):
        """Message field 'x'."""
        return self._x

    @x.setter
    def x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._x = value

    @builtins.property
    def y(self):
        """Message field 'y'."""
        return self._y

    @y.setter
    def y(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._y = value

    @builtins.property
    def z(self):
        """Message field 'z'."""
        return self._z

    @z.setter
    def z(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'z' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'z' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._z = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'yaw' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw = value
