# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uv_msgs:msg/PidState.idl
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


class Metaclass_PidState(type):
    """Metaclass of message 'PidState'."""

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
            module = import_type_support('uv_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'uv_msgs.msg.PidState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__pid_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__pid_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__pid_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__pid_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__pid_state

            from uv_msgs.msg import PidGains
            if PidGains.__class__._TYPE_SUPPORT is None:
                PidGains.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PidState(metaclass=Metaclass_PidState):
    """Message class 'PidState'."""

    __slots__ = [
        '_speed_mag',
        '_vel_x',
        '_vel_y',
        '_pos_z',
        '_yaw',
        '_yaw_rate',
        '_error_length',
        '_error_angle_deg',
        '_speed_cmd',
        '_vx_body_cmd',
        '_vy_body_cmd',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'speed_mag': 'uv_msgs/PidGains',
        'vel_x': 'uv_msgs/PidGains',
        'vel_y': 'uv_msgs/PidGains',
        'pos_z': 'uv_msgs/PidGains',
        'yaw': 'uv_msgs/PidGains',
        'yaw_rate': 'uv_msgs/PidGains',
        'error_length': 'float',
        'error_angle_deg': 'float',
        'speed_cmd': 'float',
        'vx_body_cmd': 'float',
        'vy_body_cmd': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['uv_msgs', 'msg'], 'PidGains'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['uv_msgs', 'msg'], 'PidGains'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['uv_msgs', 'msg'], 'PidGains'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['uv_msgs', 'msg'], 'PidGains'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['uv_msgs', 'msg'], 'PidGains'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['uv_msgs', 'msg'], 'PidGains'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        from uv_msgs.msg import PidGains
        self.speed_mag = kwargs.get('speed_mag', PidGains())
        from uv_msgs.msg import PidGains
        self.vel_x = kwargs.get('vel_x', PidGains())
        from uv_msgs.msg import PidGains
        self.vel_y = kwargs.get('vel_y', PidGains())
        from uv_msgs.msg import PidGains
        self.pos_z = kwargs.get('pos_z', PidGains())
        from uv_msgs.msg import PidGains
        self.yaw = kwargs.get('yaw', PidGains())
        from uv_msgs.msg import PidGains
        self.yaw_rate = kwargs.get('yaw_rate', PidGains())
        self.error_length = kwargs.get('error_length', float())
        self.error_angle_deg = kwargs.get('error_angle_deg', float())
        self.speed_cmd = kwargs.get('speed_cmd', float())
        self.vx_body_cmd = kwargs.get('vx_body_cmd', float())
        self.vy_body_cmd = kwargs.get('vy_body_cmd', float())

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
        if self.speed_mag != other.speed_mag:
            return False
        if self.vel_x != other.vel_x:
            return False
        if self.vel_y != other.vel_y:
            return False
        if self.pos_z != other.pos_z:
            return False
        if self.yaw != other.yaw:
            return False
        if self.yaw_rate != other.yaw_rate:
            return False
        if self.error_length != other.error_length:
            return False
        if self.error_angle_deg != other.error_angle_deg:
            return False
        if self.speed_cmd != other.speed_cmd:
            return False
        if self.vx_body_cmd != other.vx_body_cmd:
            return False
        if self.vy_body_cmd != other.vy_body_cmd:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def speed_mag(self):
        """Message field 'speed_mag'."""
        return self._speed_mag

    @speed_mag.setter
    def speed_mag(self, value):
        if self._check_fields:
            from uv_msgs.msg import PidGains
            assert \
                isinstance(value, PidGains), \
                "The 'speed_mag' field must be a sub message of type 'PidGains'"
        self._speed_mag = value

    @builtins.property
    def vel_x(self):
        """Message field 'vel_x'."""
        return self._vel_x

    @vel_x.setter
    def vel_x(self, value):
        if self._check_fields:
            from uv_msgs.msg import PidGains
            assert \
                isinstance(value, PidGains), \
                "The 'vel_x' field must be a sub message of type 'PidGains'"
        self._vel_x = value

    @builtins.property
    def vel_y(self):
        """Message field 'vel_y'."""
        return self._vel_y

    @vel_y.setter
    def vel_y(self, value):
        if self._check_fields:
            from uv_msgs.msg import PidGains
            assert \
                isinstance(value, PidGains), \
                "The 'vel_y' field must be a sub message of type 'PidGains'"
        self._vel_y = value

    @builtins.property
    def pos_z(self):
        """Message field 'pos_z'."""
        return self._pos_z

    @pos_z.setter
    def pos_z(self, value):
        if self._check_fields:
            from uv_msgs.msg import PidGains
            assert \
                isinstance(value, PidGains), \
                "The 'pos_z' field must be a sub message of type 'PidGains'"
        self._pos_z = value

    @builtins.property
    def yaw(self):
        """Message field 'yaw'."""
        return self._yaw

    @yaw.setter
    def yaw(self, value):
        if self._check_fields:
            from uv_msgs.msg import PidGains
            assert \
                isinstance(value, PidGains), \
                "The 'yaw' field must be a sub message of type 'PidGains'"
        self._yaw = value

    @builtins.property
    def yaw_rate(self):
        """Message field 'yaw_rate'."""
        return self._yaw_rate

    @yaw_rate.setter
    def yaw_rate(self, value):
        if self._check_fields:
            from uv_msgs.msg import PidGains
            assert \
                isinstance(value, PidGains), \
                "The 'yaw_rate' field must be a sub message of type 'PidGains'"
        self._yaw_rate = value

    @builtins.property
    def error_length(self):
        """Message field 'error_length'."""
        return self._error_length

    @error_length.setter
    def error_length(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'error_length' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'error_length' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._error_length = value

    @builtins.property
    def error_angle_deg(self):
        """Message field 'error_angle_deg'."""
        return self._error_angle_deg

    @error_angle_deg.setter
    def error_angle_deg(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'error_angle_deg' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'error_angle_deg' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._error_angle_deg = value

    @builtins.property
    def speed_cmd(self):
        """Message field 'speed_cmd'."""
        return self._speed_cmd

    @speed_cmd.setter
    def speed_cmd(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'speed_cmd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed_cmd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed_cmd = value

    @builtins.property
    def vx_body_cmd(self):
        """Message field 'vx_body_cmd'."""
        return self._vx_body_cmd

    @vx_body_cmd.setter
    def vx_body_cmd(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'vx_body_cmd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vx_body_cmd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vx_body_cmd = value

    @builtins.property
    def vy_body_cmd(self):
        """Message field 'vy_body_cmd'."""
        return self._vy_body_cmd

    @vy_body_cmd.setter
    def vy_body_cmd(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'vy_body_cmd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vy_body_cmd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vy_body_cmd = value
