# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uv_msgs:msg/Detection.idl
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


class Metaclass_Detection(type):
    """Metaclass of message 'Detection'."""

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
                'uv_msgs.msg.Detection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__detection

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Detection(metaclass=Metaclass_Detection):
    """Message class 'Detection'."""

    __slots__ = [
        '_class_id',
        '_confidence',
        '_pixel_x',
        '_pixel_y',
        '_bbox_x1',
        '_bbox_y1',
        '_bbox_x2',
        '_bbox_y2',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'class_id': 'uint16',
        'confidence': 'float',
        'pixel_x': 'float',
        'pixel_y': 'float',
        'bbox_x1': 'float',
        'bbox_y1': 'float',
        'bbox_x2': 'float',
        'bbox_y2': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        self.class_id = kwargs.get('class_id', int())
        self.confidence = kwargs.get('confidence', float())
        self.pixel_x = kwargs.get('pixel_x', float())
        self.pixel_y = kwargs.get('pixel_y', float())
        self.bbox_x1 = kwargs.get('bbox_x1', float())
        self.bbox_y1 = kwargs.get('bbox_y1', float())
        self.bbox_x2 = kwargs.get('bbox_x2', float())
        self.bbox_y2 = kwargs.get('bbox_y2', float())

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
        if self.class_id != other.class_id:
            return False
        if self.confidence != other.confidence:
            return False
        if self.pixel_x != other.pixel_x:
            return False
        if self.pixel_y != other.pixel_y:
            return False
        if self.bbox_x1 != other.bbox_x1:
            return False
        if self.bbox_y1 != other.bbox_y1:
            return False
        if self.bbox_x2 != other.bbox_x2:
            return False
        if self.bbox_y2 != other.bbox_y2:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def class_id(self):
        """Message field 'class_id'."""
        return self._class_id

    @class_id.setter
    def class_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'class_id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'class_id' field must be an unsigned integer in [0, 65535]"
        self._class_id = value

    @builtins.property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence = value

    @builtins.property
    def pixel_x(self):
        """Message field 'pixel_x'."""
        return self._pixel_x

    @pixel_x.setter
    def pixel_x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pixel_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pixel_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pixel_x = value

    @builtins.property
    def pixel_y(self):
        """Message field 'pixel_y'."""
        return self._pixel_y

    @pixel_y.setter
    def pixel_y(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'pixel_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pixel_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pixel_y = value

    @builtins.property
    def bbox_x1(self):
        """Message field 'bbox_x1'."""
        return self._bbox_x1

    @bbox_x1.setter
    def bbox_x1(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'bbox_x1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bbox_x1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bbox_x1 = value

    @builtins.property
    def bbox_y1(self):
        """Message field 'bbox_y1'."""
        return self._bbox_y1

    @bbox_y1.setter
    def bbox_y1(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'bbox_y1' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bbox_y1' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bbox_y1 = value

    @builtins.property
    def bbox_x2(self):
        """Message field 'bbox_x2'."""
        return self._bbox_x2

    @bbox_x2.setter
    def bbox_x2(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'bbox_x2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bbox_x2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bbox_x2 = value

    @builtins.property
    def bbox_y2(self):
        """Message field 'bbox_y2'."""
        return self._bbox_y2

    @bbox_y2.setter
    def bbox_y2(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'bbox_y2' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'bbox_y2' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._bbox_y2 = value
