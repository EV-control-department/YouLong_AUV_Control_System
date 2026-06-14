# generated from rosidl_generator_py/resource/_idl.py.em
# with input from uv_msgs:msg/TaskStatus.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TaskStatus(type):
    """Metaclass of message 'TaskStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'STATUS_IDLE': 0,
        'STATUS_RUNNING': 1,
        'STATUS_PAUSED': 2,
        'STATUS_DONE': 3,
        'STATUS_ERROR': 4,
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
                'uv_msgs.msg.TaskStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__task_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__task_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__task_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__task_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__task_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'STATUS_IDLE': cls.__constants['STATUS_IDLE'],
            'STATUS_RUNNING': cls.__constants['STATUS_RUNNING'],
            'STATUS_PAUSED': cls.__constants['STATUS_PAUSED'],
            'STATUS_DONE': cls.__constants['STATUS_DONE'],
            'STATUS_ERROR': cls.__constants['STATUS_ERROR'],
        }

    @property
    def STATUS_IDLE(self):
        """Message constant 'STATUS_IDLE'."""
        return Metaclass_TaskStatus.__constants['STATUS_IDLE']

    @property
    def STATUS_RUNNING(self):
        """Message constant 'STATUS_RUNNING'."""
        return Metaclass_TaskStatus.__constants['STATUS_RUNNING']

    @property
    def STATUS_PAUSED(self):
        """Message constant 'STATUS_PAUSED'."""
        return Metaclass_TaskStatus.__constants['STATUS_PAUSED']

    @property
    def STATUS_DONE(self):
        """Message constant 'STATUS_DONE'."""
        return Metaclass_TaskStatus.__constants['STATUS_DONE']

    @property
    def STATUS_ERROR(self):
        """Message constant 'STATUS_ERROR'."""
        return Metaclass_TaskStatus.__constants['STATUS_ERROR']


class TaskStatus(metaclass=Metaclass_TaskStatus):
    """
    Message class 'TaskStatus'.

    Constants:
      STATUS_IDLE
      STATUS_RUNNING
      STATUS_PAUSED
      STATUS_DONE
      STATUS_ERROR
    """

    __slots__ = [
        '_status',
        '_current_task_index',
        '_total_tasks',
        '_current_task_name',
        '_error_message',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'status': 'uint8',
        'current_task_index': 'uint32',
        'total_tasks': 'uint32',
        'current_task_name': 'string',
        'error_message': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
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
        self.status = kwargs.get('status', int())
        self.current_task_index = kwargs.get('current_task_index', int())
        self.total_tasks = kwargs.get('total_tasks', int())
        self.current_task_name = kwargs.get('current_task_name', str())
        self.error_message = kwargs.get('error_message', str())

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
        if self.status != other.status:
            return False
        if self.current_task_index != other.current_task_index:
            return False
        if self.total_tasks != other.total_tasks:
            return False
        if self.current_task_name != other.current_task_name:
            return False
        if self.error_message != other.error_message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def status(self):
        """Message field 'status'."""
        return self._status

    @status.setter
    def status(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'status' field must be an unsigned integer in [0, 255]"
        self._status = value

    @builtins.property
    def current_task_index(self):
        """Message field 'current_task_index'."""
        return self._current_task_index

    @current_task_index.setter
    def current_task_index(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'current_task_index' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'current_task_index' field must be an unsigned integer in [0, 4294967295]"
        self._current_task_index = value

    @builtins.property
    def total_tasks(self):
        """Message field 'total_tasks'."""
        return self._total_tasks

    @total_tasks.setter
    def total_tasks(self, value):
        if self._check_fields:
            assert \
                isinstance(value, int), \
                "The 'total_tasks' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'total_tasks' field must be an unsigned integer in [0, 4294967295]"
        self._total_tasks = value

    @builtins.property
    def current_task_name(self):
        """Message field 'current_task_name'."""
        return self._current_task_name

    @current_task_name.setter
    def current_task_name(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'current_task_name' field must be of type 'str'"
        self._current_task_name = value

    @builtins.property
    def error_message(self):
        """Message field 'error_message'."""
        return self._error_message

    @error_message.setter
    def error_message(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'error_message' field must be of type 'str'"
        self._error_message = value
