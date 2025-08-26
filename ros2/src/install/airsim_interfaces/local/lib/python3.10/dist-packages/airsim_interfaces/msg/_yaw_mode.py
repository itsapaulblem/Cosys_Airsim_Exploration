# generated from rosidl_generator_py/resource/_idl.py.em
# with input from airsim_interfaces:msg/YawMode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_YawMode(type):
    """Metaclass of message 'YawMode'."""

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
            module = import_type_support('airsim_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'airsim_interfaces.msg.YawMode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__yaw_mode
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__yaw_mode
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__yaw_mode
            cls._TYPE_SUPPORT = module.type_support_msg__msg__yaw_mode
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__yaw_mode

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class YawMode(metaclass=Metaclass_YawMode):
    """Message class 'YawMode'."""

    __slots__ = [
        '_is_rate',
        '_yaw_or_rate',
    ]

    _fields_and_field_types = {
        'is_rate': 'boolean',
        'yaw_or_rate': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.is_rate = kwargs.get('is_rate', bool())
        self.yaw_or_rate = kwargs.get('yaw_or_rate', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.is_rate != other.is_rate:
            return False
        if self.yaw_or_rate != other.yaw_or_rate:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def is_rate(self):
        """Message field 'is_rate'."""
        return self._is_rate

    @is_rate.setter
    def is_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_rate' field must be of type 'bool'"
        self._is_rate = value

    @builtins.property
    def yaw_or_rate(self):
        """Message field 'yaw_or_rate'."""
        return self._yaw_or_rate

    @yaw_or_rate.setter
    def yaw_or_rate(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'yaw_or_rate' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'yaw_or_rate' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._yaw_or_rate = value
