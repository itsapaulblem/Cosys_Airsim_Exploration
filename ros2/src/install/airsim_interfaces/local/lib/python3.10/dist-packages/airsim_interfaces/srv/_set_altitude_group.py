# generated from rosidl_generator_py/resource/_idl.py.em
# with input from airsim_interfaces:srv/SetAltitudeGroup.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SetAltitudeGroup_Request(type):
    """Metaclass of message 'SetAltitudeGroup_Request'."""

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
                'airsim_interfaces.srv.SetAltitudeGroup_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_altitude_group__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_altitude_group__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_altitude_group__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_altitude_group__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_altitude_group__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetAltitudeGroup_Request(metaclass=Metaclass_SetAltitudeGroup_Request):
    """Message class 'SetAltitudeGroup_Request'."""

    __slots__ = [
        '_vehicle_names',
        '_altitude',
        '_velocity',
        '_wait_on_last_task',
    ]

    _fields_and_field_types = {
        'vehicle_names': 'sequence<string>',
        'altitude': 'double',
        'velocity': 'double',
        'wait_on_last_task': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.vehicle_names = kwargs.get('vehicle_names', [])
        self.altitude = kwargs.get('altitude', float())
        self.velocity = kwargs.get('velocity', float())
        self.wait_on_last_task = kwargs.get('wait_on_last_task', bool())

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
        if self.vehicle_names != other.vehicle_names:
            return False
        if self.altitude != other.altitude:
            return False
        if self.velocity != other.velocity:
            return False
        if self.wait_on_last_task != other.wait_on_last_task:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def vehicle_names(self):
        """Message field 'vehicle_names'."""
        return self._vehicle_names

    @vehicle_names.setter
    def vehicle_names(self, value):
        if __debug__:
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'vehicle_names' field must be a set or sequence and each value of type 'str'"
        self._vehicle_names = value

    @builtins.property
    def altitude(self):
        """Message field 'altitude'."""
        return self._altitude

    @altitude.setter
    def altitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'altitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'altitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._altitude = value

    @builtins.property
    def velocity(self):
        """Message field 'velocity'."""
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'velocity' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'velocity' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._velocity = value

    @builtins.property
    def wait_on_last_task(self):
        """Message field 'wait_on_last_task'."""
        return self._wait_on_last_task

    @wait_on_last_task.setter
    def wait_on_last_task(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'wait_on_last_task' field must be of type 'bool'"
        self._wait_on_last_task = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_SetAltitudeGroup_Response(type):
    """Metaclass of message 'SetAltitudeGroup_Response'."""

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
                'airsim_interfaces.srv.SetAltitudeGroup_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__set_altitude_group__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__set_altitude_group__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__set_altitude_group__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__set_altitude_group__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__set_altitude_group__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SetAltitudeGroup_Response(metaclass=Metaclass_SetAltitudeGroup_Response):
    """Message class 'SetAltitudeGroup_Response'."""

    __slots__ = [
        '_success',
        '_message',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value


class Metaclass_SetAltitudeGroup(type):
    """Metaclass of service 'SetAltitudeGroup'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('airsim_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'airsim_interfaces.srv.SetAltitudeGroup')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__set_altitude_group

            from airsim_interfaces.srv import _set_altitude_group
            if _set_altitude_group.Metaclass_SetAltitudeGroup_Request._TYPE_SUPPORT is None:
                _set_altitude_group.Metaclass_SetAltitudeGroup_Request.__import_type_support__()
            if _set_altitude_group.Metaclass_SetAltitudeGroup_Response._TYPE_SUPPORT is None:
                _set_altitude_group.Metaclass_SetAltitudeGroup_Response.__import_type_support__()


class SetAltitudeGroup(metaclass=Metaclass_SetAltitudeGroup):
    from airsim_interfaces.srv._set_altitude_group import SetAltitudeGroup_Request as Request
    from airsim_interfaces.srv._set_altitude_group import SetAltitudeGroup_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
