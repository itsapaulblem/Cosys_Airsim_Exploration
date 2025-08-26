# generated from rosidl_generator_py/resource/_idl.py.em
# with input from airsim_interfaces:srv/GpsWaypoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GpsWaypoint_Request(type):
    """Metaclass of message 'GpsWaypoint_Request'."""

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
                'airsim_interfaces.srv.GpsWaypoint_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gps_waypoint__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gps_waypoint__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gps_waypoint__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gps_waypoint__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gps_waypoint__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GpsWaypoint_Request(metaclass=Metaclass_GpsWaypoint_Request):
    """Message class 'GpsWaypoint_Request'."""

    __slots__ = [
        '_latitude',
        '_longitude',
        '_altitude',
        '_speed',
        '_tolerance',
        '_wait_on_last_task',
    ]

    _fields_and_field_types = {
        'latitude': 'double',
        'longitude': 'double',
        'altitude': 'double',
        'speed': 'double',
        'tolerance': 'double',
        'wait_on_last_task': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.latitude = kwargs.get('latitude', float())
        self.longitude = kwargs.get('longitude', float())
        self.altitude = kwargs.get('altitude', float())
        self.speed = kwargs.get('speed', float())
        self.tolerance = kwargs.get('tolerance', float())
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
        if self.latitude != other.latitude:
            return False
        if self.longitude != other.longitude:
            return False
        if self.altitude != other.altitude:
            return False
        if self.speed != other.speed:
            return False
        if self.tolerance != other.tolerance:
            return False
        if self.wait_on_last_task != other.wait_on_last_task:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def latitude(self):
        """Message field 'latitude'."""
        return self._latitude

    @latitude.setter
    def latitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'latitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'latitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._latitude = value

    @builtins.property
    def longitude(self):
        """Message field 'longitude'."""
        return self._longitude

    @longitude.setter
    def longitude(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'longitude' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'longitude' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._longitude = value

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
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'speed' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._speed = value

    @builtins.property
    def tolerance(self):
        """Message field 'tolerance'."""
        return self._tolerance

    @tolerance.setter
    def tolerance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'tolerance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'tolerance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._tolerance = value

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
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_GpsWaypoint_Response(type):
    """Metaclass of message 'GpsWaypoint_Response'."""

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
                'airsim_interfaces.srv.GpsWaypoint_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__gps_waypoint__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__gps_waypoint__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__gps_waypoint__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__gps_waypoint__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__gps_waypoint__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GpsWaypoint_Response(metaclass=Metaclass_GpsWaypoint_Response):
    """Message class 'GpsWaypoint_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_final_distance',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'final_distance': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.final_distance = kwargs.get('final_distance', float())

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
        if self.final_distance != other.final_distance:
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

    @builtins.property
    def final_distance(self):
        """Message field 'final_distance'."""
        return self._final_distance

    @final_distance.setter
    def final_distance(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'final_distance' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'final_distance' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._final_distance = value


class Metaclass_GpsWaypoint(type):
    """Metaclass of service 'GpsWaypoint'."""

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
                'airsim_interfaces.srv.GpsWaypoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__gps_waypoint

            from airsim_interfaces.srv import _gps_waypoint
            if _gps_waypoint.Metaclass_GpsWaypoint_Request._TYPE_SUPPORT is None:
                _gps_waypoint.Metaclass_GpsWaypoint_Request.__import_type_support__()
            if _gps_waypoint.Metaclass_GpsWaypoint_Response._TYPE_SUPPORT is None:
                _gps_waypoint.Metaclass_GpsWaypoint_Response.__import_type_support__()


class GpsWaypoint(metaclass=Metaclass_GpsWaypoint):
    from airsim_interfaces.srv._gps_waypoint import GpsWaypoint_Request as Request
    from airsim_interfaces.srv._gps_waypoint import GpsWaypoint_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
