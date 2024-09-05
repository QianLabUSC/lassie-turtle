# generated from rosidl_generator_py/resource/_idl.py.em
# with input from traveler_msgs:msg/TravelerConfig.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TravelerConfig(type):
    """Metaclass of message 'TravelerConfig'."""

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
            module = import_type_support('traveler_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'traveler_msgs.msg.TravelerConfig')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__traveler_config
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__traveler_config
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__traveler_config
            cls._TYPE_SUPPORT = module.type_support_msg__msg__traveler_config
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__traveler_config

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TravelerConfig(metaclass=Metaclass_TravelerConfig):
    """Message class 'TravelerConfig'."""

    __slots__ = [
        '_running_scenario',
        '_filename',
        '_extrude_speed',
        '_extrude_angle',
        '_extrude_depth',
        '_shear_penetration_depth',
        '_shear_penetration_speed',
        '_shear_penetration_delay',
        '_shear_length',
        '_shear_speed',
        '_shear_delay',
        '_shear_return_speed',
        '_workspace_angular_speed',
        '_workspace_moving_angle',
        '_workspace_time_delay',
        '_static_length',
        '_static_angle',
        '_search_start',
        '_search_end',
        '_ground_height',
        '_back_speed',
    ]

    _fields_and_field_types = {
        'running_scenario': 'string',
        'filename': 'string',
        'extrude_speed': 'float',
        'extrude_angle': 'float',
        'extrude_depth': 'float',
        'shear_penetration_depth': 'float',
        'shear_penetration_speed': 'float',
        'shear_penetration_delay': 'float',
        'shear_length': 'float',
        'shear_speed': 'float',
        'shear_delay': 'float',
        'shear_return_speed': 'float',
        'workspace_angular_speed': 'float',
        'workspace_moving_angle': 'float',
        'workspace_time_delay': 'float',
        'static_length': 'float',
        'static_angle': 'float',
        'search_start': 'float',
        'search_end': 'float',
        'ground_height': 'float',
        'back_speed': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.running_scenario = kwargs.get('running_scenario', str())
        self.filename = kwargs.get('filename', str())
        self.extrude_speed = kwargs.get('extrude_speed', float())
        self.extrude_angle = kwargs.get('extrude_angle', float())
        self.extrude_depth = kwargs.get('extrude_depth', float())
        self.shear_penetration_depth = kwargs.get('shear_penetration_depth', float())
        self.shear_penetration_speed = kwargs.get('shear_penetration_speed', float())
        self.shear_penetration_delay = kwargs.get('shear_penetration_delay', float())
        self.shear_length = kwargs.get('shear_length', float())
        self.shear_speed = kwargs.get('shear_speed', float())
        self.shear_delay = kwargs.get('shear_delay', float())
        self.shear_return_speed = kwargs.get('shear_return_speed', float())
        self.workspace_angular_speed = kwargs.get('workspace_angular_speed', float())
        self.workspace_moving_angle = kwargs.get('workspace_moving_angle', float())
        self.workspace_time_delay = kwargs.get('workspace_time_delay', float())
        self.static_length = kwargs.get('static_length', float())
        self.static_angle = kwargs.get('static_angle', float())
        self.search_start = kwargs.get('search_start', float())
        self.search_end = kwargs.get('search_end', float())
        self.ground_height = kwargs.get('ground_height', float())
        self.back_speed = kwargs.get('back_speed', float())

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
        if self.running_scenario != other.running_scenario:
            return False
        if self.filename != other.filename:
            return False
        if self.extrude_speed != other.extrude_speed:
            return False
        if self.extrude_angle != other.extrude_angle:
            return False
        if self.extrude_depth != other.extrude_depth:
            return False
        if self.shear_penetration_depth != other.shear_penetration_depth:
            return False
        if self.shear_penetration_speed != other.shear_penetration_speed:
            return False
        if self.shear_penetration_delay != other.shear_penetration_delay:
            return False
        if self.shear_length != other.shear_length:
            return False
        if self.shear_speed != other.shear_speed:
            return False
        if self.shear_delay != other.shear_delay:
            return False
        if self.shear_return_speed != other.shear_return_speed:
            return False
        if self.workspace_angular_speed != other.workspace_angular_speed:
            return False
        if self.workspace_moving_angle != other.workspace_moving_angle:
            return False
        if self.workspace_time_delay != other.workspace_time_delay:
            return False
        if self.static_length != other.static_length:
            return False
        if self.static_angle != other.static_angle:
            return False
        if self.search_start != other.search_start:
            return False
        if self.search_end != other.search_end:
            return False
        if self.ground_height != other.ground_height:
            return False
        if self.back_speed != other.back_speed:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def running_scenario(self):
        """Message field 'running_scenario'."""
        return self._running_scenario

    @running_scenario.setter
    def running_scenario(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'running_scenario' field must be of type 'str'"
        self._running_scenario = value

    @builtins.property
    def filename(self):
        """Message field 'filename'."""
        return self._filename

    @filename.setter
    def filename(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'filename' field must be of type 'str'"
        self._filename = value

    @builtins.property
    def extrude_speed(self):
        """Message field 'extrude_speed'."""
        return self._extrude_speed

    @extrude_speed.setter
    def extrude_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'extrude_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'extrude_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._extrude_speed = value

    @builtins.property
    def extrude_angle(self):
        """Message field 'extrude_angle'."""
        return self._extrude_angle

    @extrude_angle.setter
    def extrude_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'extrude_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'extrude_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._extrude_angle = value

    @builtins.property
    def extrude_depth(self):
        """Message field 'extrude_depth'."""
        return self._extrude_depth

    @extrude_depth.setter
    def extrude_depth(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'extrude_depth' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'extrude_depth' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._extrude_depth = value

    @builtins.property
    def shear_penetration_depth(self):
        """Message field 'shear_penetration_depth'."""
        return self._shear_penetration_depth

    @shear_penetration_depth.setter
    def shear_penetration_depth(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shear_penetration_depth' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'shear_penetration_depth' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._shear_penetration_depth = value

    @builtins.property
    def shear_penetration_speed(self):
        """Message field 'shear_penetration_speed'."""
        return self._shear_penetration_speed

    @shear_penetration_speed.setter
    def shear_penetration_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shear_penetration_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'shear_penetration_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._shear_penetration_speed = value

    @builtins.property
    def shear_penetration_delay(self):
        """Message field 'shear_penetration_delay'."""
        return self._shear_penetration_delay

    @shear_penetration_delay.setter
    def shear_penetration_delay(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shear_penetration_delay' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'shear_penetration_delay' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._shear_penetration_delay = value

    @builtins.property
    def shear_length(self):
        """Message field 'shear_length'."""
        return self._shear_length

    @shear_length.setter
    def shear_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shear_length' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'shear_length' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._shear_length = value

    @builtins.property
    def shear_speed(self):
        """Message field 'shear_speed'."""
        return self._shear_speed

    @shear_speed.setter
    def shear_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shear_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'shear_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._shear_speed = value

    @builtins.property
    def shear_delay(self):
        """Message field 'shear_delay'."""
        return self._shear_delay

    @shear_delay.setter
    def shear_delay(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shear_delay' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'shear_delay' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._shear_delay = value

    @builtins.property
    def shear_return_speed(self):
        """Message field 'shear_return_speed'."""
        return self._shear_return_speed

    @shear_return_speed.setter
    def shear_return_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'shear_return_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'shear_return_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._shear_return_speed = value

    @builtins.property
    def workspace_angular_speed(self):
        """Message field 'workspace_angular_speed'."""
        return self._workspace_angular_speed

    @workspace_angular_speed.setter
    def workspace_angular_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'workspace_angular_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'workspace_angular_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._workspace_angular_speed = value

    @builtins.property
    def workspace_moving_angle(self):
        """Message field 'workspace_moving_angle'."""
        return self._workspace_moving_angle

    @workspace_moving_angle.setter
    def workspace_moving_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'workspace_moving_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'workspace_moving_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._workspace_moving_angle = value

    @builtins.property
    def workspace_time_delay(self):
        """Message field 'workspace_time_delay'."""
        return self._workspace_time_delay

    @workspace_time_delay.setter
    def workspace_time_delay(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'workspace_time_delay' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'workspace_time_delay' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._workspace_time_delay = value

    @builtins.property
    def static_length(self):
        """Message field 'static_length'."""
        return self._static_length

    @static_length.setter
    def static_length(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'static_length' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'static_length' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._static_length = value

    @builtins.property
    def static_angle(self):
        """Message field 'static_angle'."""
        return self._static_angle

    @static_angle.setter
    def static_angle(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'static_angle' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'static_angle' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._static_angle = value

    @builtins.property
    def search_start(self):
        """Message field 'search_start'."""
        return self._search_start

    @search_start.setter
    def search_start(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'search_start' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'search_start' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._search_start = value

    @builtins.property
    def search_end(self):
        """Message field 'search_end'."""
        return self._search_end

    @search_end.setter
    def search_end(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'search_end' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'search_end' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._search_end = value

    @builtins.property
    def ground_height(self):
        """Message field 'ground_height'."""
        return self._ground_height

    @ground_height.setter
    def ground_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ground_height' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'ground_height' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._ground_height = value

    @builtins.property
    def back_speed(self):
        """Message field 'back_speed'."""
        return self._back_speed

    @back_speed.setter
    def back_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'back_speed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'back_speed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._back_speed = value
