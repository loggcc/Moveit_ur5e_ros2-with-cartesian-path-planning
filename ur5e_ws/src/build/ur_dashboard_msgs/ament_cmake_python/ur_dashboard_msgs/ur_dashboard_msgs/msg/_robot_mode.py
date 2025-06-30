# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ur_dashboard_msgs:msg/RobotMode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RobotMode(type):
    """Metaclass of message 'RobotMode'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NO_CONTROLLER': -1,
        'DISCONNECTED': 0,
        'CONFIRM_SAFETY': 1,
        'BOOTING': 2,
        'POWER_OFF': 3,
        'POWER_ON': 4,
        'IDLE': 5,
        'BACKDRIVE': 6,
        'RUNNING': 7,
        'UPDATING_FIRMWARE': 8,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ur_dashboard_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ur_dashboard_msgs.msg.RobotMode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__robot_mode
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__robot_mode
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__robot_mode
            cls._TYPE_SUPPORT = module.type_support_msg__msg__robot_mode
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__robot_mode

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NO_CONTROLLER': cls.__constants['NO_CONTROLLER'],
            'DISCONNECTED': cls.__constants['DISCONNECTED'],
            'CONFIRM_SAFETY': cls.__constants['CONFIRM_SAFETY'],
            'BOOTING': cls.__constants['BOOTING'],
            'POWER_OFF': cls.__constants['POWER_OFF'],
            'POWER_ON': cls.__constants['POWER_ON'],
            'IDLE': cls.__constants['IDLE'],
            'BACKDRIVE': cls.__constants['BACKDRIVE'],
            'RUNNING': cls.__constants['RUNNING'],
            'UPDATING_FIRMWARE': cls.__constants['UPDATING_FIRMWARE'],
        }

    @property
    def NO_CONTROLLER(self):
        """Message constant 'NO_CONTROLLER'."""
        return Metaclass_RobotMode.__constants['NO_CONTROLLER']

    @property
    def DISCONNECTED(self):
        """Message constant 'DISCONNECTED'."""
        return Metaclass_RobotMode.__constants['DISCONNECTED']

    @property
    def CONFIRM_SAFETY(self):
        """Message constant 'CONFIRM_SAFETY'."""
        return Metaclass_RobotMode.__constants['CONFIRM_SAFETY']

    @property
    def BOOTING(self):
        """Message constant 'BOOTING'."""
        return Metaclass_RobotMode.__constants['BOOTING']

    @property
    def POWER_OFF(self):
        """Message constant 'POWER_OFF'."""
        return Metaclass_RobotMode.__constants['POWER_OFF']

    @property
    def POWER_ON(self):
        """Message constant 'POWER_ON'."""
        return Metaclass_RobotMode.__constants['POWER_ON']

    @property
    def IDLE(self):
        """Message constant 'IDLE'."""
        return Metaclass_RobotMode.__constants['IDLE']

    @property
    def BACKDRIVE(self):
        """Message constant 'BACKDRIVE'."""
        return Metaclass_RobotMode.__constants['BACKDRIVE']

    @property
    def RUNNING(self):
        """Message constant 'RUNNING'."""
        return Metaclass_RobotMode.__constants['RUNNING']

    @property
    def UPDATING_FIRMWARE(self):
        """Message constant 'UPDATING_FIRMWARE'."""
        return Metaclass_RobotMode.__constants['UPDATING_FIRMWARE']


class RobotMode(metaclass=Metaclass_RobotMode):
    """
    Message class 'RobotMode'.

    Constants:
      NO_CONTROLLER
      DISCONNECTED
      CONFIRM_SAFETY
      BOOTING
      POWER_OFF
      POWER_ON
      IDLE
      BACKDRIVE
      RUNNING
      UPDATING_FIRMWARE
    """

    __slots__ = [
        '_mode',
    ]

    _fields_and_field_types = {
        'mode': 'int8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.mode = kwargs.get('mode', int())

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
        if self.mode != other.mode:
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
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'mode' field must be an integer in [-128, 127]"
        self._mode = value
