# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ur_dashboard_msgs:msg/SafetyMode.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SafetyMode(type):
    """Metaclass of message 'SafetyMode'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NORMAL': 1,
        'REDUCED': 2,
        'PROTECTIVE_STOP': 3,
        'RECOVERY': 4,
        'SAFEGUARD_STOP': 5,
        'SYSTEM_EMERGENCY_STOP': 6,
        'ROBOT_EMERGENCY_STOP': 7,
        'VIOLATION': 8,
        'FAULT': 9,
        'VALIDATE_JOINT_ID': 10,
        'UNDEFINED_SAFETY_MODE': 11,
        'AUTOMATIC_MODE_SAFEGUARD_STOP': 12,
        'SYSTEM_THREE_POSITION_ENABLING_STOP': 13,
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
                'ur_dashboard_msgs.msg.SafetyMode')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__safety_mode
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__safety_mode
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__safety_mode
            cls._TYPE_SUPPORT = module.type_support_msg__msg__safety_mode
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__safety_mode

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NORMAL': cls.__constants['NORMAL'],
            'REDUCED': cls.__constants['REDUCED'],
            'PROTECTIVE_STOP': cls.__constants['PROTECTIVE_STOP'],
            'RECOVERY': cls.__constants['RECOVERY'],
            'SAFEGUARD_STOP': cls.__constants['SAFEGUARD_STOP'],
            'SYSTEM_EMERGENCY_STOP': cls.__constants['SYSTEM_EMERGENCY_STOP'],
            'ROBOT_EMERGENCY_STOP': cls.__constants['ROBOT_EMERGENCY_STOP'],
            'VIOLATION': cls.__constants['VIOLATION'],
            'FAULT': cls.__constants['FAULT'],
            'VALIDATE_JOINT_ID': cls.__constants['VALIDATE_JOINT_ID'],
            'UNDEFINED_SAFETY_MODE': cls.__constants['UNDEFINED_SAFETY_MODE'],
            'AUTOMATIC_MODE_SAFEGUARD_STOP': cls.__constants['AUTOMATIC_MODE_SAFEGUARD_STOP'],
            'SYSTEM_THREE_POSITION_ENABLING_STOP': cls.__constants['SYSTEM_THREE_POSITION_ENABLING_STOP'],
        }

    @property
    def NORMAL(self):
        """Message constant 'NORMAL'."""
        return Metaclass_SafetyMode.__constants['NORMAL']

    @property
    def REDUCED(self):
        """Message constant 'REDUCED'."""
        return Metaclass_SafetyMode.__constants['REDUCED']

    @property
    def PROTECTIVE_STOP(self):
        """Message constant 'PROTECTIVE_STOP'."""
        return Metaclass_SafetyMode.__constants['PROTECTIVE_STOP']

    @property
    def RECOVERY(self):
        """Message constant 'RECOVERY'."""
        return Metaclass_SafetyMode.__constants['RECOVERY']

    @property
    def SAFEGUARD_STOP(self):
        """Message constant 'SAFEGUARD_STOP'."""
        return Metaclass_SafetyMode.__constants['SAFEGUARD_STOP']

    @property
    def SYSTEM_EMERGENCY_STOP(self):
        """Message constant 'SYSTEM_EMERGENCY_STOP'."""
        return Metaclass_SafetyMode.__constants['SYSTEM_EMERGENCY_STOP']

    @property
    def ROBOT_EMERGENCY_STOP(self):
        """Message constant 'ROBOT_EMERGENCY_STOP'."""
        return Metaclass_SafetyMode.__constants['ROBOT_EMERGENCY_STOP']

    @property
    def VIOLATION(self):
        """Message constant 'VIOLATION'."""
        return Metaclass_SafetyMode.__constants['VIOLATION']

    @property
    def FAULT(self):
        """Message constant 'FAULT'."""
        return Metaclass_SafetyMode.__constants['FAULT']

    @property
    def VALIDATE_JOINT_ID(self):
        """Message constant 'VALIDATE_JOINT_ID'."""
        return Metaclass_SafetyMode.__constants['VALIDATE_JOINT_ID']

    @property
    def UNDEFINED_SAFETY_MODE(self):
        """Message constant 'UNDEFINED_SAFETY_MODE'."""
        return Metaclass_SafetyMode.__constants['UNDEFINED_SAFETY_MODE']

    @property
    def AUTOMATIC_MODE_SAFEGUARD_STOP(self):
        """Message constant 'AUTOMATIC_MODE_SAFEGUARD_STOP'."""
        return Metaclass_SafetyMode.__constants['AUTOMATIC_MODE_SAFEGUARD_STOP']

    @property
    def SYSTEM_THREE_POSITION_ENABLING_STOP(self):
        """Message constant 'SYSTEM_THREE_POSITION_ENABLING_STOP'."""
        return Metaclass_SafetyMode.__constants['SYSTEM_THREE_POSITION_ENABLING_STOP']


class SafetyMode(metaclass=Metaclass_SafetyMode):
    """
    Message class 'SafetyMode'.

    Constants:
      NORMAL
      REDUCED
      PROTECTIVE_STOP
      RECOVERY
      SAFEGUARD_STOP
      SYSTEM_EMERGENCY_STOP
      ROBOT_EMERGENCY_STOP
      VIOLATION
      FAULT
      VALIDATE_JOINT_ID
      UNDEFINED_SAFETY_MODE
      AUTOMATIC_MODE_SAFEGUARD_STOP
      SYSTEM_THREE_POSITION_ENABLING_STOP
    """

    __slots__ = [
        '_mode',
    ]

    _fields_and_field_types = {
        'mode': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
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
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value
