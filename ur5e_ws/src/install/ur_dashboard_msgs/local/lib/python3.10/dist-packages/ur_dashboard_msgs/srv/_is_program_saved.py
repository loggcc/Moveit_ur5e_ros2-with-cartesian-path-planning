# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ur_dashboard_msgs:srv/IsProgramSaved.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_IsProgramSaved_Request(type):
    """Metaclass of message 'IsProgramSaved_Request'."""

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
            module = import_type_support('ur_dashboard_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ur_dashboard_msgs.srv.IsProgramSaved_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__is_program_saved__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__is_program_saved__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__is_program_saved__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__is_program_saved__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__is_program_saved__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IsProgramSaved_Request(metaclass=Metaclass_IsProgramSaved_Request):
    """Message class 'IsProgramSaved_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_IsProgramSaved_Response(type):
    """Metaclass of message 'IsProgramSaved_Response'."""

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
            module = import_type_support('ur_dashboard_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ur_dashboard_msgs.srv.IsProgramSaved_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__is_program_saved__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__is_program_saved__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__is_program_saved__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__is_program_saved__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__is_program_saved__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class IsProgramSaved_Response(metaclass=Metaclass_IsProgramSaved_Response):
    """Message class 'IsProgramSaved_Response'."""

    __slots__ = [
        '_answer',
        '_program_name',
        '_program_saved',
        '_success',
    ]

    _fields_and_field_types = {
        'answer': 'string',
        'program_name': 'string',
        'program_saved': 'boolean',
        'success': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.answer = kwargs.get('answer', str())
        self.program_name = kwargs.get('program_name', str())
        self.program_saved = kwargs.get('program_saved', bool())
        self.success = kwargs.get('success', bool())

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
        if self.answer != other.answer:
            return False
        if self.program_name != other.program_name:
            return False
        if self.program_saved != other.program_saved:
            return False
        if self.success != other.success:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def answer(self):
        """Message field 'answer'."""
        return self._answer

    @answer.setter
    def answer(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'answer' field must be of type 'str'"
        self._answer = value

    @builtins.property
    def program_name(self):
        """Message field 'program_name'."""
        return self._program_name

    @program_name.setter
    def program_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'program_name' field must be of type 'str'"
        self._program_name = value

    @builtins.property
    def program_saved(self):
        """Message field 'program_saved'."""
        return self._program_saved

    @program_saved.setter
    def program_saved(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'program_saved' field must be of type 'bool'"
        self._program_saved = value

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


class Metaclass_IsProgramSaved(type):
    """Metaclass of service 'IsProgramSaved'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ur_dashboard_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ur_dashboard_msgs.srv.IsProgramSaved')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__is_program_saved

            from ur_dashboard_msgs.srv import _is_program_saved
            if _is_program_saved.Metaclass_IsProgramSaved_Request._TYPE_SUPPORT is None:
                _is_program_saved.Metaclass_IsProgramSaved_Request.__import_type_support__()
            if _is_program_saved.Metaclass_IsProgramSaved_Response._TYPE_SUPPORT is None:
                _is_program_saved.Metaclass_IsProgramSaved_Response.__import_type_support__()


class IsProgramSaved(metaclass=Metaclass_IsProgramSaved):
    from ur_dashboard_msgs.srv._is_program_saved import IsProgramSaved_Request as Request
    from ur_dashboard_msgs.srv._is_program_saved import IsProgramSaved_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
