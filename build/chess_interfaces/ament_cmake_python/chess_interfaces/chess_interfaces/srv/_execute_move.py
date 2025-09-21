# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chess_interfaces:srv/ExecuteMove.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ExecuteMove_Request(type):
    """Metaclass of message 'ExecuteMove_Request'."""

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
            module = import_type_support('chess_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'chess_interfaces.srv.ExecuteMove_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__execute_move__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__execute_move__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__execute_move__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__execute_move__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__execute_move__request

            from chess_interfaces.msg import ChessMove
            if ChessMove.__class__._TYPE_SUPPORT is None:
                ChessMove.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteMove_Request(metaclass=Metaclass_ExecuteMove_Request):
    """Message class 'ExecuteMove_Request'."""

    __slots__ = [
        '_chess_move',
        '_confirm_execution',
    ]

    _fields_and_field_types = {
        'chess_move': 'chess_interfaces/ChessMove',
        'confirm_execution': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['chess_interfaces', 'msg'], 'ChessMove'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from chess_interfaces.msg import ChessMove
        self.chess_move = kwargs.get('chess_move', ChessMove())
        self.confirm_execution = kwargs.get('confirm_execution', bool())

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
        if self.chess_move != other.chess_move:
            return False
        if self.confirm_execution != other.confirm_execution:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def chess_move(self):
        """Message field 'chess_move'."""
        return self._chess_move

    @chess_move.setter
    def chess_move(self, value):
        if __debug__:
            from chess_interfaces.msg import ChessMove
            assert \
                isinstance(value, ChessMove), \
                "The 'chess_move' field must be a sub message of type 'ChessMove'"
        self._chess_move = value

    @builtins.property
    def confirm_execution(self):
        """Message field 'confirm_execution'."""
        return self._confirm_execution

    @confirm_execution.setter
    def confirm_execution(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'confirm_execution' field must be of type 'bool'"
        self._confirm_execution = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_ExecuteMove_Response(type):
    """Metaclass of message 'ExecuteMove_Response'."""

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
            module = import_type_support('chess_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'chess_interfaces.srv.ExecuteMove_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__execute_move__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__execute_move__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__execute_move__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__execute_move__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__execute_move__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecuteMove_Response(metaclass=Metaclass_ExecuteMove_Response):
    """Message class 'ExecuteMove_Response'."""

    __slots__ = [
        '_success',
        '_error_message',
        '_actual_execution_time',
        '_piece_captured',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'error_message': 'string',
        'actual_execution_time': 'float',
        'piece_captured': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.error_message = kwargs.get('error_message', str())
        self.actual_execution_time = kwargs.get('actual_execution_time', float())
        self.piece_captured = kwargs.get('piece_captured', bool())

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
        if self.error_message != other.error_message:
            return False
        if self.actual_execution_time != other.actual_execution_time:
            return False
        if self.piece_captured != other.piece_captured:
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
    def error_message(self):
        """Message field 'error_message'."""
        return self._error_message

    @error_message.setter
    def error_message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'error_message' field must be of type 'str'"
        self._error_message = value

    @builtins.property
    def actual_execution_time(self):
        """Message field 'actual_execution_time'."""
        return self._actual_execution_time

    @actual_execution_time.setter
    def actual_execution_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'actual_execution_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'actual_execution_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._actual_execution_time = value

    @builtins.property
    def piece_captured(self):
        """Message field 'piece_captured'."""
        return self._piece_captured

    @piece_captured.setter
    def piece_captured(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'piece_captured' field must be of type 'bool'"
        self._piece_captured = value


class Metaclass_ExecuteMove(type):
    """Metaclass of service 'ExecuteMove'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('chess_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'chess_interfaces.srv.ExecuteMove')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__execute_move

            from chess_interfaces.srv import _execute_move
            if _execute_move.Metaclass_ExecuteMove_Request._TYPE_SUPPORT is None:
                _execute_move.Metaclass_ExecuteMove_Request.__import_type_support__()
            if _execute_move.Metaclass_ExecuteMove_Response._TYPE_SUPPORT is None:
                _execute_move.Metaclass_ExecuteMove_Response.__import_type_support__()


class ExecuteMove(metaclass=Metaclass_ExecuteMove):
    from chess_interfaces.srv._execute_move import ExecuteMove_Request as Request
    from chess_interfaces.srv._execute_move import ExecuteMove_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
