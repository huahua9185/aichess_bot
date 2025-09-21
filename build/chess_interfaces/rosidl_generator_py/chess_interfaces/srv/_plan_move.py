# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chess_interfaces:srv/PlanMove.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlanMove_Request(type):
    """Metaclass of message 'PlanMove_Request'."""

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
                'chess_interfaces.srv.PlanMove_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__plan_move__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__plan_move__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__plan_move__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__plan_move__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__plan_move__request

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


class PlanMove_Request(metaclass=Metaclass_PlanMove_Request):
    """Message class 'PlanMove_Request'."""

    __slots__ = [
        '_chess_move',
        '_speed_factor',
        '_avoid_pieces',
        '_use_safe_approach',
    ]

    _fields_and_field_types = {
        'chess_move': 'chess_interfaces/ChessMove',
        'speed_factor': 'float',
        'avoid_pieces': 'boolean',
        'use_safe_approach': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['chess_interfaces', 'msg'], 'ChessMove'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from chess_interfaces.msg import ChessMove
        self.chess_move = kwargs.get('chess_move', ChessMove())
        self.speed_factor = kwargs.get('speed_factor', float())
        self.avoid_pieces = kwargs.get('avoid_pieces', bool())
        self.use_safe_approach = kwargs.get('use_safe_approach', bool())

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
        if self.speed_factor != other.speed_factor:
            return False
        if self.avoid_pieces != other.avoid_pieces:
            return False
        if self.use_safe_approach != other.use_safe_approach:
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
    def speed_factor(self):
        """Message field 'speed_factor'."""
        return self._speed_factor

    @speed_factor.setter
    def speed_factor(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'speed_factor' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'speed_factor' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._speed_factor = value

    @builtins.property
    def avoid_pieces(self):
        """Message field 'avoid_pieces'."""
        return self._avoid_pieces

    @avoid_pieces.setter
    def avoid_pieces(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'avoid_pieces' field must be of type 'bool'"
        self._avoid_pieces = value

    @builtins.property
    def use_safe_approach(self):
        """Message field 'use_safe_approach'."""
        return self._use_safe_approach

    @use_safe_approach.setter
    def use_safe_approach(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'use_safe_approach' field must be of type 'bool'"
        self._use_safe_approach = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_PlanMove_Response(type):
    """Metaclass of message 'PlanMove_Response'."""

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
                'chess_interfaces.srv.PlanMove_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__plan_move__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__plan_move__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__plan_move__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__plan_move__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__plan_move__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlanMove_Response(metaclass=Metaclass_PlanMove_Response):
    """Message class 'PlanMove_Response'."""

    __slots__ = [
        '_success',
        '_error_message',
        '_execution_time',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'error_message': 'string',
        'execution_time': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.error_message = kwargs.get('error_message', str())
        self.execution_time = kwargs.get('execution_time', float())

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
        if self.execution_time != other.execution_time:
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
    def execution_time(self):
        """Message field 'execution_time'."""
        return self._execution_time

    @execution_time.setter
    def execution_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'execution_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'execution_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._execution_time = value


class Metaclass_PlanMove(type):
    """Metaclass of service 'PlanMove'."""

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
                'chess_interfaces.srv.PlanMove')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__plan_move

            from chess_interfaces.srv import _plan_move
            if _plan_move.Metaclass_PlanMove_Request._TYPE_SUPPORT is None:
                _plan_move.Metaclass_PlanMove_Request.__import_type_support__()
            if _plan_move.Metaclass_PlanMove_Response._TYPE_SUPPORT is None:
                _plan_move.Metaclass_PlanMove_Response.__import_type_support__()


class PlanMove(metaclass=Metaclass_PlanMove):
    from chess_interfaces.srv._plan_move import PlanMove_Request as Request
    from chess_interfaces.srv._plan_move import PlanMove_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
