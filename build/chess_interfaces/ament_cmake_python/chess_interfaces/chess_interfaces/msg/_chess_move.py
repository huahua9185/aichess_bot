# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chess_interfaces:msg/ChessMove.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ChessMove(type):
    """Metaclass of message 'ChessMove'."""

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
                'chess_interfaces.msg.ChessMove')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__chess_move
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__chess_move
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__chess_move
            cls._TYPE_SUPPORT = module.type_support_msg__msg__chess_move
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__chess_move

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ChessMove(metaclass=Metaclass_ChessMove):
    """Message class 'ChessMove'."""

    __slots__ = [
        '_header',
        '_from_square',
        '_to_square',
        '_piece_type',
        '_captured_piece',
        '_promotion',
        '_is_castling',
        '_is_en_passant',
        '_is_check',
        '_is_checkmate',
        '_confidence',
        '_thinking_time',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'from_square': 'int8',
        'to_square': 'int8',
        'piece_type': 'int8',
        'captured_piece': 'int8',
        'promotion': 'int8',
        'is_castling': 'boolean',
        'is_en_passant': 'boolean',
        'is_check': 'boolean',
        'is_checkmate': 'boolean',
        'confidence': 'float',
        'thinking_time': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.from_square = kwargs.get('from_square', int())
        self.to_square = kwargs.get('to_square', int())
        self.piece_type = kwargs.get('piece_type', int())
        self.captured_piece = kwargs.get('captured_piece', int())
        self.promotion = kwargs.get('promotion', int())
        self.is_castling = kwargs.get('is_castling', bool())
        self.is_en_passant = kwargs.get('is_en_passant', bool())
        self.is_check = kwargs.get('is_check', bool())
        self.is_checkmate = kwargs.get('is_checkmate', bool())
        self.confidence = kwargs.get('confidence', float())
        self.thinking_time = kwargs.get('thinking_time', float())

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
        if self.header != other.header:
            return False
        if self.from_square != other.from_square:
            return False
        if self.to_square != other.to_square:
            return False
        if self.piece_type != other.piece_type:
            return False
        if self.captured_piece != other.captured_piece:
            return False
        if self.promotion != other.promotion:
            return False
        if self.is_castling != other.is_castling:
            return False
        if self.is_en_passant != other.is_en_passant:
            return False
        if self.is_check != other.is_check:
            return False
        if self.is_checkmate != other.is_checkmate:
            return False
        if self.confidence != other.confidence:
            return False
        if self.thinking_time != other.thinking_time:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def from_square(self):
        """Message field 'from_square'."""
        return self._from_square

    @from_square.setter
    def from_square(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'from_square' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'from_square' field must be an integer in [-128, 127]"
        self._from_square = value

    @builtins.property
    def to_square(self):
        """Message field 'to_square'."""
        return self._to_square

    @to_square.setter
    def to_square(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'to_square' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'to_square' field must be an integer in [-128, 127]"
        self._to_square = value

    @builtins.property
    def piece_type(self):
        """Message field 'piece_type'."""
        return self._piece_type

    @piece_type.setter
    def piece_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'piece_type' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'piece_type' field must be an integer in [-128, 127]"
        self._piece_type = value

    @builtins.property
    def captured_piece(self):
        """Message field 'captured_piece'."""
        return self._captured_piece

    @captured_piece.setter
    def captured_piece(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'captured_piece' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'captured_piece' field must be an integer in [-128, 127]"
        self._captured_piece = value

    @builtins.property
    def promotion(self):
        """Message field 'promotion'."""
        return self._promotion

    @promotion.setter
    def promotion(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'promotion' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'promotion' field must be an integer in [-128, 127]"
        self._promotion = value

    @builtins.property
    def is_castling(self):
        """Message field 'is_castling'."""
        return self._is_castling

    @is_castling.setter
    def is_castling(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_castling' field must be of type 'bool'"
        self._is_castling = value

    @builtins.property
    def is_en_passant(self):
        """Message field 'is_en_passant'."""
        return self._is_en_passant

    @is_en_passant.setter
    def is_en_passant(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_en_passant' field must be of type 'bool'"
        self._is_en_passant = value

    @builtins.property
    def is_check(self):
        """Message field 'is_check'."""
        return self._is_check

    @is_check.setter
    def is_check(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_check' field must be of type 'bool'"
        self._is_check = value

    @builtins.property
    def is_checkmate(self):
        """Message field 'is_checkmate'."""
        return self._is_checkmate

    @is_checkmate.setter
    def is_checkmate(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_checkmate' field must be of type 'bool'"
        self._is_checkmate = value

    @builtins.property
    def confidence(self):
        """Message field 'confidence'."""
        return self._confidence

    @confidence.setter
    def confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence = value

    @builtins.property
    def thinking_time(self):
        """Message field 'thinking_time'."""
        return self._thinking_time

    @thinking_time.setter
    def thinking_time(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'thinking_time' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'thinking_time' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._thinking_time = value
