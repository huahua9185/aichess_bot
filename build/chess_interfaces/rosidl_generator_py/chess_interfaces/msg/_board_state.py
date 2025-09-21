# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chess_interfaces:msg/BoardState.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'board_squares'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_BoardState(type):
    """Metaclass of message 'BoardState'."""

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
                'chess_interfaces.msg.BoardState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__board_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__board_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__board_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__board_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__board_state

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

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


class BoardState(metaclass=Metaclass_BoardState):
    """Message class 'BoardState'."""

    __slots__ = [
        '_header',
        '_board_squares',
        '_square_positions_3d',
        '_white_to_move',
        '_castling_rights',
        '_en_passant_square',
        '_halfmove_clock',
        '_fullmove_number',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'board_squares': 'int8[64]',
        'square_positions_3d': 'geometry_msgs/Point[64]',
        'white_to_move': 'boolean',
        'castling_rights': 'boolean[4]',
        'en_passant_square': 'int8',
        'halfmove_clock': 'int32',
        'fullmove_number': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('int8'), 64),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'), 64),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('boolean'), 4),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        if 'board_squares' not in kwargs:
            self.board_squares = numpy.zeros(64, dtype=numpy.int8)
        else:
            self.board_squares = numpy.array(kwargs.get('board_squares'), dtype=numpy.int8)
            assert self.board_squares.shape == (64, )
        from geometry_msgs.msg import Point
        self.square_positions_3d = kwargs.get(
            'square_positions_3d',
            [Point() for x in range(64)]
        )
        self.white_to_move = kwargs.get('white_to_move', bool())
        self.castling_rights = kwargs.get(
            'castling_rights',
            [bool() for x in range(4)]
        )
        self.en_passant_square = kwargs.get('en_passant_square', int())
        self.halfmove_clock = kwargs.get('halfmove_clock', int())
        self.fullmove_number = kwargs.get('fullmove_number', int())

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
        if all(self.board_squares != other.board_squares):
            return False
        if self.square_positions_3d != other.square_positions_3d:
            return False
        if self.white_to_move != other.white_to_move:
            return False
        if self.castling_rights != other.castling_rights:
            return False
        if self.en_passant_square != other.en_passant_square:
            return False
        if self.halfmove_clock != other.halfmove_clock:
            return False
        if self.fullmove_number != other.fullmove_number:
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
    def board_squares(self):
        """Message field 'board_squares'."""
        return self._board_squares

    @board_squares.setter
    def board_squares(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.int8, \
                "The 'board_squares' numpy.ndarray() must have the dtype of 'numpy.int8'"
            assert value.size == 64, \
                "The 'board_squares' numpy.ndarray() must have a size of 64"
            self._board_squares = value
            return
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
                 len(value) == 64 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= -128 and val < 128 for val in value)), \
                "The 'board_squares' field must be a set or sequence with length 64 and each value of type 'int' and each integer in [-128, 127]"
        self._board_squares = numpy.array(value, dtype=numpy.int8)

    @builtins.property
    def square_positions_3d(self):
        """Message field 'square_positions_3d'."""
        return self._square_positions_3d

    @square_positions_3d.setter
    def square_positions_3d(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
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
                 len(value) == 64 and
                 all(isinstance(v, Point) for v in value) and
                 True), \
                "The 'square_positions_3d' field must be a set or sequence with length 64 and each value of type 'Point'"
        self._square_positions_3d = value

    @builtins.property
    def white_to_move(self):
        """Message field 'white_to_move'."""
        return self._white_to_move

    @white_to_move.setter
    def white_to_move(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'white_to_move' field must be of type 'bool'"
        self._white_to_move = value

    @builtins.property
    def castling_rights(self):
        """Message field 'castling_rights'."""
        return self._castling_rights

    @castling_rights.setter
    def castling_rights(self, value):
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
                 len(value) == 4 and
                 all(isinstance(v, bool) for v in value) and
                 True), \
                "The 'castling_rights' field must be a set or sequence with length 4 and each value of type 'bool'"
        self._castling_rights = value

    @builtins.property
    def en_passant_square(self):
        """Message field 'en_passant_square'."""
        return self._en_passant_square

    @en_passant_square.setter
    def en_passant_square(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'en_passant_square' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'en_passant_square' field must be an integer in [-128, 127]"
        self._en_passant_square = value

    @builtins.property
    def halfmove_clock(self):
        """Message field 'halfmove_clock'."""
        return self._halfmove_clock

    @halfmove_clock.setter
    def halfmove_clock(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'halfmove_clock' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'halfmove_clock' field must be an integer in [-2147483648, 2147483647]"
        self._halfmove_clock = value

    @builtins.property
    def fullmove_number(self):
        """Message field 'fullmove_number'."""
        return self._fullmove_number

    @fullmove_number.setter
    def fullmove_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fullmove_number' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'fullmove_number' field must be an integer in [-2147483648, 2147483647]"
        self._fullmove_number = value
