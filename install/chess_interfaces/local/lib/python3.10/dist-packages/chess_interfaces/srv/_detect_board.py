# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chess_interfaces:srv/DetectBoard.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_DetectBoard_Request(type):
    """Metaclass of message 'DetectBoard_Request'."""

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
                'chess_interfaces.srv.DetectBoard_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__detect_board__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__detect_board__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__detect_board__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__detect_board__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__detect_board__request

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DetectBoard_Request(metaclass=Metaclass_DetectBoard_Request):
    """Message class 'DetectBoard_Request'."""

    __slots__ = [
        '_rgb_image',
        '_depth_image',
        '_force_update',
    ]

    _fields_and_field_types = {
        'rgb_image': 'sensor_msgs/Image',
        'depth_image': 'sensor_msgs/Image',
        'force_update': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import Image
        self.rgb_image = kwargs.get('rgb_image', Image())
        from sensor_msgs.msg import Image
        self.depth_image = kwargs.get('depth_image', Image())
        self.force_update = kwargs.get('force_update', bool())

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
        if self.rgb_image != other.rgb_image:
            return False
        if self.depth_image != other.depth_image:
            return False
        if self.force_update != other.force_update:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def rgb_image(self):
        """Message field 'rgb_image'."""
        return self._rgb_image

    @rgb_image.setter
    def rgb_image(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'rgb_image' field must be a sub message of type 'Image'"
        self._rgb_image = value

    @builtins.property
    def depth_image(self):
        """Message field 'depth_image'."""
        return self._depth_image

    @depth_image.setter
    def depth_image(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'depth_image' field must be a sub message of type 'Image'"
        self._depth_image = value

    @builtins.property
    def force_update(self):
        """Message field 'force_update'."""
        return self._force_update

    @force_update.setter
    def force_update(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'force_update' field must be of type 'bool'"
        self._force_update = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_DetectBoard_Response(type):
    """Metaclass of message 'DetectBoard_Response'."""

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
                'chess_interfaces.srv.DetectBoard_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__detect_board__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__detect_board__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__detect_board__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__detect_board__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__detect_board__response

            from chess_interfaces.msg import BoardState
            if BoardState.__class__._TYPE_SUPPORT is None:
                BoardState.__class__.__import_type_support__()

            from geometry_msgs.msg import TransformStamped
            if TransformStamped.__class__._TYPE_SUPPORT is None:
                TransformStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class DetectBoard_Response(metaclass=Metaclass_DetectBoard_Response):
    """Message class 'DetectBoard_Response'."""

    __slots__ = [
        '_success',
        '_error_message',
        '_board_state',
        '_board_transform',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'error_message': 'string',
        'board_state': 'chess_interfaces/BoardState',
        'board_transform': 'geometry_msgs/TransformStamped',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['chess_interfaces', 'msg'], 'BoardState'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'TransformStamped'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.error_message = kwargs.get('error_message', str())
        from chess_interfaces.msg import BoardState
        self.board_state = kwargs.get('board_state', BoardState())
        from geometry_msgs.msg import TransformStamped
        self.board_transform = kwargs.get('board_transform', TransformStamped())

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
        if self.board_state != other.board_state:
            return False
        if self.board_transform != other.board_transform:
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
    def board_state(self):
        """Message field 'board_state'."""
        return self._board_state

    @board_state.setter
    def board_state(self, value):
        if __debug__:
            from chess_interfaces.msg import BoardState
            assert \
                isinstance(value, BoardState), \
                "The 'board_state' field must be a sub message of type 'BoardState'"
        self._board_state = value

    @builtins.property
    def board_transform(self):
        """Message field 'board_transform'."""
        return self._board_transform

    @board_transform.setter
    def board_transform(self, value):
        if __debug__:
            from geometry_msgs.msg import TransformStamped
            assert \
                isinstance(value, TransformStamped), \
                "The 'board_transform' field must be a sub message of type 'TransformStamped'"
        self._board_transform = value


class Metaclass_DetectBoard(type):
    """Metaclass of service 'DetectBoard'."""

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
                'chess_interfaces.srv.DetectBoard')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__detect_board

            from chess_interfaces.srv import _detect_board
            if _detect_board.Metaclass_DetectBoard_Request._TYPE_SUPPORT is None:
                _detect_board.Metaclass_DetectBoard_Request.__import_type_support__()
            if _detect_board.Metaclass_DetectBoard_Response._TYPE_SUPPORT is None:
                _detect_board.Metaclass_DetectBoard_Response.__import_type_support__()


class DetectBoard(metaclass=Metaclass_DetectBoard):
    from chess_interfaces.srv._detect_board import DetectBoard_Request as Request
    from chess_interfaces.srv._detect_board import DetectBoard_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
