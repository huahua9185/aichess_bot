# generated from rosidl_generator_py/resource/_idl.py.em
# with input from chess_interfaces:msg/PieceDetection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PieceDetection(type):
    """Metaclass of message 'PieceDetection'."""

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
                'chess_interfaces.msg.PieceDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__piece_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__piece_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__piece_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__piece_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__piece_detection

            from geometry_msgs.msg import Point
            if Point.__class__._TYPE_SUPPORT is None:
                Point.__class__.__import_type_support__()

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

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


class PieceDetection(metaclass=Metaclass_PieceDetection):
    """Message class 'PieceDetection'."""

    __slots__ = [
        '_header',
        '_position_3d',
        '_position_2d',
        '_piece_type',
        '_piece_color',
        '_confidence',
        '_roi_image',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'position_3d': 'geometry_msgs/Point',
        'position_2d': 'geometry_msgs/Point',
        'piece_type': 'int8',
        'piece_color': 'int8',
        'confidence': 'float',
        'roi_image': 'sensor_msgs/Image',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'Point'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from geometry_msgs.msg import Point
        self.position_3d = kwargs.get('position_3d', Point())
        from geometry_msgs.msg import Point
        self.position_2d = kwargs.get('position_2d', Point())
        self.piece_type = kwargs.get('piece_type', int())
        self.piece_color = kwargs.get('piece_color', int())
        self.confidence = kwargs.get('confidence', float())
        from sensor_msgs.msg import Image
        self.roi_image = kwargs.get('roi_image', Image())

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
        if self.position_3d != other.position_3d:
            return False
        if self.position_2d != other.position_2d:
            return False
        if self.piece_type != other.piece_type:
            return False
        if self.piece_color != other.piece_color:
            return False
        if self.confidence != other.confidence:
            return False
        if self.roi_image != other.roi_image:
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
    def position_3d(self):
        """Message field 'position_3d'."""
        return self._position_3d

    @position_3d.setter
    def position_3d(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'position_3d' field must be a sub message of type 'Point'"
        self._position_3d = value

    @builtins.property
    def position_2d(self):
        """Message field 'position_2d'."""
        return self._position_2d

    @position_2d.setter
    def position_2d(self, value):
        if __debug__:
            from geometry_msgs.msg import Point
            assert \
                isinstance(value, Point), \
                "The 'position_2d' field must be a sub message of type 'Point'"
        self._position_2d = value

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
    def piece_color(self):
        """Message field 'piece_color'."""
        return self._piece_color

    @piece_color.setter
    def piece_color(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'piece_color' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'piece_color' field must be an integer in [-128, 127]"
        self._piece_color = value

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
    def roi_image(self):
        """Message field 'roi_image'."""
        return self._roi_image

    @roi_image.setter
    def roi_image(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'roi_image' field must be a sub message of type 'Image'"
        self._roi_image = value
