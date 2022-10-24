import enum
import numpy as np

class Color(enum.Enum):
    OTHER = 0, np.array([0, 0, 0], dtype=np.uint8)
    GREEN = 1, np.array([0, 255, 0], dtype=np.uint8)
    MAGENTA = 2, np.array([255, 0, 255], dtype=np.uint8)
    BLUE = 3, np.array([255, 0, 0], dtype=np.uint8)
    ORANGE = 4, np.array([0, 127, 255], dtype=np.uint8)
    WHITE = 5, np.array([255, 255, 255], dtype=np.uint8)
    BLACK = 6, np.array([64, 64, 64], dtype=np.uint8)

    def __new__(cls, value, color):
        enum = object.__new__(cls)
        enum._value_ = value
        enum.color = color
        return enum

    def __int__(self):
        return self.value