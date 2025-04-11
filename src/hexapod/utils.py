import math


class Vector:
    @staticmethod
    def from_coord(coord: "Coord"):
        return Vector(coord.x, coord.y, coord.z)

    @staticmethod
    def from_a_to_b(a: "Coord", b: "Coord"):
        return Vector(b.x - a.x, b.y - a.y, b.z - a.z)

    def __init__(self, x: float, y: float, z: float = 0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "Vector3D({:.4f}, {:.4f}, {:.4f})".format(self.x, self.y, self.z)

    def __repr__(self):
        return self.__str__()

    def __add__(self, vec: "Vector"):
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector3D")
        return Coord(self.x + vec.x, self.y + vec.y, self.z + vec.z)

    def __subract__(self, vec: "Vector"):
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector3D")
        return Coord(self.x - vec.x, self.y - vec.y, self.z - vec.z)

    def dot(self, vec: "Vector"):
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector3D")
        return self.x * vec.x + self.y * vec.y + self.z * vec.z

    def cross(self, vec: "Vector"):
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector3D")
        return Vector(
            self.y * vec.z - self.z * vec.y,
            self.z * vec.x - self.x * vec.z,
            self.x * vec.y - self.y * vec.x,
        )

    def scale(self, scalar: float):
        return Vector(self.x * scalar, self.y * scalar, self.z * scalar)

    def copy(self):
        return self.__class__(self.x, self.y, self.z)

    def length(self) -> float:
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self) -> "Vector":
        length = self.length()
        if length == 0:
            raise ValueError("Cannot normalize a zero-length vector")
        return Vector(self.x / length, self.y / length, self.z / length)

    def as_list(self):
        return [self.x, self.y, self.z]


class Coord:
    def __init__(self, x: float, y: float, z: float = 0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "Coord3D({:.4f}, {:.4f}, {:.4f})".format(self.x, self.y, self.z)

    def __add__(self, vec: Vector):
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector3D")
        return Coord(self.x + vec.x, self.y + vec.y, self.z + vec.z)

    def __subtract__(self, vec: Vector):
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector3D")
        return Coord(self.x - vec.x, self.y - vec.y, self.z - vec.z)

    def copy(self):
        return Coord(self.x, self.y, self.z)

    def as_list(self):
        return [self.x, self.y, self.z]

    def mirror(self, axis):
        if axis == "xy":
            return Coord(self.x, self.y, -self.z)
        elif axis == "yz":
            return Coord(-self.x, self.y, self.z)
        elif axis == "xz":
            return Coord(self.x, -self.y, self.z)
        else:
            raise ValueError("Invalid axis. Use 'xy', 'yz', or 'xz'.")


class Transform3D:
    """
    A 3D transformation matrix for translation and rotation which can
    be used for simple transformations in 3D space. This class supports
    creating transformations from translation and rotation parameters,
    combining transformations, and applying them to 3D coordinates.
    """

    @staticmethod
    def from_matrix(matrix: list) -> "Transform3D":
        """
        Create a Transform3D object from a 4x4 matrix.
        """
        transform = Transform3D()
        transform.matrix = matrix
        return transform

    @staticmethod
    def _matrix_multiply(a: list, b: list) -> list:
        """
        Multiply two 4x4 matrices.
        """
        result = [[0] * 4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                result[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
        return result

    def __init__(
        self,
        translation: Vector | None = None,
        x_rotation=0,
        y_rotation=0,
        z_rotation=0,
    ):
        tx, ty, tz = (
            (translation.x, translation.y, translation.z) if translation else (0, 0, 0)
        )
        x_angle = math.radians(x_rotation)
        y_angle = math.radians(y_rotation)
        z_angle = math.radians(z_rotation)

        cos_x, sin_x = math.cos(x_angle), math.sin(x_angle)
        cos_y, sin_y = math.cos(y_angle), math.sin(y_angle)
        cos_z, sin_z = math.cos(z_angle), math.sin(z_angle)

        # Rotation matrices for each axis
        rot_x = [
            [1, 0, 0, 0],
            [0, cos_x, -sin_x, 0],
            [0, sin_x, cos_x, 0],
            [0, 0, 0, 1],
        ]

        rot_y = [
            [cos_y, 0, sin_y, 0],
            [0, 1, 0, 0],
            [-sin_y, 0, cos_y, 0],
            [0, 0, 0, 1],
        ]

        rot_z = [
            [cos_z, -sin_z, 0, 0],
            [sin_z, cos_z, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]

        # Combine rotations
        rotation = self._matrix_multiply(self._matrix_multiply(rot_x, rot_y), rot_z)

        # Add translation
        self.matrix = [
            [rotation[0][0], rotation[0][1], rotation[0][2], tx],
            [rotation[1][0], rotation[1][1], rotation[1][2], ty],
            [rotation[2][0], rotation[2][1], rotation[2][2], tz],
            [0, 0, 0, 1],
        ]

    def dot(self, other: "Transform3D") -> "Transform3D":
        """
        Multiply two transformation matrices.
        """
        result = self._matrix_multiply(self.matrix, other.matrix)
        return Transform3D.from_matrix(result)

    def invert(self) -> "Transform3D":
        """
        Invert the transformation matrix.
        """
        # Extract the rotation (upper-left 3x3) and translation (last column)
        rotation = [row[:3] for row in self.matrix[:3]]
        translation = [row[3] for row in self.matrix[:3]]

        # Transpose the rotation matrix
        rotation_transposed = [[rotation[j][i] for j in range(3)] for i in range(3)]

        # Compute the inverted translation
        inverted_translation = [
            -sum(rotation_transposed[i][j] * translation[j] for j in range(3))
            for i in range(3)
        ]

        # Construct the inverted matrix
        inverted_matrix = [
            rotation_transposed[0] + [inverted_translation[0]],
            rotation_transposed[1] + [inverted_translation[1]],
            rotation_transposed[2] + [inverted_translation[2]],
            [0, 0, 0, 1],
        ]

        return Transform3D.from_matrix(inverted_matrix)

    def apply(self, coord: Coord) -> Coord:
        if not isinstance(coord, Coord):
            raise TypeError("Input must be an instance of Coord3D")
        x, y, z = coord.x, coord.y, coord.z
        m = self.matrix
        new_x = m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3]
        new_y = m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3]
        new_z = m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3]
        return Coord(new_x, new_y, new_z)
