import math


def matmult(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    """
    Multiply two matrices. The number of columns in the first matrix must match
    the number of rows in the second matrix.

    Arguments:
        a: The first matrix as a list of lists.
        b: The second matrix as a list of lists.

    Returns:
        The resulting matrix as a list of lists.

    Raises:
        ValueError: If the matrices cannot be multiplied due to shape mismatch.
    """
    if len(a[0]) != len(b):
        raise ValueError(
            f"Cannot multiply matrices: {len(a[0])} columns in A != {len(b)} rows in B"
        )
    result = [[0.0] * len(b[0]) for _ in range(len(a))]
    for i in range(len(a)):
        for j in range(len(b[0])):
            result[i][j] = sum(a[i][k] * b[k][j] for k in range(len(a[0])))
    return result


class Point:
    """A 3D point in space."""

    def __init__(self, x: float = 0, y: float = 0, z: float = 0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "{}({:.4f}, {:.4f}, {:.4f})".format(
            self.__class__.__name__, self.x, self.y, self.z
        )

    def __repr__(self):
        return self.__str__()

    def __add__(self, vec: "Vector") -> "Point":
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector")
        return Point(self.x + vec.x, self.y + vec.y, self.z + vec.z)

    def __sub__(self, vec: "Vector") -> "Point":
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector")
        return Point(self.x - vec.x, self.y - vec.y, self.z - vec.z)

    def mirror(self, axis: str) -> "Point":
        """Mirror the point across the given plane.

        Arguments:
            axis -- The axis plane to mirror across. 'xy', 'xz', 'yz'

        Raises:
            ValueError: ValueError if invalid axis is passed

        Returns:
            The mirrored Point.
        """
        if axis == "xy":
            return Point(self.x, self.y, -self.z)
        if axis == "yz":
            return Point(-self.x, self.y, self.z)
        if axis == "xz":
            return Point(self.x, -self.y, self.z)
        raise ValueError("Invalid axis. Use 'xy', 'yz', or 'xz'.")

    def mirrored(self, axis: str) -> "Point":
        m = self.copy()
        return m.mirror(axis)

    def copy(self) -> "Point":
        """Create a copy of the instance.

        Returns:
            The new instance.
        """
        return self.__class__(self.x, self.y, self.z)

    def to_list(self) -> list[float]:
        """Get the coordinates as a list.

        Returns:
            [x, y, z]
        """
        return [self.x, self.y, self.z]


class Vector(Point):
    """A vector in 3D space."""

    @staticmethod
    def from_point(point: Point) -> "Vector":
        return Vector(point.x, point.y, point.z)

    def __mul__(self, scalar: float) -> "Vector":
        """
        Scale the vector by a scalar value.

        Arguments:
            scalar -- The value by which to scale the vector. This is a float
                      that multiplies each component of the vector.

        Returns:
            A new Vector instance scaled by the given scalar.
        """
        return Vector(self.x * scalar, self.y * scalar, self.z * scalar)

    def dot(self, vec: "Vector") -> float:
        """Multiply two vectors with dot matrix multiplication.

        Arguments:
            vec -- the vector to multiply.

        Raises:
            TypeError: TypeError if a non-vector is passed.

        Returns:
            The resulting scalar.
        """
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector")
        return self.x * vec.x + self.y * vec.y + self.z * vec.z

    def cross(self, vec: "Vector") -> "Vector":
        """Get the cross product of two vectors.

        Arguments:
            vec -- the vector to multiply.

        Raises:
            TypeError: TypeError if a non-vector is passed.

        Returns:
            The cross product Vector.
        """
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector")
        return Vector(
            self.y * vec.z - self.z * vec.y,
            self.z * vec.x - self.x * vec.z,
            self.x * vec.y - self.y * vec.x,
        )

    def length(self) -> float:
        """Get the magnitude of the vector."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def normalize(self) -> "Vector":
        """Normalize the vector to a unit vector with magnitude equal to 1."""
        length = self.length()
        if length == 0:
            raise ValueError(
                f"Cannot normalize a zero-length vector with coordinates ({self.x}, {self.y}, {self.z})"
            )
        self.x /= length
        self.y /= length
        self.z /= length
        return self

    def normalized(self) -> "Vector":
        normalized = self.copy()
        return normalized.normalize()

    def as_point(self) -> Point:
        """Convert the vector to a Point."""
        return Point(self.x, self.y, self.z)


class Rotation:
    """
     A 3D rotation matrix for rotation about each axis. Rotations are specified in
    the xyz order.
    """

    @staticmethod
    def from_matrix(matrix: list) -> "Rotation":
        """
        Create a Rotation object from a 3x3 matrix.

        Raises:
            ValueError: If the input matrix is not 3x3.
        """
        if len(matrix) != 3 or any(len(row) != 3 for row in matrix):
            raise ValueError("Input matrix must be 3x3.")
        rotation = Rotation()
        rotation.m = matrix
        return rotation

    def __init__(self, x=0, y=0, z=0):
        """Initialize a rotation about each axis in terms of degrees.

        Arguments:
            x -- rotation about the x-axis in degrees
            y -- rotation about the y axis in degrees
            z -- rotation about the z axis in degrees
        """
        self.x = math.radians(x)
        self.y = math.radians(y)
        self.z = math.radians(z)
        self.m = self._build_matrix()

    def _build_matrix(self):
        cos_x, sin_x = math.cos(self.x), math.sin(self.x)
        cos_y, sin_y = math.cos(self.y), math.sin(self.y)
        cos_z, sin_z = math.cos(self.z), math.sin(self.z)
        return [
            [
                cos_y * cos_z,
                cos_z * sin_x * sin_y - cos_x * sin_z,
                sin_x * sin_z + cos_x * cos_z * sin_y,
            ],
            [
                cos_y * sin_z,
                cos_x * cos_z + sin_x * sin_y * sin_z,
                cos_x * sin_y * sin_z - cos_z * sin_x,
            ],
            [
                -sin_y,
                cos_y * sin_x,
                cos_x * cos_y,
            ],
        ]

    def transpose(self):
        """Transposes the rotation matrix in place.

        Returns:
            The updated Rotation.
        """
        new_m = [
            [self.m[0][0], self.m[1][0], self.m[2][0]],
            [self.m[0][1], self.m[1][1], self.m[2][1]],
            [self.m[0][2], self.m[1][2], self.m[2][2]],
        ]
        self.m = new_m
        return self

    def transposed(self):
        """Returns a transposed rotation.

        Returns:
            The new transposed rotation.
        """
        r2 = Rotation.from_matrix(self.m)
        return r2.transpose()


class Transform:
    """
    A 3D transformation matrix for translation and rotation which can
    be used for simple transformations in 3D space. This class supports
    creating transformations from translation and rotation parameters,
    combining transformations, and applying them to 3D Points.
    """

    @staticmethod
    def from_matrix(matrix: list) -> "Transform":
        """
        Create a Transform3D object from a 4x4 matrix.
        """
        transform = Transform()
        transform.m = matrix
        return transform

    def __init__(
        self,
        translation: Vector | None = None,
        rotation: Rotation | None = None,
    ):
        translation = translation or Vector(0, 0, 0)
        rotation = rotation or Rotation(0, 0, 0)

        # Add translation
        self.m = [
            [rotation.m[0][0], rotation.m[0][1], rotation.m[0][2], translation.x],
            [rotation.m[1][0], rotation.m[1][1], rotation.m[1][2], translation.y],
            [rotation.m[2][0], rotation.m[2][1], rotation.m[2][2], translation.z],
            [0, 0, 0, 1],
        ]

    def dot(self, other: "Transform") -> "Transform":
        """
        Multiply two transformation matrices.
        """
        result = matmult(self.m, other.m)
        return Transform.from_matrix(result)

    def invert(self) -> "Transform":
        """Invert the transformation in place."""
        # Extract the rotation (upper-left 3x3) and translation (last column)
        rot_t = Transform(rotation=self.get_rotation().transpose())
        translation = self.get_translation()

        # Compute the inverted translation
        inverted_translation = rot_t.apply(translation * -1)

        # Construct the inverted matrix
        self.m = [
            [rot_t.m[0][0], rot_t.m[0][1], rot_t.m[0][2], inverted_translation.x],
            [rot_t.m[1][0], rot_t.m[1][1], rot_t.m[1][2], inverted_translation.y],
            [rot_t.m[2][0], rot_t.m[2][1], rot_t.m[2][2], inverted_translation.z],
            [0, 0, 0, 1],
        ]

        return self

    def inverted(self) -> "Transform":
        """
        Invert the transformation matrix and return a new instance of the
        inverted transform.
        """
        t = Transform.from_matrix(self.m)
        return t.invert()

    def apply(self, point: Point | Vector) -> Point:
        """Apply the transform to a point.

        Arguments:
            point -- The point to transform to a new location.

        Raises:
            TypeError: TypeError if pass a non Point.

        Returns:
            The new Point.
        """
        if not isinstance(point, (Point, Vector)):
            raise TypeError("Input must be an instance of Point or Vector")
        new_point = Point(
            self.m[0][0] * point.x
            + self.m[0][1] * point.y
            + self.m[0][2] * point.z
            + self.m[0][3],
            self.m[1][0] * point.x
            + self.m[1][1] * point.y
            + self.m[1][2] * point.z
            + self.m[1][3],
            self.m[2][0] * point.x
            + self.m[2][1] * point.y
            + self.m[2][2] * point.z
            + self.m[2][3],
        )
        return new_point

    def get_rotation(self):
        return Rotation.from_matrix([row[:3] for row in self.m[:3]])

    def get_vector(self):
        return Vector(self.m[0][3], self.m[1][3], self.m[2][3])
