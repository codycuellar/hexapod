import math

import hexapod.matmath as mm
from hexapod.matmath import Matrix


class Vector(Matrix):
    """A 3D Vector which represents motion or translation in space."""

    def __init__(self, data: list[float] | Matrix | None = None):
        if data is None:
            super().__init__([0.0, 0.0, 0.0])
        elif isinstance(data, Matrix):
            if data.shape != (3,):
                raise ValueError("Matrix must be of shape (3,) to convert to Vector.")
            super().__init__(data._data[0])
        else:
            if len(data) != 3:
                raise ValueError("Vector must be initialized with 3 elements.")
            super().__init__(data)

    def __add__(self, other: "Vector") -> "Vector":
        return Vector((super() + other))

    def __sub__(self, other: "Vector") -> "Vector":
        return Vector((super() - other))

    def __mul__(self, scalar: float) -> "Vector":
        return Vector((super() * scalar))

    def __truediv__(self, scalar: float) -> "Vector":
        return Vector((super() / scalar))

    def __matmul__(self, other: "Vector") -> float:
        result = super().__matmul__(other)
        return result._data[0][0]

    @property
    def length(self) -> float:
        """Calculate the magnitude (length) of the vector."""
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    @property
    def x(self) -> float:
        """Get the x component of the vector."""
        return self._get_value(0, 0)

    @property
    def y(self) -> float:
        """Get the y component of the vector."""
        return self._get_value(0, 1)

    @property
    def z(self) -> float:
        """Get the z component of the vector."""
        return self._get_value(0, 2)

    def cross(self, vec: "Vector") -> "Vector":
        """
        Get the cross product of another vector and this vector.
        :param vec: The vector to multiply.
        :return: The cross product Vector.
        :raises TypeError: TypeError if a non-vector is passed.
        """
        if not isinstance(vec, Vector):
            raise TypeError("Operand must be an instance of Vector")
        return Vector(
            [
                self.y * vec.z - self.z * vec.y,
                self.z * vec.x - self.x * vec.z,
                self.x * vec.y - self.y * vec.x,
            ]
        )

    def normalize(self) -> "Vector":
        """
        Normalize the vector to a unit vector with magnitude equal to 1.
        :return: A new normalized Vector instance.
        :raises ValueError: ValueError if the vector has zero length.
        """
        if self.length == 0:
            raise ValueError(
                f"Cannot normalize a zero-length vector with coordinates ({self.x}, {self.y}, {self.z})"
            )
        return self / self.length


class Rotation(Matrix):
    """
    A 3D rotation matrix for rotation about each axis. Rotations are specified in
    the xyz order.
    """

    @staticmethod
    def degrees(x_deg: float, y_deg: float, z_deg: float) -> "Rotation":
        """
        Create a Rotation instance from Euler angles in degrees.
        :param x_deg: Rotation about the x axis in degrees.
        :param y_deg: Rotation about the y axis in degrees.
        :param z_deg: Rotation about the z axis in degrees.
        :return: The Rotation instance.
        """
        return Rotation.radians(
            math.radians(x_deg), math.radians(y_deg), math.radians(z_deg)
        )

    @staticmethod
    def radians(x_rad: float, y_rad: float, z_rad: float) -> "Rotation":
        """
        Create a Rotation instance from Euler angles in radians.
        :param x_rad: Rotation about the x axis in radians.
        :param y_rad: Rotation about the y axis in radians.
        :param z_rad: Rotation about the z axis in radians.
        :return: The Rotation instance.
        """
        cos_x, sin_x = math.cos(x_rad), math.sin(x_rad)
        cos_y, sin_y = math.cos(y_rad), math.sin(y_rad)
        cos_z, sin_z = math.cos(z_rad), math.sin(z_rad)
        return Rotation(
            [
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
        )

    def __init__(self, data: list[list[float]] | Matrix | None = None):
        if data is None:
            super().__init__(Matrix.identity(3).as_list(copy=False))
        elif isinstance(data, Matrix):
            if data.shape != (3, 3):
                raise ValueError("Matrix must be of shape (3,3) to convert to Matrix.")
            super().__init__(data._data)
        else:
            if len(data) != 3 and len(data[0]) != 3:
                raise ValueError("Matrix must be initialized with a 3x3 list.")
            super().__init__(data)

    def __add__(self, _) -> "Vector":
        raise NotImplementedError("Rotation addition is not defined.")

    def __sub__(self, _) -> "Vector":
        raise NotImplementedError("Rotation subtraction is not defined.")

    def __mul__(self, _) -> "Vector":
        raise NotImplementedError("Rotation multiplication is not defined.")

    def __truediv__(self, _) -> "Vector":
        raise NotImplementedError("Rotation division is not defined.")

    def __matmul__(self, other: "Rotation | Vector"):
        result = super() @ other
        if isinstance(other, Rotation):
            return Rotation(result.as_list(copy=False))  # type: ignore
        elif isinstance(other, Vector):
            return Vector(result._get_row(0))
        else:
            raise TypeError(f"Cannot matmul Rotation with {type(other)}")


class Transform(Matrix):
    """
    A 3D transformation matrix for translation and rotation which can
    be used for simple transformations in 3D space. This class supports
    creating transformations from translation and rotation parameters,
    combining transformations, and applying them to 3D Points.
    """

    @staticmethod
    def create(
        rotation: "Rotation | Matrix | None" = None,
        translation: "Vector | Matrix | None" = None,
    ):
        # Build 4x4 homogeneous matrix
        if not isinstance(rotation, Rotation):
            rotation = Rotation(rotation)
        if not isinstance(translation, Vector):
            translation = Vector(translation)

        return Transform(Transform._combine_transforms(rotation, translation))

    @staticmethod
    def _combine_transforms(rotation: Rotation, translation: Vector):
        return [
            [
                rotation._get_value(0, 0),
                rotation._get_value(0, 1),
                rotation._get_value(0, 2),
                translation.x,
            ],
            [
                rotation._get_value(1, 0),
                rotation._get_value(1, 1),
                rotation._get_value(1, 2),
                translation.y,
            ],
            [
                rotation._get_value(2, 0),
                rotation._get_value(2, 1),
                rotation._get_value(2, 2),
                translation.z,
            ],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def __init__(self, data: list[list[float]] | Matrix | None = None):
        if data is None:
            super().__init__(Transform._combine_transforms(Rotation(), Vector()))
        elif isinstance(data, Matrix):
            if data.shape != (4, 4):
                raise ValueError(
                    "Matrix must be of shape (4,4) to convert to Transform."
                )
            super().__init__(data._data)
        else:
            if len(data) != 4 or len(data[0]) != 4:
                raise ValueError("Matrix must be initialized with a 4x4 list.")
            super().__init__(data)

    def __matmul__(self, other) -> "Transform | Vector":
        if not isinstance(other, Matrix):
            other = Matrix(other)

        if other.shape == (4, 4):
            return Transform(super() @ other)
        elif other.shape == (4,):
            return Vector(super() @ other)
        elif other.shape == (3,):
            return Vector(super() @ Matrix(other._get_row(0) + [1.0]))
        else:
            raise ValueError(
                f"Right hand of matrix multiply with Transform produced unexpected shape {other.shape}"
            )

    @property
    def rotation(self) -> Rotation:
        return Rotation(self[:3, :3])

    @property
    def translation(self) -> Vector:
        return Vector(self[0:3, 3])

    def rotate(self, rotation: Rotation) -> "Transform":
        return Transform(self @ Transform.create(rotation=rotation))

    def translate(self, translation: Vector) -> "Transform":
        return Transform(self @ Transform.create(translation=translation))

    def inverse(self) -> "Transform":
        rot_inv = self.rotation.inverse()
        return Transform.create(rot_inv, -(rot_inv @ self.translation))


class Frame:
    """
    A 3D coordinate frame which can be a child of another frame. This is the
    core class for building a hierarchical coordinate system of relational entities.
    Each frame has a position and rotation relative to its parent frame. The world
    position and rotation can be computed by traversing up the parent chain.
    """

    def __init__(
        self,
        origin: Vector = Vector(),
        rotation: Rotation = Rotation.identity(),
        parent: "Frame | None" = None,
    ):
        """
        :param pos: The local position of the frame origin within the parent frame context.
        :param rotation: The local rotation of the frame within the parent frame context.
        :param parent: The parent frame of this frame. The only frame without a parennt
                       should be the world frame.
        """
        self.origin = origin
        self.parent = parent
        self.rotation = rotation

    def rotation(self, rotation: Rotation):
        """Set the rotation matrix for the frame."""
        if not isinstance(rotation, Rotation):
            raise TypeError("Rotation must be an instance of Rotation")
        self.rotation = rotation

    def get_global_position(self) -> "Frame":
        """Get the global position of the frame."""
        if self.parent is None:
            return self
        parent_global = self.parent.get_global_position()
        transformed_point = Transform(
            translation=Vector(self.as_list()),
            rotation=self.rotation,
        ).apply(parent_global)
        return transformed_point

    def get_global_rotation(self) -> Rotation:
        """Get the global rotation of the frame."""
        if self.parent is None:
            return self.rotation
        parent_global_rotation = self.parent.get_global_rotation()
        combined_rotation_matrix = matmult(parent_global_rotation.m, self.rotation.m)
        return Rotation.from_matrix(combined_rotation_matrix)

    def copy(self) -> "Frame":
        """Create a copy of the frame."""
        new_frame = Frame(self.x, self.y, self.z, self.parent)
        new_frame.rotation = self.rotation.copy()
        return new_frame
