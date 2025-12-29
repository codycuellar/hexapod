import math
from typing import overload

from hexapod.matmath import Vector, Matrix


class Vec3d(Vector):
    @staticmethod
    def zero():
        return Vec3d(0, 0, 0)

    def __init__(self, x: float, y: float, z: float):
        return super().__init__([x, y, z])

    def __add__(self, other: "Vec3d") -> "Vec3d":
        return Vec3d(*super().__add__(other)._data)

    def __sub__(self, other: "Vec3d") -> "Vec3d":
        self._ensure_len_eq(other)
        return Vec3d(*super().__sub__(other)._data)

    def __mul__(self, scalar: float) -> "Vec3d":
        return Vec3d(*super().__mul__(scalar)._data)

    def __truediv__(self, scalar: float) -> "Vec3d":
        return Vec3d(*super().__truediv__(scalar)._data)

    def __neg__(self):
        return Vec3d(*super().__neg__()._data)

    @property
    def x(self):
        return self[0]

    @property
    def y(self):
        return self[1]

    @property
    def z(self):
        return self[2]

    def cross(self, other: "Vec3d"):
        return Vec3d(
            self[1] * other[2] - self[2] * other[1],
            self[2] * other[0] - self[0] * other[2],
            self[0] * other[1] - self[1] * other[0],
        )

    def normalize(self) -> "Vec3d":
        l = self.length()
        if l == 0:
            return Vec3d(0, 0, 0)
        return Vec3d(*(self * (1 / l)))

    def to_transform(self):
        return Transform.create(translation=self)


class Rotation(Matrix):
    """
    A 3D rotation matrix for rotation about each axis. Rotations are specified in
    the xyz order.
    """

    @staticmethod
    def identity() -> "Rotation":
        """Always returns a 3x3 identity rotation matrix."""
        return Rotation([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

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

    def __init__(self, data: list[list[float]]):
        if not (len(data) == 3 and len(data[0]) == 3):
            # super's init ensures 3x3, so just need to check initial row here
            raise ValueError("Rotation matrix must be of size 3x3")
        super().__init__(data)

    @overload
    def __matmul__(self, other: "Rotation") -> "Rotation": ...
    @overload
    def __matmul__(self, other: Vec3d) -> Vec3d: ...
    def __matmul__(self, other):
        if isinstance(other, "Rotation"):
            return Rotation(super().__matmul__(other).as_list())
        else:
            return Vec3d(*super().__matmul__(other).as_list())

    @property
    def T(self) -> "Rotation":
        return Rotation(super().T.as_list())

    def inverse(self) -> "Rotation":
        return Rotation(super().inverse().as_list())

    def to_transform(self) -> "Transform":
        return Transform.create(rotation=self)


class Transform(Matrix):
    """
    A 3D transformation matrix for translation and rotation which can
    be used for simple transformations in 3D space. This class supports
    creating transformations from translation and rotation parameters,
    combining transformations, and applying them to 3D Points.
    """

    @staticmethod
    def identity():
        return Transform(Matrix.identity(4).as_list())

    @staticmethod
    def create(
        rotation: Rotation | None = None,
        translation: Vec3d | None = None,
    ) -> "Transform":
        # Build 4x4 homogeneous matrix
        if not isinstance(rotation, Rotation):
            rotation = Rotation.identity()
        if not isinstance(translation, Vec3d):
            translation = Vec3d.zero()

        return Transform(Transform._combine_transforms(rotation, translation))

    @staticmethod
    def _combine_transforms(rot: Rotation, trans: Vec3d):
        return [
            [rot[0, 0], rot[0, 1], rot[0, 2], trans.x],
            [rot[1, 0], rot[1, 1], rot[1, 2], trans.y],
            [rot[2, 0], rot[2, 1], rot[2, 2], trans.z],
            [0.0, 0.0, 0.0, 1.0],
        ]

    def __init__(self, data: list[list[float]]):
        if len(data) != 4 or len(data[0]) != 4:
            # super's init ensures 3x3, so just need to check initial row here
            raise ValueError("Rotation matrix must be of size 4x4")
        super().__init__(data)

    def __matmul__(self, other: "Transform") -> "Transform":
        return Transform(super().__matmul__(other)._data)

    @property
    def rotation(self) -> Rotation:
        return Rotation([self.row(i).as_list() for i in range(3)])

    @property
    def translation(self) -> Vec3d:
        vec = self.col(3)
        return Vec3d(vec[0], vec[1], vec[2])

    def rotate(self, rotation: Rotation) -> "Transform":
        return self @ rotation.to_transform()

    def translate(self, translation: Vec3d) -> "Transform":
        return self @ translation.to_transform()

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
        origin: Vec3d | None = None,
        rotation: Rotation | None = None,
        parent: "Frame | None" = None,
    ):
        """
        :param pos: The local position of the frame origin within the parent frame context.
        :param rotation: The local rotation of the frame within the parent frame context.
        :param parent: The parent frame of this frame. The only frame without a parennt
                       should be the world frame.
        """
        self._origin = origin or Vec3d.zero()
        self.transform = Transform.create(rotation, origin)
        self.parent = parent

    @property
    def origin(self):
        return self._origin

    @origin.setter
    def origin(self, value: Vec3d):
        self._origin = value
        self.transform = Transform.create(self.transform.rotation, value)

    @property
    def rotation(self):
        return self.transform.rotation

    @rotation.setter
    def rotation(self, rotation: Rotation):
        self.transform = Transform.create(rotation, self.transform.translation)

    def move_local(self, delta: Vec3d) -> "Frame":
        """
        Update the frame origin by a delta.
        :param delta: The vector to move the frame origin by.
        """
        self.transform = self.transform.translate(delta)
        return self

    def rotate_local(self, delta: Rotation) -> "Frame":
        """
        Update the frame origin by a delta.
        :param delta: The vector to move the frame origin by.
        """
        self.transform = self.transform.rotate(delta)
        return self

    def get_position_in_frame(self, target: "Frame | None" = None) -> Vec3d:
        """
        Get this frame's origin position relative to a parent target. If no
        target is supplied, we traverse the parent tree to the top-most (usually
        World space).
        :param target: The target parent frame to calculate the relative position from.
        :return: The position of this frame's origin within in the target frame.
        """
        relative_t = self._get_transform_to(target)
        pos = relative_t @ self._origin.to_transform()
        return pos.translation

    def to_local_position(
        self, position: Vec3d, target: "Frame | None" = None
    ) -> Vec3d:
        """
        Convert a position relative to a parent frame into local frame coordinate.
        :param position: The position relative to a parent frame to convert to this
                         frame's context.
        :param target: The target parent frame to calculate the relative position from.
        :return: The converted coordinate position
        """
        relative_t = self._get_transform_to(target) @ self.transform
        pos_inv = relative_t.inverse() @ position.to_transform()
        return pos_inv.translation

    def to_frame_position(self, position: Vec3d, target: "Frame|None" = None) -> Vec3d:
        """
        Docstring for to_frame_position
        :param position: Description
        :param target: Description
        """
        relative_t = self._get_transform_to(target)
        pos = relative_t @ position.to_transform()
        return pos.translation

    def copy(self):
        return Frame(self.origin, self.rotation)

    def _get_transform_to(self, target: "Frame | None" = None) -> Transform:
        """
        Get the transform from a target parent frame to the local frame.
        :param target: The final target frame to find when traversing parents.
        :return: The transform, not including the current frame's transform.
        """
        node = self.parent
        transform = Transform.identity()

        while node is not None:
            transform = node.transform @ transform
            if node is target:
                break
            node = node.parent

        if target and not node:
            # we didnt' find the parent
            raise RuntimeError("Target frame not found in parent hierarchy.")

        return transform
