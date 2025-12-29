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
            return Rotation(super().__matmul__(other).to_list())
        else:
            return Vec3d(*super().__matmul__(other).as_list())

    @property
    def T(self) -> "Rotation":
        return Rotation(super().T.to_list())

    def inverse(self) -> "Rotation":
        return Rotation(super().inverse().to_list())

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
        return Transform(Matrix.identity(4).to_list())

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

    @overload
    def __matmul__(self, other: "Transform") -> "Transform": ...
    @overload
    def __matmul__(self, other: Vec3d) -> Vec3d: ...
    def __matmul__(self, other):
        if isinstance(other, Transform):
            return Transform(super().__matmul__(other)._data)
        else:
            vec = super().__matmul__(Vector(other.to_list() + [1.0]))
            return Vec3d(*vec.to_list()[:3])

    @property
    def rotation(self) -> Rotation:
        return Rotation([self.row(i).to_list() for i in range(3)])

    @rotation.setter
    def rotation(self, rotation: Rotation):
        for row in range(3):
            for col in range(3):
                self[row, col] = rotation[row, col]

    @property
    def translation(self) -> Vec3d:
        return Vec3d(*self.col(3).to_list())

    @translation.setter
    def translation(self, translation: Vec3d):
        for i in range(3):
            self._data[i][3] = translation[i]

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
        position: Vec3d | None = None,
        rotation: Rotation | None = None,
        parent: "Frame | None" = None,
    ):
        """
        :param origin:
            The local position of the frame origin within the parent frame
            context or relative to (0,0,0) if no parent.
        :param rotation:
            The local rotation of the frame within the parent frame context.
        :param parent:
            The parent frame of this frame. The only frame without a parennt
            should be the world frame.
        """
        self._transform = Transform.create(rotation, position)
        self._dirty = False  # if this frame or a parent has changed
        self._parent: "Frame | None" = None
        self._children: list[Frame] = []
        if parent:
            self.parent = parent
        self._global_transform = self._get_global_transform()

    def __repr__(self):
        return f"<Frame origin={self.position} rotation={self.rotation} parent={self.parent is not None}>"

    @property
    def position(self):
        return self._transform.translation

    @position.setter
    def position(self, value: Vec3d):
        self._transform.translation = value
        self._set_dirty()

    @property
    def rotation(self):
        return self._transform.rotation

    @rotation.setter
    def rotation(self, rotation: Rotation):
        self._transform.rotation = rotation
        self._set_dirty()

    @property
    def parent(self):
        return self._parent

    @parent.setter
    def parent(self, parent: "Frame | None"):
        if self._parent is not None:
            self._parent.remove_child(self)

        self._parent = parent
        self._set_dirty()

        if parent is not None:
            parent.set_child(self)

    def set_child(self, child: "Frame"):
        if child not in self._children:
            self._children.append(child)

    def remove_child(self, child: "Frame"):
        self._children.remove(child)

    def move(self, delta: Vec3d) -> "Frame":
        """
        Translate the frame by a delta. This moves the origin and affects all
        children.
        :param delta: The delta vector to move the frame by.
        """
        self._transform = self._transform.translate(delta)
        self._set_dirty()
        return self

    def rotate(self, rotation: Rotation) -> "Frame":
        """
        Rotate the rame about its own origin. This affects children node's
        positions, and origin remains the same.
        :param delta: The vector to move the frame origin by.
        """
        self._transform = self._transform.rotate(rotation)
        self._set_dirty()
        return self

    def get_global_position(self) -> Vec3d:
        return self._get_global_transform().translation

    def get_local_position_in(self, target: "Frame | None" = None) -> Vec3d:
        pos = self.get_global_position()
        if target:
            pos = target._get_global_transform().inverse() @ pos
        return pos

    def world_to_local(self, world_pos: Vec3d) -> Vec3d:
        return self._get_global_transform().inverse() @ world_pos

    def local_to_world(self, local_pos: Vec3d) -> Vec3d:
        return self._get_global_transform() @ local_pos

    def copy(self):
        return Frame(self.position, self.rotation)

    def _get_global_transform(self) -> Transform:
        if self._dirty:
            if self.parent:
                self._global_transform = (
                    self.parent._get_global_transform() @ self._transform
                )
            else:
                self._global_transform = self._transform
            self._dirty = False

        return self._global_transform

    def _set_dirty(self):
        if not self._dirty:
            self._dirty = True
            for child in self._children:
                child._set_dirty()
