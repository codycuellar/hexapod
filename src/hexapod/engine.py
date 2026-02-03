import math
from typing import overload

from hexapod.matmath import Vector, Matrix


class Vec2d(Vector):
    def __init__(self, x: float = 0.0, y: float = 0.0):
        return super().__init__([x, y])

    def __add__(self, other: "Vec2d") -> "Vec2d":  # type: ignore
        return Vec2d(*super().__add__(other)._data)

    def __sub__(self, other: "Vec2d") -> "Vec2d":  # type: ignore
        self._ensure_len_eq(other)
        return Vec2d(*super().__sub__(other)._data)

    def __mul__(self, scalar: float) -> "Vec2d":
        return Vec2d(*super().__mul__(scalar)._data)

    def __truediv__(self, scalar: float) -> "Vec2d":
        return Vec2d(*super().__truediv__(scalar)._data)

    def __neg__(self):
        return Vec2d(*super().__neg__()._data)

    @property
    def x(self):
        return self._data[0]

    @property
    def y(self):
        return self._data[1]

    def normalize(self) -> "Vec2d":
        l = self.length()
        if l == 0:
            return Vec2d(0, 0)
        return Vec2d(*(self * (1 / l)))

    def angle_x(self) -> float:
        return math.atan2(self.y, self.x)

    def degree_x(self) -> float:
        return math.degrees(self.angle_x())

    def angle_y(self) -> float:
        if all([i == 0.0 for i in self._data]):
            return 0.0
        offset = math.pi / 2
        return (self.angle_x() - offset + math.pi) % (2 * math.pi) - math.pi

    def degree_y(self) -> float:
        return math.degrees(self.angle_y())

    def rotate(self, deg: float):
        rad = math.radians(deg)
        cos_r = math.cos(rad)
        sin_r = math.sin(rad)
        return Vec2d(self.x * cos_r - self.y * sin_r, self.x * sin_r + self.y * cos_r)

    def to_3d(self):
        return Vec3d(self._data[0], self._data[1])

    def copy(self) -> "Vec2d":
        return Vec2d(*self._data)


class Vec3d(Vector):
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        super().__init__([x, y, z])
        l = self.length()
        if l == 0:
            self._normalized = [0, 0, 0]
        else:
            self._normalized = [x / l, y / l, z / l]

    def __add__(self, other: "Vec3d") -> "Vec3d":  # type: ignore
        return Vec3d(*super().__add__(other)._data)

    def __sub__(self, other: "Vec3d") -> "Vec3d":  # type: ignore
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

    def elementwise(self, op: str, other: "Vec3d"):
        if op == "mul":
            return Vec3d(self.x * other.x, self.y * other.y, self.z * other.z)
        elif op == "div":
            return Vec3d(self.x / other.x, self.y / other.y, self.z / other.z)
        elif op == "add":
            return Vec3d(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            return Vec3d(self.x - other.x, self.y - other.y, self.z - other.z)

    def cross(self, other: "Vec3d"):
        return Vec3d(
            self[1] * other[2] - self[2] * other[1],
            self[2] * other[0] - self[0] * other[2],
            self[0] * other[1] - self[1] * other[0],
        )

    def normalize(self) -> "Vec3d":
        return Vec3d(*self._normalized)

    def to_transform(self):
        return Transform.create(translation=self)

    def to_2d(self):
        return Vec2d(self.x, self.y)

    def copy(self) -> "Vec3d":
        return Vec3d(*self._data)

    def replace(
        self, x: float | None = None, y: float | None = None, z: float | None = None
    ):
        return Vec3d(
            x if x is not None else self._data[0],
            y if y is not None else self._data[1],
            z if z is not None else self._data[2],
        )


class Rotation(Matrix):
    """
    A 3D rotation matrix for rotation about each axis. Rotations are specified in
    the xyz order.
    """

    @staticmethod
    def identity() -> "Rotation":  # type: ignore
        """Always returns a 3x3 identity rotation matrix."""
        return Rotation([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

    @staticmethod
    def degrees(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> "Rotation":
        """
        Create a Rotation instance from Euler angles in degrees.
        :param x: Rotation about the x axis in degrees.
        :param y: Rotation about the y axis in degrees.
        :param z: Rotation about the z axis in degrees.
        :return: The Rotation instance.
        """
        return Rotation.radians(math.radians(x), math.radians(y), math.radians(z))

    @staticmethod
    def radians(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> "Rotation":
        """
        Create a Rotation instance from Euler angles in radians.
        :param x_rad: Rotation about the x axis in radians.
        :param y_rad: Rotation about the y axis in radians.
        :param z_rad: Rotation about the z axis in radians.
        :return: The Rotation instance.
        """
        cos_x, sin_x = math.cos(x), math.sin(x)
        cos_y, sin_y = math.cos(y), math.sin(y)
        cos_z, sin_z = math.cos(z), math.sin(z)
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

    @staticmethod
    def degrees_vec(vector: Vec3d):
        return Rotation.degrees(vector.x, vector.y, vector.z)

    def __init__(self, data: list[list[float]]):
        if not (len(data) == 3 and len(data[0]) == 3):
            # super's init ensures 3x3, so just need to check initial row here
            raise ValueError("Rotation matrix must be of size 3x3")
        super().__init__(data)

    @overload
    def __matmul__(self, other: "Rotation") -> "Rotation": ...
    @overload
    def __matmul__(self, other: Vec3d) -> Vec3d: ...
    def __matmul__(self, other: "Rotation | Vec3d"):  # type: ignore
        if isinstance(other, Rotation):
            return Rotation(super().__matmul__(other).to_list())
        else:
            return Vec3d(*super().__matmul__(other).to_list())

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
    def identity():  # type: ignore
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
            translation = Vec3d()

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

    def __matmul__(self, other: "Transform") -> "Transform":  # type: ignore
        return Transform(super().__matmul__(other)._data)

    @property
    def rotation(self) -> Rotation:
        return Rotation([self.row(i).to_list()[:3] for i in range(3)])

    @rotation.setter
    def rotation(self, rotation: Rotation):
        for row in range(3):
            for col in range(3):
                self[row, col] = rotation[row, col]

    @property
    def translation(self) -> Vec3d:
        return Vec3d(*self.col(3).to_list()[:3])

    @translation.setter
    def translation(self, translation: Vec3d):
        for i in range(3):
            self._data[i][3] = translation[i]

    def apply_to_point(self, point: Vec3d):
        v = super().__matmul__(Vector([point.x, point.y, point.z, 1.0]))
        w = v[3]
        if abs(w) < 1e-8:
            raise ValueError("Invalid homogeneous point (w ≈ 0)")
        return Vec3d(v[0] / w, v[1] / w, v[2] / w)

    def rotate(self, rotation: Rotation) -> "Transform":
        return self @ rotation.to_transform()

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
        :param origin:
            The local position of the frame origin within the parent frame
            context or relative to (0,0,0) if no parent.
        :param rotation:
            The local rotation of the frame within the parent frame context.
        :param parent:
            The parent frame of this frame. The only frame without a parennt
            should be the world frame.
        """
        self._transform = Transform.create(rotation, origin)
        self._parent: "Frame | None" = None
        self._children: list[Frame] = []
        self._global_transform: Transform | None = None
        if parent:
            self.parent = parent

    def __repr__(self):
        return f"<Frame origin={self.origin} rotation={self.rotation} parent={self.parent is not None}>"

    @property
    def origin(self):
        return self._transform.translation

    @origin.setter
    def origin(self, value: Vec3d):
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

    def has_child(self, child: "Frame"):
        return child in self._children

    def move(self, delta: Vec3d) -> "Frame":
        """
        Translate the frame by a delta. This moves the origin and affects all
        children.
        :param delta: The delta vector to move the frame by.
        """
        self._transform = self._transform @ Transform.create(translation=delta)
        self._set_dirty()
        return self

    def rotate(self, rotation: Rotation) -> "Frame":
        """
        Rotate the frame about its own origin. This affects children node's
        positions, and origin remains the same.
        :param delta: The vector to move the frame origin by.
        """
        self._transform = self._transform @ rotation.to_transform()
        self._set_dirty()
        return self

    def apply_transform(self, t: Transform):
        self._transform = t @ self._transform
        return self

    def rotate_about(self, reference: "Frame", rotation: Rotation) -> "Frame":
        local_pos = reference.world_pos_to_local(self.get_origin_in_world())
        rotated_local_pos = rotation @ local_pos
        self.origin = reference.local_pos_to_world(rotated_local_pos)
        self.rotate(rotation)
        return self

    def get_origin_in_world(self) -> Vec3d:
        return self._local_to_world_t().translation

    def get_origin_in_frame(self, target: "Frame") -> Vec3d:
        p_world = self.get_origin_in_world()
        return target._local_to_world_t().inverse().apply_to_point(p_world)

    def world_pos_to_local(self, world_pos: Vec3d) -> Vec3d:
        return self._local_to_world_t().inverse().apply_to_point(world_pos)

    def local_pos_to_world(self, local_pos: Vec3d) -> Vec3d:
        return self._local_to_world_t().apply_to_point(local_pos)

    def local_pos_to_frame(self, target: "Frame", local_pos: Vec3d):
        pos = self.local_pos_to_world(local_pos)
        return target._local_to_world_t().inverse().apply_to_point(pos)

    def copy(self):
        return Frame(self.origin, self.rotation)

    def as_transform(self):
        return self._transform

    def _local_to_world_t(self):
        if self._global_transform is None:
            if self.parent:
                self._global_transform = self.parent._local_to_world_t() @ self._transform
            else:
                self._global_transform = self._transform
        return self._global_transform

    def _set_dirty(self):
        if self._global_transform:
            self._global_transform = None
            for child in self._children:
                child._set_dirty()
