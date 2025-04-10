import math

class Coord3D:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "Coord3D({:.1f}, {:.1f}, {:.1f})".format(self.x, self.y, self.z)
    
    def __repr__(self):
        return self.__str__()

    def copy(self):
        return Coord3D(self.x, self.y, self.z)

    def as_list(self):
        return [self.x, self.y, self.z]
    
    def mirror(self, axis):
        if axis == 'xy':
            return Coord3D(self.x, self.y, -self.z)
        elif axis == 'yz':
            return Coord3D(-self.x, self.y, self.z)
        elif axis == 'xz':
            return Coord3D(self.x, -self.y, self.z)
        else:
            raise ValueError("Invalid axis. Use 'xy', 'yz', or 'xz'.")
    
    def translate(self, offset: 'Coord3D'):
        return Coord3D(self.x + offset.x, self.y + offset.y, self.z + offset.z)
    
    def rotate(self, angle: float, axis: str):
        """
        Rotate the coordinate around the specified axis by the given angle in degrees.
        """
        angle = math.radians(angle) 
        if axis == 'x':
            y_new = self.y * math.cos(angle) - self.z * math.sin(angle)
            z_new = self.y * math.sin(angle) + self.z * math.cos(angle)
            return Coord3D(self.x, y_new, z_new)
        elif axis == 'y':
            x_new = self.x * math.cos(angle) + self.z * math.sin(angle)
            z_new = -self.x * math.sin(angle) + self.z * math.cos(angle)
            return Coord3D(x_new, self.y, z_new)
        elif axis == 'z':
            x_new = self.x * math.cos(angle) - self.y * math.sin(angle)
            y_new = self.x * math.sin(angle) + self.y * math.cos(angle)
            return Coord3D(x_new, y_new, self.z)
        else:
            raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")

class Coord2D:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def __str__(self):
        return "Coord2D({:.1f}, {:.1f})".format(self.x, self.y)

    def as_list(self):
        return [self.x, self.y]

class Transform3D:
    """
    A 3D transformation matrix for translation and rotation which can
    be used for simple transformations in 3D space. This class supports
    creating transformations from translation and rotation parameters,
    combining transformations, and applying them to 3D coordinates.
    """

    @staticmethod
    def from_matrix(matrix: list) -> 'Transform3D':
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
        result = [[0]*4 for _ in range(4)]
        for i in range(4):
            for j in range(4):
                result[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
        return result

    def __init__(self, translation=None, x_rotation=0, y_rotation=0, z_rotation=0):
        tx, ty, tz = (translation.x, translation.y, translation.z) if translation else (0, 0, 0)
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
            [0, 0, 0, 1]
        ]

        rot_y = [
            [cos_y, 0, sin_y, 0],
            [0, 1, 0, 0],
            [-sin_y, 0, cos_y, 0],
            [0, 0, 0, 1]
        ]

        rot_z = [
            [cos_z, -sin_z, 0, 0],
            [sin_z, cos_z, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]

        # Combine rotations
        rotation = self._matrix_multiply(self._matrix_multiply(rot_x, rot_y), rot_z)

        # Add translation
        self.matrix = [
            [rotation[0][0], rotation[0][1], rotation[0][2], tx],
            [rotation[1][0], rotation[1][1], rotation[1][2], ty],
            [rotation[2][0], rotation[2][1], rotation[2][2], tz],
            [0, 0, 0, 1]
        ]

    def dot(self, other: 'Transform3D') -> 'Transform3D':
        """
        Multiply two transformation matrices.
        """
        result = self._matrix_multiply(self.matrix, other.matrix)
        return Transform3D.from_matrix(result)

    def invert(self) -> 'Transform3D':
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
            [0, 0, 0, 1]
        ]

        return Transform3D.from_matrix(inverted_matrix)

    def apply(self, coord: Coord3D) -> Coord3D:
        if not isinstance(coord, Coord3D):
            raise TypeError("Input must be an instance of Coord3D")
        x, y, z = coord.x, coord.y, coord.z
        m = self.matrix
        new_x = m[0][0]*x + m[0][1]*y + m[0][2]*z + m[0][3]
        new_y = m[1][0]*x + m[1][1]*y + m[1][2]*z + m[1][3]
        new_z = m[2][0]*x + m[2][1]*y + m[2][2]*z + m[2][3]
        return Coord3D(new_x, new_y, new_z)
