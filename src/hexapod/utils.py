import math

class Coord3D:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def as_list(self):
        return [self.x, self.y, self.z]
    
    def mirror(self, axis):
        if axis == 'x':
            return Coord3D(-self.x, self.y, self.z)
        elif axis == 'y':
            return Coord3D(self.x, -self.y, self.z)
        elif axis == 'z':
            return Coord3D(self.x, self.y, -self.z)
        else:
            raise ValueError("Invalid axis. Use 'x', 'y', or 'z'.")
    
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

    def as_list(self):
        return [self.x, self.y]

