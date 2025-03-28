class Coord3D:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def as_list(self):
        return [self.x, self.y, self.z]

class Coord2D:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def as_list(self):
        return [self.x, self.y]

