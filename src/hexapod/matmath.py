import math
from typing import overload


class Vector:
    """
    A 3D Vector which represents motion or translation in space, as well as generic 3D
    positions in space.
    """

    def __init__(self, data: list[float]):
        self._data = data

    def __add__(self, other: "Vector") -> "Vector":
        self._ensure_len_eq(other)
        return Vector([self[i] + other[i] for i in range(len(self))])

    def __sub__(self, other: "Vector") -> "Vector":
        self._ensure_len_eq(other)
        return Vector([self[i] - other[i] for i in range(len(self))])

    def __mul__(self, scalar: float) -> "Vector":
        return Vector([x * scalar for x in self._data])

    def __truediv__(self, scalar: float) -> "Vector":
        return Vector([x / scalar for x in self._data])

    def __matmul__(self, other: "Vector") -> float:
        self._ensure_len_eq(other)
        return sum(self[i] * other[i] for i in range(len(self)))

    def __neg__(self) -> "Vector":
        return self * -1

    def __len__(self):
        return len(self._data)

    def __getitem__(self, i: int):
        return self._data[i]

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.to_list()})"

    def __str__(self) -> str:
        return f"{self.to_list()}"

    def length(self):
        return math.sqrt(sum(self[i] ** 2 for i in range(len(self))))

    def distance_to(self, other: "Vector") -> float:
        return abs((self - other).length())

    def to_list(self):
        return self._data[:]

    def copy(self):
        return Vector(self._data[:])

    def _ensure_len_eq(self, other: "Vector"):
        if len(self) != len(other):
            raise ValueError(
                f"Lengths of vectors must be the same, a={len(self)}, b={len(other)}"
            )


class Matrix:
    """
    Lightweight matrix class supporting basic operations. This class handles
    scalars, 1-D arrays (vectors), and 2-D matrices. It will consume the input
    data as-is without validation, or copying, so make sure you are not passing
    a mutable structure tied to other code.
    """

    @classmethod
    def identity(cls, N: int) -> "Matrix":
        return cls([[1.0 if i == j else 0.0 for j in range(N)] for i in range(N)])

    @classmethod
    def zero(cls, N: int) -> "Matrix":
        return cls([[0.0 for _ in range(N)] for _ in range(N)])

    def __init__(self, data: list[list[float]]):
        self.N = len(data)
        if not all([len(row) == self.N for row in data]):
            raise ValueError("Matrix must be square (NxN).")
        self._data = [row[:] for row in data]

    def __getitem__(self, key: tuple[int, int]):
        return self._data[key[0]][key[1]]

    def __setitem__(self, key: tuple[int, int], val: float):
        self._data[key[0]][key[1]] = val

    @overload
    def __matmul__(self, other: "Matrix") -> "Matrix": ...
    @overload
    def __matmul__(self, other: Vector) -> Vector: ...
    def __matmul__(self, other):
        if isinstance(other, Matrix):
            if self.N != other.N:
                raise ValueError("Matrix sizes do not match for multiplication.")
            result = Matrix.zero(self.N)
            for i in range(self.N):
                for j in range(self.N):
                    result[i, j] = sum(
                        a * b
                        for a, b in zip(self.row(i).to_list(), other.col(j).to_list())
                    )
            return result
        else:
            if len(other) != self.N:
                raise ValueError(
                    f"Matrix size of {self.N} does not match vector {len(other)}"
                )
            return Vector(
                [
                    sum(self[i, k] * other[k] for k in range(self.N))
                    for i in range(self.N)
                ]
            )

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.to_list()})"

    def __str__(self) -> str:
        return f"{self.to_list()}"

    @property
    def T(self) -> "Matrix":
        """Get the transposed view of this matrix."""
        new_data = [
            [self._data[j][i] for j in range(len(self._data))]
            for i in range(len(self._data[0]))
        ]
        m = Matrix(new_data)
        return m

    def row(self, i: int) -> Vector:
        return Vector(self._data[i][:])

    def col(self, j: int) -> Vector:
        return Vector([self._data[i][j] for i in range(self.N)])

    def inverse(self) -> "Matrix":
        """
        Check that the Matrix is orthogonal and then return its transpose,
        otherwise throw an error.
        """
        if self.is_orthogonal():
            return self.T
        else:
            raise ValueError("Matrix is not orthogonal and cannot be inverted.")

    def is_orthogonal(self, tol: float = 1e-6) -> bool:
        """
        Check if this matrix is orthogonal, or that it's transpose
        dot multiplied with itself renders very close to an identity
        Matrix.
        """
        identity = Matrix.identity(self.N)
        mat = self @ self.T
        for i in range(self.N):
            for j in range(self.N):
                if abs(mat[i, j] - identity[i, j]) >= tol:
                    return False
        return True

    def to_list(self):
        return [row[:] for row in self._data]

    def copy(self) -> "Matrix":
        return Matrix(self.to_list())
