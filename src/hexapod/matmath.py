class Matrix:
    """
    Lightweight matrix class supporting basic operations. This class handles
    scalars, 1-D arrays (vectors), and 2-D matrices. It will consume the input
    data as-is without validation, or copying, so make sure you are not passing
    a mutable structure tied to other code.
    """

    @staticmethod
    def identity(size: int) -> "Matrix":
        """Create an identity matrix of given size (N, N)."""
        return Matrix(
            [[1.0 if i == j else 0.0 for j in range(size)] for i in range(size)]
        )

    def __init__(self, data: float | list[float] | list[list[float]]):
        """
        Construct a Matrix (lightweight, assumes valid input).
        Behavior:
        - scalar (float) -> stored as [[value]], ndim=0, shape=()
        - list[float] -> 1-D array (ndim==1, stored as single row 1xN, shape=(N,))
        - list[list[float]] -> 2-D array (ndim==2, shape=(M,N))
        """
        self._data: list[list[float]]

        if isinstance(data, list):
            if len(data) == 0:
                raise ValueError("Cannot initialize Matrix with empty list")

            if isinstance(data[0], list) and isinstance(data[0][0], (float, int)):
                # 2-D: assume list-of-rows with consistent lengths
                self._ndim = 2
                self._data = data  # type: ignore
                self.shape = (len(data), len(data[0]))
            elif isinstance(data[0], (float, int)):
                # 1-D: wrap as single row
                self._ndim = 1
                self._data = [data]  # type: ignore
                self.shape = (len(data),)
            else:
                raise ValueError(
                    f"Invalid list element type for Matrix: {type(data[0])}"
                )
        elif isinstance(data, (float, int)):
            # Scalar
            self._ndim = 0
            self._data = [[float(data)]]
            self.shape = ()
        else:
            raise ValueError(
                f"Invalid data type for Matrix initialization: {type(data)}"
            )

    def __getitem__(self, key) -> "Matrix":
        if self._ndim == 0:
            raise IndexError("Matrix is scalar and cannot be indexed")

        elif isinstance(key, int):
            # single accessor m[i]
            if self._ndim == 1:
                # 1-D array
                return Matrix([self._data[0][key]])
            else:
                # 2-D matrix row
                return Matrix([self._data[key][:]])

        elif isinstance(key, slice):
            # row slice m[i:j]
            if self._ndim == 1:
                # 1-D array slice
                return Matrix(self._data[0][key])
            else:
                # 2-D matrix row slice
                return Matrix([row[:] for row in self._data[key]])

        elif isinstance(key, tuple):
            if not self._ndim == 2:
                raise IndexError("Can only use tuple indexing on 2-D matrices")

            # row/col slice m[i,j] or m[i:j, k:l]
            row_key, col_key = key
            if isinstance(row_key, int):
                rows = [row_key]
            else:
                rows = range(*row_key.indices(self.nrows()))

            if isinstance(col_key, int):
                cols = [col_key]
            else:
                cols = range(*col_key.indices(self.ncols()))

            return Matrix([[self._get_value(r, c) for c in cols] for r in rows])

        else:
            raise TypeError(f"Invalid index type {type(key)}")

    def __add__(self, other) -> "Matrix":
        return self._elementwise(other, "add")

    def __sub__(self, other) -> "Matrix":
        return self._elementwise(other, "sub")

    def __mul__(self, other) -> "Matrix":
        return self._elementwise(other, "mul")

    def __truediv__(self, other) -> "Matrix":
        if isinstance(other, (int, float)) and other == 0:
            raise ValueError("Division by zero")
        return self._elementwise(other, "div")

    def __neg__(self):
        return self * -1

    def __matmul__(self, other) -> "Matrix":
        if not isinstance(other, Matrix):
            other = Matrix(other)

        a_ndim = self._ndim
        b_ndim = other._ndim

        if a_ndim == 0 or b_ndim == 0:
            raise ValueError(
                "Cannot perform matmul on scalar matrices; use * for scalar multiplication."
            )

        a_cols = self.ncols()
        b_rows = other.nrows()

        if a_cols != b_rows:
            raise ValueError(
                f"Cannot multiply matrices: {a_cols} cols in A != {b_rows} rows in B"
            )

        eff_rows = 1 if a_ndim == 1 else self.nrows()
        eff_cols = 1 if b_ndim == 1 else other.ncols()
        result = [[0.0 for _ in range(eff_cols)] for _ in range(eff_rows)]

        for i in range(eff_rows):
            for j in range(eff_cols):
                s = 0.0
                for k in range(a_cols):
                    a_val = self._get_value(i if a_ndim == 2 else 0, k)
                    b_val = other._get_value(k, j if b_ndim == 2 else 0)
                    s += a_val * b_val
                result[i][j] = s

        # Promote result based on input ndim
        if a_ndim == 1 and b_ndim == 1:
            return Matrix(result[0][0])  # scalar
        if a_ndim == 1 and b_ndim == 2:
            return Matrix(result[0])  # 1-D row
        if a_ndim == 2 and b_ndim == 1:
            return Matrix([row[0] for row in result])  # 1-D column

        return Matrix(result)  # 2-D

    def __len__(self) -> int:
        """Return number of rows in the matrix."""
        return self.nrows()

    def __repr__(self) -> str:
        return f"{self.__class__.__name__}({self.as_list()})"

    def __str__(self) -> str:
        return f"{self.as_list()}"

    @property
    def T(self) -> "Matrix":
        """Get the transposed view of this matrix."""
        if self._ndim == 0:
            return self.copy()
        if self._ndim == 1:
            # 1×N becomes N×1
            new_data = [[self._data[0][i]] for i in range(len(self._data[0]))]
            m = Matrix(new_data)
            return m
        # 2-D: swap rows/cols
        new_data = [
            [self._data[j][i] for j in range(len(self._data))]
            for i in range(len(self._data[0]))
        ]
        m = Matrix(new_data)
        return m

    def nrows(self) -> int:
        """Logical number of rows."""
        if self._ndim == 2:
            return len(self._data)
        return self._ndim

    def ncols(self) -> int:
        """Logical number of columns."""
        if self._ndim == 0:
            return 0
        return len(self._data[0])

    def copy(self) -> "Matrix":
        """Get a deep copy of this Matrix."""
        return Matrix([row[:] for row in self._data])

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
        mat = self @ self.T
        return all(
            abs(mat._get_value(i, j) - (1 if i == j else 0)) < tol
            for i in range(3)
            for j in range(3)
        )

    def as_list(self, copy: bool = True):
        """
        Get the raw representation of this Matrix.
        :param copy: If False, will not create a deep copy of the underlyiny
                     data. This can improve speed, but should only be used on
                     intermediary Matrixes where the reference will not be stored,
                     otherwise you may unintentially mutate the Matrix data.
        :return: The raw data
          - float for scalar Matrix
          - list[float] for 1D Matrix
          - list[list[float]] for 2D Matrix.
        """
        if self._ndim == 0:
            return self._data[0][0]
        elif self._ndim == 1:
            if copy:
                return self._get_row(0)[:]
            return self._get_row(0)
        else:
            if copy:
                return [self._get_row(r)[:] for r in range(self.nrows())]
            return [self._get_row(r) for r in range(self.nrows())]

    def _get_row(self, index: int) -> list[float]:
        """Get row at index."""
        if self._ndim == 0:
            raise IndexError("Matrix is scalar")
        return self._data[index]

    def _get_value(self, row: int, col: int) -> float:
        """
        Get the underlying data value at physical (row, col). Since
        all types are stored as list of lists, Scalars will need to
        get m(0, 0), 1D Matrix needs to get m(0, i), and 2D matrix
        m(i, j). This is private due to the internal knowledge needed
        of the ndims.
        """
        return self._data[row][col]

    def _elementwise(self, other, op: str) -> "Matrix":
        """
        Perform element-wise operation with broadcasting where possible.
        :param other: Another Matrix (scalar, 1-D, or 2-D)
        :param op: Operation type: "add", "sub", "mul", "div"
        :return: Resulting Matrix
        """
        if not isinstance(other, Matrix):
            other = Matrix(other)

        # Determine the function
        if op == "add":
            func = lambda x, y: x + y
        elif op == "sub":
            func = lambda x, y: x - y
        elif op == "mul":
            func = lambda x, y: x * y
        elif op == "div":
            func = lambda x, y: x / y if y != 0 else 0.0  # optional zero-handling
        else:
            raise ValueError(f"Unsupported operation '{op}'")

        # Scalar & Any | Any & Scalar
        if self._ndim == 0 or other._ndim == 0:
            # Pick scalar vs matrix
            if self._ndim == 0 and other._ndim == 0:
                return Matrix(func(self._data[0][0], other._data[0][0]))
            scalar, mat = (
                (self._data[0][0], other)
                if self._ndim == 0
                else (other._data[0][0], self)
            )
            result = [
                [func(scalar, mat._get_value(r, c)) for c in range(mat.ncols())]
                for r in range(mat.nrows())
            ]
            # if mat was 1D, flatten to 1D
            return Matrix(result[0] if mat._ndim == 1 else result)

        # 1-D + 1-D
        if self._ndim == 1 and other._ndim == 1:
            if self.ncols() != other.ncols():
                raise ValueError(
                    f"Cannot {op} 1-D arrays of different lengths: {self.ncols()} vs {other.ncols()}"
                )
            return Matrix(
                [
                    func(self._get_value(0, c), other._get_value(0, c))
                    for c in range(self.ncols())
                ]
            )

        # 1-D + 2-D (must be single row)
        if self._ndim == 1 and other._ndim == 2:
            if other.nrows() != 1 or self.ncols() != other.ncols():
                raise ValueError(
                    f"Cannot {op} 1-D array of length {self.ncols()} to 2-D shape {other.shape}"
                )
            return Matrix(
                [
                    [
                        func(self._get_value(0, c), other._get_value(0, c))
                        for c in range(self.ncols())
                    ]
                ]
            )
        if self._ndim == 2 and other._ndim == 1:
            if self.nrows() != 1 or self.ncols() != other.ncols():
                raise ValueError(
                    f"Cannot {op} 1-D array of length {other.ncols()} to 2-D shape {self.shape}"
                )
            return Matrix(
                [
                    [
                        func(self._get_value(0, c), other._get_value(0, c))
                        for c in range(self.ncols())
                    ]
                ]
            )

        # 2-D + 2-D
        if self._ndim == 2 and other._ndim == 2:
            if self.shape != other.shape:
                raise ValueError(
                    f"Cannot {op} 2-D matrices of different shapes: {self.shape} vs {other.shape}"
                )
            return Matrix(
                [
                    [
                        func(self._get_value(r, c), other._get_value(r, c))
                        for c in range(self.ncols())
                    ]
                    for r in range(self.nrows())
                ]
            )

        raise TypeError(f"Unsupported {op} between {self._ndim}D and {other._ndim}D")
