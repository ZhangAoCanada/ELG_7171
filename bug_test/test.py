import numpy as np
from tanbug import TangentBug

tt = TangentBug(10 + 10)

a = np.array([1, 1])
b = np.array([  [2, 0],  [4, 0]  ])

test = tt.LineSegDist(a, b)

print(test)