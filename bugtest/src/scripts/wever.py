import numpy as np


a = np.arange(10)

b = np.roll(a, len(a)//2)

print(b)