import numpy as np

a = np.array([[1, 2],
               [3,4],
               [5,6]])

r,f = np.unravel_index(np.argmax(a, axis=None), a.shape)
print(r,f)
