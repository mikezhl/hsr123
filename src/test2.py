import numpy as np

a=[1,2,3]
b=np.append(a, [a[-1]], axis=0)
print(b)

