import cvxpy as cvx
import numpy as np
import time
a=time.time()

f =  np.array([[3],[3/2]])
lb = np.array([[-1], [0]])
ub = np.array([[2], [3]])

x = cvx.Variable([2,1])
print(x)

obj = cvx.Minimize(-f.T*x)

constraints = [ lb <= x, x<=ub]

prob = cvx.Problem(obj, constraints)
result = prob.solve()

print(time.time()-a)
y = cvx.Variable()
z = y+3
print(z+3)
print(result)
