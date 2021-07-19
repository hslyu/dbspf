from sympy import solve, Symbol, nsolve
from sympy.solvers.solveset import nonlinsolve
n=4

x = Symbol('x',real=True)
y = Symbol('y', real=True)
f1=-8e6
f2=-60

for i in range(n):
    rho = (200+(-1)**i*10*(1+i//2))/2e7
    print(rho)
    f1 += 1/(x+rho*y)
    f2 += rho/(x+rho*y)

print(f1)
print(f2)

import time

start = time.time()
a=solve([f1,f2],[x,y])
#a=solve(f1,x)
end = time.time()
print(a)
print('------------')
for sol in a:
    print(sol)
    sol_x=sol[0]
    sol_y=sol[1]
    for i in range(n):
        print(1/(sol_x+(200+10*i)/2e7*sol_y))
    print('------------')
print("time",end-start)

