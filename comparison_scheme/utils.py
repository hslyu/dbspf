from scipy.stats import rice, rayleigh

# https://docs.scipy.org/doc/scipy/reference/generated/scipy.stats.rice.html 
# K is the ratio between the power in the direct path and the power in the other, scattered, paths
# rice.rvs(K, size=1000) -> random sample of rician distribution.
rv = rice.rvs(10**1.24, size=10, scale=1)
rice.rvs(10**1.24, size=10, scale=1)

rv = rayleigh.rvs(size=20, scale = 1 / 2**.5)
print(rv)

import numpy as np

a = np.random.randn(10,2)
print(a)
