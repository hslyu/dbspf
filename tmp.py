import math
import pdb

n = 5
dist = [
    [0, 2, 5, 3, 7],
    [2, 0, 10, 6, 6],
    [5, 10, 0, 4, 8],
    [3, 6, 4, 0, 12],
    [7, 6, 8, 12, 0]
]
"""
dist = [
    [0, 2],
    [2, 0],
    ]
"""

# Cache and route are indexed by bit masking
cache = [[None for _ in range(1<<n)] for _ in range(n)]
route = [[ -1 for _ in range(1<<n)] for _ in range(n)]

INF = 99999

def shortest_path2(cur, visited):
    # base case
    if visited == (1<<n)-1:
        ##
        route[cur][visited-1] = 0
        return dist[cur][0]

    if cache[cur][visited-1] != None:
        return cache[cur][visited-1]

    cache[cur][visited-1] = INF
    best=-1
    for _next in range(n):
        if not (visited & (1<<_next)):
            cache[cur][visited-1] = \
            min(cache[cur][visited-1], dist[cur][_next]+shortest_path2(_next, visited+(1<<_next)))

    return cache[cur][visited-1]

if __name__ == "__main__":
    #pdb.set_trace()
    print(shortest_path2(0, 1<<0))
    print(cache)
