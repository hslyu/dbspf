import math

def shortest_path(cur, visited):
    # base case
    if visited == (1<<n)-1:
        ##
        next_node[cur][visited-1] = start
        return dist[cur][start]

    if cache[cur][visited-1] != None:
        return cache[cur][visited-1]

    cache[cur][visited-1] = INF
    best=-1
    for _next in range(n):
        if not (visited & (1<<_next)):
            prev=cache[cur][visited-1]
            cache[cur][visited-1] = \
            min(cache[cur][visited-1], dist[cur][_next]+shortest_path(_next, visited+(1<<_next)))
            if prev!=cache[cur][visited-1]:
                next_node[cur][visited-1]=_next

    return cache[cur][visited-1]

def find_route(start, next_node):
    route=[]
    route.append(start)

    cur=start
    mask=1<<start
    while mask != (1<<n)-1:
        dest = next_node[cur][mask-1]
        route.append(dest)
        mask += 1<<dest
        cur=dest

    return route

if __name__ == "__main__":
    n = 5
    dist = [
        [0, 2, 5, 3, 7],
        [2, 0, 10, 6, 6],
        [5, 10, 0, 4, 8],
        [3, 6, 4, 0, 12],
        [7, 6, 8, 12, 0]
    ]

    # Cache and route are indexed by bit masking
    cache = [[None for _ in range(1<<n)] for _ in range(n)]
    next_node = [[ -1 for _ in range(1<<n)] for _ in range(n)]

    INF = 99999
    start=0

    assert start<n, "Initial node index should be lower than total number of node - 1"
    for i in range(len(dist)):
        print(dist[i])

    print(shortest_path(start, 1<<start))
    print(find_route(start, next_node))
