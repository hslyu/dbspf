import math

def shortest_path(cur, visited):
    # For the array indexing, visited is subtracted by 1.
    # ex) cache[cur][1<<n] ==> overflow!!! last index representing visiting all nodes is 1<<n-1
    # If we visit all node
    if visited == (1<<n)-1:
        # next node will be the start node
        next_node[cur][visited-1] = start
        return dist[cur][start]

    # If the state is already recorded, use it.
    if cache[cur][visited-1] != None:
        return cache[cur][visited-1]

    # If current state is not recorded, calculate it.
    # Initialize
    cache[cur][visited-1] = INF
    # Search next node
    for _next in range(n):
        # If _next is not visited node,
        if not (visited & (1<<_next)):
            # For the shortest path check
            prev = cache[cur][visited-1]
            # If the current path is longer than the path via _next, update the path length 
            cache[cur][visited-1] =  min(cache[cur][visited-1], dist[cur][_next]+shortest_path(_next, visited+(1<<_next)))
            # If the cache is updated, _next is the new next node.
            if prev != cache[cur][visited-1]:
                next_node[cur][visited-1] = _next

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
    n = 3
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
    for j in range(n):
        print(cache[j])
