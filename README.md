# pathfinding.py
A collection of pathfinding algorithms.

## Preface
Many artificial intelligence (AI) problems can be abstracted into the problem of finding a path in a directed graph.
- E.g., the agent is in one state; it has a set of deterministic actions it can carry out and wants to get to a goal state.

## Purpose
The aim of this project is to review pathfinding algorithms and implement them in Python.

### Notes
- Evaluating performance
    - Completeness
    - Optimality
    - Time/space complexity
        - $b$: the maximum branching factor (finite)
        - $d$: the depth of the shallowest goal node (finite)
        - $m$: the maximum path length (may be finite)
- Uninformed search (with no costs)
    - Breath-first search (BFS)
        - BFS treats the frontier as a queue (FIFO); expands the shallowest node in the frontier.
        - Complete if the branching factor is finite.
        - Optimal.
        - Time complexity: $O(b^d)$
        - Space complexity: $O(b^d)$
    - Depth-first search (DFS)
        - DFS treats the frontier as a stack (LILO); expands the deepest node in the frontier.
        - Complete if the graph is finite and does not contain cycles.
        - Not optimal.
        - Time complexity: $O(b^m)$
        - Space complexity: $O(bm)$
- Uninformed search (with costs)
    - Lowest-cost-first search (uniform cost search)
        - A variation of Dijkstra's algorithm.
        - Treats the frontier as a priority queue ordered by path cost; expands the node with the lowest total path cost in the frontier.
        - $f(n) = g(n)$
        - Complete if every step cost exceeds $\epsilon > 0$.
        - Optimal.
        - Time complexity: $O(b^{1 + \lfloor C^* / \epsilon \rfloor}$, where $C^*$ is the cost of the optimal path
        - Space complexity: $O(b^{1 + \lfloor C^* / \epsilon \rfloor}$
- Informed search
    - Informed search refers to a search strategy using domain-specific knowledge.
    - It treats the frontier as a priority queue ordered by $f(n)$.
    - Let $f(n)$ be the estimate of the cost of the cheapest path from the start node to the goal node through node $n$
        - $f(n) = g(n) + h(n)$
        - $g(n)$ is the cost of the path from the start node to node $n$.
        - Heuristic $h(n)$ is the estimated cost of the cheapest path from node $n$ to a goal node.
            - $h(n)$ is arbitrary, non-negative, and problem-specific.
            - $h(n) = 0$ if $n$ is a goal node.
    - A* search (Dijkstra's with heuristics)
        - $f(n) = g(n) + h(n)$
        - Complete if all arc costs exceed some $\epsilon > 0$ and $b$ is finite.
        - Optimal if the heuristic is admissible, all arc costs exceeds some $\epsilon > 0$, and $b$ is finite.
            - A search hueristic $h(n)$ is admissible if it is never an overestimate of the cost from node $n$ to a goal node; that is, $\forall n \ h(n) \leq h^*(n)$.
        - Time complexity: $O(b^m)$
        - Space complexity: $O(b^m)$
