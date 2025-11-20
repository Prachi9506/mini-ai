# Two Cleaning Bots Simulation
# Features:
# - Grid with obstacles & dirty cells
# - A* path planning
# - Greedy load-balanced task assignment
# - Multi-agent cleaned coverage simulation
# - Visualization + efficiency score

import heapq
import random
from collections import deque
import matplotlib.pyplot as plt

# -----------------------
# CONFIGURATION
# -----------------------
random.seed(42)
GRID_H, GRID_W = 15, 20
NUM_DIRTY = 30
OBSTACLE_PROB = 0.08

# Create grid: 0 free, 1 obstacle
grid = [[0 for _ in range(GRID_W)] for _ in range(GRID_H)]
for r in range(GRID_H):
    for c in range(GRID_W):
        if random.random() < OBSTACLE_PROB:
            grid[r][c] = 1

# Agent start positions (must be free)
agent_starts = [(0,0), (GRID_H-1, GRID_W-1)]
for (sr, sc) in agent_starts:
    grid[sr][sc] = 0

# Generate dirty cells
free_cells = [(r,c) for r in range(GRID_H) for c in range(GRID_W)
              if grid[r][c] == 0 and (r,c) not in agent_starts]

dirty = set(random.sample(free_cells, min(NUM_DIRTY, len(free_cells))))

# -----------------------
# A* SEARCH
# -----------------------
def astar(start, goal, grid):
    H, W = len(grid), len(grid[0])
    def h(a,b): return abs(a[0]-b[0]) + abs(a[1]-b[1])

    open_heap = []
    gscore = {start: 0}
    fscore = {start: h(start, goal)}
    heapq.heappush(open_heap, (fscore[start], start))
    came = {}
    neighbors = [(1,0),(-1,0),(0,1),(0,-1)]

    while open_heap:
        _, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came:
                current = came[current]
                path.append(current)
            return list(reversed(path))

        for dr, dc in neighbors:
            nr, nc = current[0] + dr, current[1] + dc
            if 0 <= nr < H and 0 <= nc < W and grid[nr][nc] == 0:
                neigh = (nr,nc)
                tentative = gscore[current] + 1
                if tentative < gscore.get(neigh, 1e9):
                    came[neigh] = current
                    gscore[neigh] = tentative
                    fscore[neigh] = tentative + h(neigh, goal)
                    heapq.heappush(open_heap, (fscore[neigh], neigh))

    return None

# -----------------------
# TASK ASSIGNMENT (GREEDY + LOAD BALANCING)
# -----------------------
def assign_tasks(agent_starts, dirty, grid):
    assignments = {0: [], 1: []}
    est_pos = {0: agent_starts[0], 1: agent_starts[1]}
    est_len = {0: 0, 1: 0}
    remaining = set(dirty)

    while remaining:
        best = None
        best_score = float('inf')

        for cell in remaining:
            for ag in [0,1]:
                path = astar(est_pos[ag], cell, grid)
                if path is None:
                    continue
                score = est_len[ag] + len(path) - 1
                if score < best_score:
                    best = (cell, ag, path)
                    best_score = score

        if best is None: break
        
        cell, ag, path = best
        assignments[ag].append(cell)
        est_pos[ag] = cell
        est_len[ag] = best_score
        remaining.remove(cell)

    return assignments

assignments = assign_tasks(agent_starts, dirty, grid)

# -----------------------
# ROUTE PLANNING
# -----------------------
def plan_full_path(start, tasks, grid):
    pos = start
    full = [pos]
    for t in tasks:
        p = astar(pos, t, grid)
        if p:
            full.extend(p[1:])
            pos = t
    return full

paths = {
    0: plan_full_path(agent_starts[0], assignments[0], grid),
    1: plan_full_path(agent_starts[1], assignments[1], grid)
}

# -----------------------
# SIMULATION
# -----------------------
cleaned = set()
agent_positions = {0: agent_starts[0], 1: agent_starts[1]}
agent_steps = {0: 0, 1: 0}
agent_path_idx = {0: 0, 1: 0}

history_positions = {0: [], 1: []}
max_steps = max(len(paths[0]), len(paths[1])) + 200

for step in range(max_steps):
    moved = False
    for ag in [0,1]:
        if agent_path_idx[ag] + 1 < len(paths[ag]):
            agent_path_idx[ag] += 1
            agent_positions[ag] = paths[ag][agent_path_idx[ag]]
            agent_steps[ag] += 1
            moved = True

        if agent_positions[ag] in dirty:
            cleaned.add(agent_positions[ag])

        history_positions[ag].append(agent_positions[ag])

    if not moved: break

total_steps = agent_steps[0] + agent_steps[1]
efficiency = len(cleaned) / total_steps

# -----------------------
# VISUALIZATION
# -----------------------
fig, ax = plt.subplots(figsize=(10,7))
ax.set_title(f"Cleaning Bots — Cleaned {len(cleaned)}/{len(dirty)} — Efficiency {efficiency:.3f}")
ax.set_xlim(-0.5, GRID_W-0.5)
ax.set_ylim(-0.5, GRID_H-0.5)
ax.set_xticks(range(GRID_W))
ax.set_yticks(range(GRID_H))
ax.grid(True)
ax.set_aspect('equal')
ax.invert_yaxis()

# obstacles
for r in range(GRID_H):
    for c in range(GRID_W):
        if grid[r][c] == 1:
            ax.plot(c, r, marker='s', markersize=18, color='black')

# dirty cells
for (r,c) in dirty:
    ax.plot(c, r, marker='s', markersize=12, color='brown')

# cleaned cells
for (r,c) in cleaned:
    ax.plot(c, r, marker='o', markersize=8, markerfacecolor='white', markeredgecolor='black')

# agent paths
colors = ['orange', 'blue']
for ag in [0,1]:
    px = [p[1] for p in history_positions[ag]]
    py = [p[0] for p in history_positions[ag]]
    ax.plot(px, py, linewidth=2, color=colors[ag])
    # ending position
    ax.plot(px[-1], py[-1], marker='X', markersize=12, color=colors[ag])

plt.show()

# Summary
print("=== SUMMARY ===")
for ag in [0,1]:
    print(f"Agent {ag}: {len(assignments[ag])} tasks, Route length = {len(paths[ag])}")
print(f"Total steps = {total_steps}")
print(f"Cleaned = {len(cleaned)} / {len(dirty)}")
print(f"Efficiency = {efficiency:.4f}")
