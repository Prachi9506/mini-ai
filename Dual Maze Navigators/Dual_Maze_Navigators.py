import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
from collections import deque

# -------------------------------
# MAZE
# -------------------------------
maze = [
    list("###########"),
    list("#A..k....B#"),
    list("#.#.#.##..#"),
    list("#...k.....#"),
    list("###########")
]

R, C = len(maze), len(maze[0])
dirs = [(1,0), (-1,0), (0,1), (0,-1)]

startA = startB = None
for i in range(R):
    for j in range(C):
        if maze[i][j] == 'A': startA = (i, j)
        if maze[i][j] == 'B': startB = (i, j)

visited = [[False]*C for _ in range(R)]
visited[startA[0]][startA[1]] = True
visited[startB[0]][startB[1]] = True

pathA = set([startA])
pathB = set([startB])
keys_collected = []

def bfs(start, store):
    q = deque([start])
    while q:
        x, y = q.popleft()
        for dx, dy in dirs:
            nx, ny = x+dx, y+dy
            if 0 <= nx < R and 0 <= ny < C:
                if maze[nx][ny] != '#' and not visited[nx][ny]:
                    visited[nx][ny] = True
                    store.add((nx, ny))
                    q.append((nx, ny))
                    if maze[nx][ny] == 'k':
                        keys_collected.append((nx, ny))

bfs(startA, pathA)
bfs(startB, pathB)

# -------------------------------
# DRAWING (BETTER GRID)
# -------------------------------
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_xlim(0, C)
ax.set_ylim(0, R)
ax.invert_yaxis()
ax.set_aspect("equal")
ax.axis("off")

def draw_cell(x, y, color):
    rect = patches.Rectangle((y, x), 1, 1, linewidth=0.8,
                             edgecolor="#222222", facecolor=color,
                             joinstyle='round')
    ax.add_patch(rect)

# COLORS
COLORS = {
    "wall": "#000000",         # black
    "pathA": "#ff6b6b",        # soft red
    "pathB": "#6b8bff",        # soft blue
    "empty": "#f6f6f6",        # light grey
    "agentA": "#d00000",       # strong red
    "agentB": "#0022cc",       # strong blue
    "key": "#ffea00"           # bright yellow
}

# Draw grid
for i in range(R):
    for j in range(C):
        cell = maze[i][j]

        if cell == '#':
            draw_cell(i, j, COLORS["wall"])
        elif (i, j) == startA:
            draw_cell(i, j, COLORS["agentA"])
        elif (i, j) == startB:
            draw_cell(i, j, COLORS["agentB"])
        elif (i, j) in keys_collected:
            draw_cell(i, j, COLORS["key"])
        elif (i, j) in pathA:
            draw_cell(i, j, COLORS["pathA"])
        elif (i, j) in pathB:
            draw_cell(i, j, COLORS["pathB"])
        else:
            draw_cell(i, j, COLORS["empty"])

# LABELS (A, B, keys)
for x, y in [startA]:
    ax.text(y+0.5, x+0.65, "A", color="white",
            ha="center", va="center", fontsize=16, weight='bold')

for x, y in [startB]:
    ax.text(y+0.5, x+0.65, "B", color="white",
            ha="center", va="center", fontsize=16, weight='bold')

for x, y in keys_collected:
    ax.text(y+0.5, x+0.6, "K", color="black",
            ha="center", va="center", fontsize=16, weight='bold')

plt.tight_layout()
plt.show()
