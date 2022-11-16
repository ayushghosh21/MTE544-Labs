#%matplotlib inline
import matplotlib.pyplot as plt
f = 'map_maze_1.pgm'
with open(f, 'rb') as pgmf:
    im = plt.imread(pgmf)
    
plt.imshow(im)

import yaml
with open("map_maze_1.yaml", 'r') as stream:
    data_loaded = yaml.safe_load(stream)

origin = data_loaded['origin']
print(origin)
res = data_loaded['resolution']

plt.plot(-1*origin[0]/res, -1*origin[1]/res, marker="o", markersize=20, markeredgecolor="red",
markerfacecolor="green")
