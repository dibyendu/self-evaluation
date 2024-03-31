from client import *
import numpy as np

epsilon = 0.02
delta = 0.05
beta = 0.95

N_RUNS_PER_K = 1000

LOG_FILE = f'demonstrations/scooping_{epsilon}_{delta}_{beta}.json'

K = [
  {'x': 1, 'y': 2}, # K = 2
  {'x': 1, 'y': 4}, # K = 4
  {'x': 2, 'y': 3}, # K = 6
  {'x': 2, 'y': 4}, # K = 8
  {'x': 2, 'y': 5}, # K = 10
  {'x': 3, 'y': 4}, # K = 12
  {'x': 2, 'y': 7}, # K = 14
  {'x': 4, 'y': 4}, # K = 16

  # {'x': 3, 'y': 3}, # K = 9
  # {'x': 5, 'y': 5}, # K = 25
  # {'x': 8, 'y': 8}, # K = 64
  # {'x': 6, 'y': 6}, # K = 36
  # {'x': 7, 'y': 7}, # K = 49
]



data = {}
if os.path.exists(LOG_FILE):
  f = open(LOG_FILE, 'r')
  data = json.loads(f.read())
  f.close()






data = sorted([(k['x'] * k['y'], data[str(k['x'] * k['y'])]) for k in K], key = lambda x: x[0])
K = [k for k, _ in data]




import matplotlib
matplotlib.use('agg')
matplotlib.use('pgf')
import matplotlib.pyplot as plt
margin = 0 #inch
scale = 3
plt.xlim(left=margin)
plt.ylim(bottom=margin)
plt.rcParams.update({
  'text.usetex': True,
  'font.size': 10 * scale,
  'axes.titlesize': 10 * scale,
  'axes.labelsize': 10 * scale,
  'xtick.labelsize': 10 * scale,
  'ytick.labelsize': 10 * scale
})


fig, ax = plt.subplots()

m, M = np.inf, -np.inf
for k, (_, v) in zip(K, data):
  unique, counts = np.unique(v, return_counts=True)
  counts = counts / np.sum(counts)
  x = k * 100
  width_scale = 100

  palette = ['#fee08b7f', '#9e01427f', '#66c2a57f', '#3288bd7f', '#f46d437f', '#fdae617f', '#d53e4f7f', '#abdda47f', '#e6f5987f']

  # np.random.shuffle(palette)  

  ax.barh(
    unique,
    width=width_scale * counts,
    left=x - width_scale * counts,

    # color=[f'#BBD6E8{int(255 * c):02x}' for c in counts],
    # color=[f'#000000{int(255 * c):02x}' for c in counts],
    color=palette[np.min(unique) - 1:np.max(unique)],
    height=0.9
  )

  ax.plot([x - width_scale, x], [np.max(unique)] * 2, linestyle='--', linewidth=0.5, color='#2777B4')
  ax.plot([x - width_scale, x], [np.mean(unique)] * 2, linestyle='--', linewidth=0.5, color='#2777B4')
  ax.plot([x - width_scale, x], [np.min(unique)] * 2, linestyle='--', linewidth=0.5, color='#2777B4')

  ax.annotate(rf'${np.max(unique)}$', xy=(x - width_scale, np.max(unique)), color='black')
  ax.annotate(rf'${np.mean(unique)}$', xy=(x - width_scale - 25, np.mean(unique) - 0.25), color='black')
  ax.annotate(rf'${np.min(unique)}$', xy=(x - width_scale, np.min(unique) - 0.45), color='black')

  m = min(m, np.min(unique))
  M = max(M, np.max(unique))


ax.set_xticks(np.array(K) * 100, [rf'${n}$' for n in K], rotation=30)
ax.set_yticks(range(m-1, M+2), [rf'${n}$' for n in range(m-1, M+2)])
ax.set_xlabel(r'$K$', labelpad=0)
ax.set_ylabel(r'$Demonstrations$')

ax.annotate(
  rf'${N_RUNS_PER_K}\ trials\ per\ K\ for \ Scooping$' + '\n',
  # rf'$\epsilon={epsilon} \quad \delta={delta} \quad \beta={beta}$',
  xy=(0.02, 0.9),
  xycoords='axes fraction',
  color='b'
)

plt.savefig(f'images/violin_{N_RUNS_PER_K}_samples.png', bbox_inches='tight')
plt.savefig(f'images/violin_{N_RUNS_PER_K}_samples.pgf', bbox_inches='tight')