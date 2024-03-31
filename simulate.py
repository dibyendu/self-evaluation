from client import *

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


for k in K:
  key = str(k['x'] * k['y'])
  if len(data.get(key, [])) >= N_RUNS_PER_K:
    continue

  for d in DIMENSIONS:
    if d['name'] == 'x':
      d['n_segments'] = k['x']
    if d['name'] == 'y':
      d['n_segments'] = k['y']
  
  intervals = ([(interval[i], interval[i + 1]) for i in range(dimension['n_segments'])] for interval, dimension in zip((np.linspace(d['min'], d['max'], d['n_segments'] + 1) for d in DIMENSIONS), DIMENSIONS))
  
  segments = itertools.product(*intervals, repeat = N_OBJECTS)
  
  BANDIT_ARMS = {(i+1): {
    'segment': segment,
    'samples': {},
    'demos': []
  } for i, segment in enumerate(segments)}
  
  for demo in DEMONSTRATIONS:
    f = open(demo['object_poses_file'], newline='')
    x, y = [float(row[-1]) for row in csv.reader(f, delimiter=',')][:2]
    f.close()
    for arm in BANDIT_ARMS.values():
      (x_min, x_max), (y_min, y_max), _ = arm['segment']
      if (x_min <= x <= x_max) and (y_min <= y <= y_max):
        arm['demos'].append(demo)
        break

  for _ in range(N_RUNS_PER_K - len(data.get(key, []))):
    demos = self_evaluation(copy.deepcopy(BANDIT_ARMS), epsilon = epsilon, delta = delta, beta = beta, reuse_tasks_instances = True)
    if len(demos):
      data.setdefault(key, []).append(len(demos))
      f = open(LOG_FILE, 'w')
      f.write(json.dumps(data))
      f.close()



data = sorted([(k['x'] * k['y'], data[str(k['x'] * k['y'])]) for k in K], key = lambda x: x[0])
K = [k for k, _ in data]
data = [v for _, v in data]





import matplotlib
matplotlib.use('agg')
matplotlib.use('pgf')
import matplotlib.pyplot as plt
margin = 0 #inch
scale = 1.6
plt.xlim(left=margin)
plt.ylim(bottom=margin)
plt.rcParams.update({
  'text.usetex': True,
  'font.size': 8 * scale,
  'axes.titlesize': 8 * scale,
  'axes.labelsize': 8 * scale,
  'xtick.labelsize': 8 * scale,
  'ytick.labelsize': 8 * scale
})

fig, ax = plt.subplots()

v1 = ax.violinplot(data, positions=K, widths = [scale] * len(K), showmeans=True)

for body in v1['bodies']:
  m = np.mean(body.get_paths()[0].vertices[:, 0])
  body.get_paths()[0].vertices[:, 0] = np.clip(body.get_paths()[0].vertices[:, 0], -np.inf, m)

v1['cbars'].set_linewidths(0)
for lines in (v1['cmins'], v1['cmeans'], v1['cmaxes']):
  lines.set_linewidths(0.5)
  lines.set_linestyle('--')
  segments = lines.get_segments()
  for k, seg in zip(K, segments):
    seg[0,0] -= 0.5
    seg[-1,0] = k
  lines.set_segments(segments)


for x, samples in zip(K, data):
  s_min, s_mean, s_max = np.min(samples), np.mean(samples), np.max(samples)
  plt.annotate(rf'${int(s_min)}$', (x, s_min), ha='right', va='top')
  plt.annotate(rf'${s_mean:.1f}$', (x, s_mean), ha='right', va='center')
  plt.annotate(rf'${int(s_max)}$', (x, s_max), ha='right', va='bottom')



m, M= zip(*[(min(d), max(d)) for d in data])
m, M = min(m) - 1, max(M) + 1
ax.set_xticks(K, [rf'${n}$' for n in K], rotation=30)
ax.set_yticks(range(m, M+1), [rf'${n}$' for n in range(m, M+1)])
ax.set_xlabel(r'$K$', labelpad=0)
ax.set_ylabel(r'$Demonstrations$')

ax.annotate(
  rf'${N_RUNS_PER_K}\ trials\ per\ K\ for \ Scooping$' + '\n' +
  rf'$\epsilon={epsilon} \quad \delta={delta} \quad \beta={beta}$',
  xy=(0.02, 0.9),
  xycoords='axes fraction',
  color='b'
)

plt.savefig(f'images/violin_{N_RUNS_PER_K}_samples.png', bbox_inches='tight')
plt.savefig(f'images/violin_{N_RUNS_PER_K}_samples.pgf', bbox_inches='tight')