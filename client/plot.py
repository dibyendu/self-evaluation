import csv
import matplotlib
import numpy as np

from client import config

matplotlib.use('agg')
# if config.USE_LATEX_IMAGE: matplotlib.use('pgf')

import matplotlib.pyplot as plt
from mpl_toolkits.axes_grid1.axes_divider import make_axes_locatable


def plot_heatmap(dimensions, n_objects, arms, demontrations, samples, output_directory):

  margin = 0  # inch
  scale = 4

  plt.xlim(left=margin)
  plt.ylim(bottom=margin)

  plt.rcParams.update({
    'text.usetex': config.USE_LATEX_IMAGE,
    'font.size': 10 * scale,  # Use 10pt font in plots, to match 10pt font in document
    # 'legend.fontsize': 8 * scale,
    'axes.titlesize': 10 * scale,
    'axes.labelsize': 25,  #10 * scale,
    'xtick.labelsize': 25,  #10 * scale,
    'ytick.labelsize': 25  #10 * scale
  })

  x = list(filter(lambda d: d['name'] == 'x', dimensions))[0]
  y = list(filter(lambda d: d['name'] == 'y', dimensions))[0]
  zz = np.empty((y['n_segments'], x['n_segments']))
  x, y = (
    np.linspace(x['min'], x['max'], x['n_segments'] + 1),
    np.linspace(y['min'], y['max'], y['n_segments'] + 1)
  )

  epsilon = 1e-10
  for j in range(len(y) - 1):
    for i in range(len(x) - 1):
      segment = ((x[i], x[i + 1]), (y[j], y[j + 1]), (0, 0))
      for arm_id, arm in arms.items():
        if all(abs(s1[0] - s2[0]) < epsilon for s1, s2 in zip(arm['segment'], segment)):
          zz[j, i] = len(samples[arm_id]['failure']) / (len(samples[arm_id]['success']) + len(samples[arm_id]['failure']))
          break

  xx, yy = np.meshgrid(x, y)

  fig, ax = plt.subplots()

  fig.set_facecolor((1, 1, 1, 0))
  ax.set_facecolor((1, 1, 1, 0))

  heatmap_cmap = plt.get_cmap('binary')

  ax.pcolormesh(xx, yy, zz, cmap=heatmap_cmap, vmin=0, vmax=1)

  ax.axis([xx.min(), xx.max(), yy.min(), yy.max()])

  ax.set_xticks(x, [rf'${n:.2f}$' for n in x], rotation=30)
  ax.set_yticks(y, [rf'${n:.2f}$' for n in y])

  ax.set_xlabel(r'$x$', labelpad=0)
  ax.set_ylabel(r'$y$', labelpad=0)

  cbar = plt.colorbar(
    mappable=matplotlib.cm.ScalarMappable(
      norm=matplotlib.colors.Normalize(vmin=0, vmax=np.abs(zz).max()),
      cmap=matplotlib.colors.LinearSegmentedColormap.from_list(
        f'trunc({heatmap_cmap.name}, 0, {np.abs(zz).max():.2f})',
        heatmap_cmap(np.linspace(0, np.abs(zz).max(), heatmap_cmap.N))
      )
    ),
    ax=ax,
    extend='max' if np.abs(zz).max() < 1.0 else 'neither'
  )
  cbar.ax.tick_params()
  cbar.ax.set_facecolor((1, 1, 1, 0))
  cbar.ax.tick_params(labelsize=25)

  success, failure = np.empty((0, n_objects, len(dimensions))), np.empty((0, n_objects, len(dimensions)))
  for arm_id, arm in arms.items():
    success = np.vstack((success, samples[arm_id]['success']))
    failure = np.vstack((failure, samples[arm_id]['failure']))
  success, failure = success[:, :, :2], failure[:, :, :2]

  plt.scatter(success[:, :, 0], success[:, :, 1], s=0.4 * scale, c='green')
  plt.scatter(failure[:, :, 0], failure[:, :, 1], s=0.4 * scale, c='red')

  demo_x, demo_y = [], []
  for i, demo in enumerate(demontrations):
    f = open(demo['object_poses_file'], newline='')
    x, y = [float(row[-1]) for row in csv.reader(f, delimiter=',')][:2]
    f.close()
    demo_x.append(x)
    demo_y.append(y)
    plt.annotate(rf'${str(i+1)}$', (x, y), xytext=(x, y), textcoords='offset points', ha='center', va='center', color='white', fontsize=10 * scale)

    # if demo['score'] != -1:
    #   plt.annotate(rf'${demo["score"]:.4f}$', (x, y), xytext=(x, y-10), textcoords='offset points', ha='center', va='top', color='black', fontsize=9 * scale)

  plt.scatter(demo_x, demo_y, s=100 * scale, c='black')

  plt.savefig(f'{output_directory}/heatmap_{len(arms)}_{len(demontrations)}.png', bbox_inches='tight', pad_inches=margin)
  if config.USE_LATEX_IMAGE:
    plt.savefig(f'{output_directory}/heatmap_{len(arms)}_{len(demontrations)}.pgf', bbox_inches='tight', pad_inches=margin)
  plt.close()








def plot_heatmap_for_video(dimensions, n_objects, arms, demontrations, samples, next_demo, draw_next_demo, output_directory, transparent = False):

  margin = 0
  scale = 1.2

  plt.xlim(left=margin)
  plt.ylim(bottom=margin)

  plt.rcParams.update({
    'text.usetex': config.USE_LATEX_IMAGE,
    'font.size': 10 * scale,
    'axes.titlesize': 10 * scale,
    'axes.labelsize': 25,  #10 * scale,
    'xtick.labelsize': 25,  #10 * scale,
    'ytick.labelsize': 25  #10 * scale
  })

  x = list(filter(lambda d: d['name'] == 'x', dimensions))[0]
  y = list(filter(lambda d: d['name'] == 'y', dimensions))[0]
  zz = np.empty((y['n_segments'], x['n_segments']))
  x, y = (
    np.linspace(x['min'], x['max'], x['n_segments'] + 1),
    np.linspace(y['min'], y['max'], y['n_segments'] + 1)
  )


  epsilon = 1e-10
  for j in range(len(y) - 1):
    for i in range(len(x) - 1):
      segment = ((x[i], x[i + 1]), (y[j], y[j + 1]), (0, 0))
      for arm_id, arm in arms.items():
        if all(abs(s1[0] - s2[0]) < epsilon for s1, s2 in zip(arm['segment'], segment)):
          zz[j, i] = len(samples[arm_id]['failure']) / (len(samples[arm_id]['success']) + len(samples[arm_id]['failure']))
          break

  xx, yy = np.meshgrid(x, y)


  fig_scale = 10
  figsize=((x[-1] - x[0]) * fig_scale, (y[-1] - y[0]) * fig_scale)
  fig, ax = plt.subplots()


  fig.set_facecolor((1, 1, 1, 0))
  ax.set_facecolor((1, 1, 1, 0))

  transparent_cmap = matplotlib.colors.LinearSegmentedColormap.from_list(
    'transparent_cmap', [(c, c, c, 0.4) for c in np.linspace(1, 0, 200)],
    N=200)

  heatmap_cmap = transparent_cmap if transparent else plt.get_cmap('binary')

  ax.pcolormesh(xx, yy, zz, cmap=heatmap_cmap, vmin=0, vmax=1)

  ax.hlines(y[1:-1], x[0], x[-1], colors='#ffffff44', linestyles='dotted')
  ax.vlines(x[1:-1], y[0], y[-1], colors='#ffffff44', linestyles='dotted')

  ax.axis([xx.min(), xx.max(), yy.min(), yy.max()])

  ax.set_xticks(x, [rf'${n:.2f}$' for n in x], rotation=-90)
  ax.set_yticks(y, [rf'${n:.2f}$' for n in y[::-1]], rotation=-150)


  divider = make_axes_locatable(ax)
  cax = divider.append_axes('bottom', size='2%', pad='10%')

  cbar = plt.colorbar(
    mappable=matplotlib.cm.ScalarMappable(
      norm=matplotlib.colors.Normalize(vmin=0, vmax=np.abs(zz).max()),
      cmap=matplotlib.colors.LinearSegmentedColormap.from_list(
        f'trunc({heatmap_cmap.name}, 0, {np.abs(zz).max():.2f})',
        heatmap_cmap(np.linspace(0, np.abs(zz).max(), heatmap_cmap.N))
      )
    ),
    cax=cax,
    extend='max' if np.abs(zz).max() < 1.0 else 'neither',
    orientation='horizontal'
  )
  cbar.set_label(r'$Failure\ Probability$', labelpad=10)
  cbar.ax.tick_params(rotation=-90)
  cbar.ax.set_facecolor((1, 1, 1, 0))


  success, failure = np.empty((0, n_objects, len(dimensions))), np.empty((0, n_objects, len(dimensions)))
  for arm_id, arm in arms.items():
    success = np.vstack((success, samples[arm_id]['success']))
    failure = np.vstack((failure, samples[arm_id]['failure']))
  success, failure = success[:, :, :2], failure[:, :, :2]

  ax.scatter(success[:, :, 0], success[:, :, 1], s=8 * scale, c='green')
  ax.scatter(failure[:, :, 0], failure[:, :, 1], s=8 * scale, c='red')

  demo_x, demo_y = [], []
  for i, demo in enumerate(demontrations):
    f = open(demo['object_poses_file'], newline='')
    x, y = [float(row[-1]) for row in csv.reader(f, delimiter=',')][:2]
    f.close()
    demo_x.append(x)
    demo_y.append(y)
    ax.annotate(rf'${str(i+1)}$', (x, y), xytext=(x, y), textcoords='offset points', ha='center', va='center', color='white', fontsize=16 * scale, rotation=-90)

  ax.scatter(demo_x, demo_y, s=250 * scale, c='black')

  
  l = ax.figure.subplotpars.left
  r = ax.figure.subplotpars.right
  t = ax.figure.subplotpars.top
  b = ax.figure.subplotpars.bottom + cax.figure.subplotpars.bottom
  fig_width = float(figsize[0]) / (r - l)
  fig_height = float(figsize[1]) / (t - b)
  ax.figure.set_size_inches(fig_width, fig_height)


  plt.savefig(f'{output_directory}/heatmap_{len(arms)}_{len(demontrations)}.png', bbox_inches='tight', pad_inches=margin, transparent=transparent)

  if draw_next_demo:
    next_demo_x, next_demo_y = next_demo[0], next_demo[1]
    plt.annotate(' ', (next_demo_x, next_demo_y), xytext=(next_demo_x, next_demo_y), textcoords='offset points', ha='center', va='center', color='white', fontsize=10 * scale, rotation=-90)
    plt.scatter([next_demo_x], [next_demo_y], s=80 * scale, c='black')
    plt.savefig(f'{output_directory}/next_demo_{len(arms)}_{len(demontrations)}.png', bbox_inches='tight', pad_inches=margin, transparent=transparent)

  plt.close()








def plot_object_for_video(dimensions, pose, output_directory, file_name, transparent = False):

  margin = 0
  scale = 1.2

  plt.xlim(left=margin)
  plt.ylim(bottom=margin)

  plt.rcParams.update({
    'text.usetex': config.USE_LATEX_IMAGE,
    'font.size': 10 * scale,
    'axes.titlesize': 10 * scale,
    'axes.labelsize': 25,  #10 * scale,
    'xtick.labelsize': 25,  #10 * scale,
    'ytick.labelsize': 25  #10 * scale
  })

  x = list(filter(lambda d: d['name'] == 'x', dimensions))[0]
  y = list(filter(lambda d: d['name'] == 'y', dimensions))[0]
  x, y = (
    np.linspace(x['min'], x['max'], x['n_segments'] + 1),
    np.linspace(y['min'], y['max'], y['n_segments'] + 1)
  )

  fig_scale = 10
  figsize=((x[-1] - x[0]) * fig_scale, (y[-1] - y[0]) * fig_scale)
  fig, ax = plt.subplots()

  fig.set_facecolor((1, 1, 1, 0))
  ax.set_facecolor((1, 1, 1, 0))

  ax.hlines(y[1:-1], x[0], x[-1], colors='#ffffff44', linestyles='dotted')
  ax.vlines(x[1:-1], y[0], y[-1], colors='#ffffff44', linestyles='dotted')

  ax.axis([x[0], x[-1], y[0], y[-1]])

  ax.set_xticks(x, [rf'${n:.2f}$' for n in x], rotation=-90)
  ax.set_yticks(y, [rf'${n:.2f}$' for n in y[::-1]], rotation=-150)

  pose_x, pose_y = pose[0], pose[1]

  ax.annotate(' ', (pose_x, pose_y), xytext=(pose_x, pose_y), textcoords='offset points', ha='center', va='center', color='white', fontsize=16 * scale, rotation=-90)

  ax.scatter([pose_x], [pose_y], s=250 * scale, c='black')

  l = ax.figure.subplotpars.left
  r = ax.figure.subplotpars.right
  t = ax.figure.subplotpars.top
  b = ax.figure.subplotpars.bottom
  fig_width = float(figsize[0]) / (r - l)
  fig_height = float(figsize[1]) / (t - b)
  ax.figure.set_size_inches(fig_width, fig_height)

  plt.savefig(f'{output_directory}/{file_name}.png', bbox_inches='tight', pad_inches=margin, transparent=transparent)
  plt.close()