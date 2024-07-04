export const robotConfigurationFiles = {
  'baxter_joint_axes.csv': 'joint_axes',
  'baxter_joint_q.csv': 'joint_q',
  'baxter_joint_limits.csv': 'joint_limits',
  'baxter_gst0.csv': 'gst0'
}

export const demonstrationConfigurationFile = 'config.json'

export const demonstrationConfigurationFormat = {
  n_objects: '[number]',
  dimensions: [
    {name: 'x', min: '[number]', max: '[number]', n_segments: '[number]'},
    {name: 'y', min: '[number]', max: '[number]', n_segments: '[number]'},
    {name: 'θ', min: '[number]', max: '[number]', n_segments: '[number]'}
  ],
  initial_joint_config: ['[number]', '[number]', '[number]', '[number]', '[number]', '[number]', '[number]']
}

export const demonstrationFiles = {
  object_pose: 'object_poses.csv',
  joint_angle: 'joint_angles.csv',
  roi: 'region_of_interest.txt'
}


export const backEndHost = 'http://127.0.0.1:8080'
