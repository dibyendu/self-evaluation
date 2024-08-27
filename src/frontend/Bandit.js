import nj from 'https://esm.run/numjs'

import { robotConfigurationFiles } from './Config'


export default function naivePAC({ dimensions, initial_joint_config, n_objects, demonstrations, arms, epsilon=0.2, delta=0.05, onFinishCallback=() => {} }) {
  const n_arms = Object.keys(arms).length
  const n_samples_per_arm = Math.floor(Math.log(2 * n_arms / delta) / (2 * epsilon ** 2))

  const metadata = {}

  const robotConfiguration = Object.assign(
    ...Object.values(robotConfigurationFiles)
    .map(key => ({ [key]: localStorage.getItem(key) }))
  )

  for (const [arm_id, arm] of Object.entries(arms)) {
    console.log(`Evaluating ${n_samples_per_arm} task instances for Bandit-Arm #${arm_id} ...`)

    const worker = new Worker(new URL('bandit.worker.js', import.meta.url), { type: 'module' })
    worker.postMessage({ arm_id, arms, demonstrations, n_objects, n_dimensions: dimensions.length, initial_joint_config, n_samples_per_arm, robot_config: robotConfiguration })
    worker.onmessage = ({ data: { result: { arm_id, metadata: meta }}}) => {
      worker.terminate()
      metadata[arm_id] = meta
      if (Object.keys(metadata).length === n_arms) {

        const arm_probabilities = Object.entries(metadata).map(([arm_id, { failed_indices, task_instances }]) => [arm_id, failed_indices.length / task_instances.length]),
              worst_arm_failure_probability = nj.max(arm_probabilities.map(([, probability]) => probability)),
              worst_arm_indices = arm_probabilities
                                  .filter(([arm_id, probability]) => probability === worst_arm_failure_probability)
                                  .map(([arm_id]) => arm_id),
              worst_arm_index = worst_arm_indices[Math.floor(Math.random() * worst_arm_indices.length)]

        const min_score = nj.min(metadata[worst_arm_index].failure_scores) ?? 0

        const failed_task_instances = metadata[worst_arm_index].failed_indices
                                      .map((failed_index, index) => metadata[worst_arm_index].failure_scores[index] === min_score ? failed_index : null)
                                      .filter(index => index !== null)

        for (const id in metadata) {
          let task_instances = nj.array(metadata[id].task_instances)
          task_instances = task_instances.slice(null, null, [3], -1)
          task_instances = task_instances.reshape(task_instances.shape.slice(0, -1))
          metadata[id].task_instances = task_instances.tolist()
        }

        let next_demonstration = null
        if (failed_task_instances.length) {
          const failed_task_index = failed_task_instances[Math.floor(Math.random() * failed_task_instances.length)]
          next_demonstration = metadata[worst_arm_index].task_instances[failed_task_index]
        }

        onFinishCallback({
          metadata,
          worst_arm_index,
          worst_arm_failure_probability,
          next_demonstration
        })
      }
    }
  }
}