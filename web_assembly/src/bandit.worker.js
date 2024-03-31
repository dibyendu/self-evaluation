import nj from 'https://esm.run/numjs'
import WasmModule from './server.mjs'


function to_SE3(pose) {
  const [x, y, theta] = pose
  return [[Math.cos(theta), -Math.sin(theta), 0,                    x],
          [Math.sin(theta),  Math.cos(theta), 0,                    y],
          [              0,                0, 1, -0.06447185171756116],
          [              0,                0, 0,                    1]]
}


function zip(a, b){
  return a.map((v, i) => [v, b[i]])
}


function unzip(a){
  return a.reduce((acc, [l, h]) => {
    acc[0].push(l)
    acc[1].push(h)
    return acc
  }, [[], []])
}


function uniformRandom(low, high, shape) {
  return nj.add(
    nj.multiply(
      nj.random(shape),
      Array(shape[0]).fill(zip(low, high).map(([l, h]) => h - l))
    ),
    Array(shape[0]).fill(low)
  )
}


async function motionPlanner(demontrations, init_joint_config, task_instances, robot_config) {

  const Module = await WasmModule()

  const initJointVector = new Module.VectorDouble(),
        taskInstancesVector = new Module.TaskInstanceVector(),
        demonstrationVector = new Module.DemonstrationVector()
  for (const joint_angle of init_joint_config)
    initJointVector.push_back(joint_angle)
  for (const { id, joint_angles, object_poses, score, region_of_interest } of demontrations)
    demonstrationVector.push_back({ id, joint_angles, object_poses, score, region_of_interest  })
  let poseVector = new Module.VectorDouble()
  for (const task_instance of task_instances.tolist()) {
    const taskInstanceVector = new Module.VectorVectorDouble()
    for (const pose of task_instance) {
      const poseVector = new Module.VectorDouble()
      for (const elem of nj.flatten(nj.array(pose)).tolist())
        poseVector.push_back(elem)
      taskInstanceVector.push_back(poseVector)
    }
    taskInstancesVector.push_back(taskInstanceVector)
  }

  const plansVector = Module.motionPlan(robot_config, initJointVector, demonstrationVector, taskInstancesVector)

  return new Array(plansVector.size()).fill().map((_, i) => {
    const currentPlans = plansVector.get(i)
    return new Array(currentPlans.size()).fill().map((_, j) => {
      const plan = currentPlans.get(j),
            joint_angles = plan.joint_angles,
            screw_segments = plan.screw_segments
      return {
        demonstration_id: plan.demo_id,
        is_successful: plan.is_successful,
        n_screw_segments: plan.n_screw_segments,
        screw_segments: new Array(screw_segments.size()).fill().map((_, k) => screw_segments.get(k)),
        plan: new Array(joint_angles.size()).fill().map((_, k) => {
          const jointVector = joint_angles.get(k)
          return new Array(jointVector.size()).fill().map((_, l) => jointVector.get(l))
        }),
        failed_screw_segment: plan.failed_screw_segment,
        failed_joint_angle: plan.failed_joint_id
      }
    })
  })
}


async function evaluateArm(arm_id, arms, demontrations, n_objects, n_dimensions, initial_joint_config, n_task_instances, robot_config) {
  const arm = arms[arm_id]
  let task_instances

  const [low, high] = unzip(arm['segment'])
  task_instances = uniformRandom(low, high, [n_task_instances, n_dimensions])
  task_instances = nj.reshape(task_instances, [n_task_instances, n_objects, n_dimensions])

  task_instances = nj.array(task_instances.tolist().map(([pose]) => [to_SE3(pose)]))

  demontrations = demontrations.toSorted(({ score: s1 }, { score: s2 }) => s2 - s1)

  const motion_plans = await motionPlanner(demontrations, initial_joint_config, task_instances, robot_config)

  const [failed_indices, scores] = unzip(
    motion_plans.map((plans, index) => {
      if (!plans.at(-1).is_successful) {
        const score = nj.mean(plans.map(plan => plan.failed_screw_segment / plan.n_screw_segments))
        return [index, score]
      }
      return null
    })
    .filter(elem => elem !== null)
  )

  return {
    task_instances: task_instances.tolist(),
    failed_indices,
    failure_scores: scores,
    plans: motion_plans
  }
}


onmessage = ({ data: { arm_id, arms, demonstrations, n_objects, n_dimensions, initial_joint_config, n_samples_per_arm, robot_config }}) => {
  evaluateArm(arm_id, arms, demonstrations, n_objects, n_dimensions, initial_joint_config, n_samples_per_arm, robot_config)
  .then(metadata => postMessage({ result: { arm_id, metadata }}))
}