import { useCallback, useEffect, useState, useRef } from 'react'
import nj from 'https://esm.run/numjs'

import './css/style.css'
import {
  ε, δ, β,
  backEndHost,
  demonstrationFiles,
  robotConfigurationFiles,
  demonstrationConfigurationFile,
  demonstrationConfigurationFormat
} from './Config'
import naivePAC from './Bandit'
import robotImageUrl from './img/robot_top_view.svg'




const robotConfiguration = import.meta.glob('../../baxter_config/*.csv', { query: '?raw' })
import demonstrationConfiguration from '../../demonstration_config.json'




const defaultWorkSpaceTheme = {
  border: { width: 1, color: '#000000' },
  scale: 1100,
  heatmapColor: { red: 100, green: 100, blue: 100 },
  demonstration: { size: 12, color: '#000000', textColor: '#ffffff' },
  taskInstance: { size: 4, success: '#00ff00', failure: '#ff0000', hint: '#ffff00' }
}

const robotImageWidth = 400


function calculate_plan_score(joint_limits, joint_angles) {
  joint_angles = joint_angles.slice(null, [1, 8])
  const [row] = joint_angles.shape

  const lower_limit = nj.array(Array(row).fill(joint_limits.pick(null, 0).tolist())),
        upper_limit = nj.array(Array(row).fill(joint_limits.pick(null, 1).tolist()))

  const lower_bound_proximity = joint_angles.subtract(lower_limit),
        upper_bound_proximity = upper_limit.subtract(joint_angles)

  return Math.min(nj.min(lower_bound_proximity), nj.min(upper_bound_proximity))
}


function linspace(start, end, segments) {
  const step = (end - start) / (segments - 1)
  let index = -1,
      length = Math.max(Math.ceil((end - start) / (step || 1)), 0),
      result = Array(length)
  while (length--) {
    result[++index] = start
    start += step
  }
  return result.length === segments ? result : result.concat(end)
}

function cartesianProduct(iterables, repeat) {
  return Array(repeat).fill(iterables.slice()).flat().reduce((acc, value) => {
    let tmp = []
    acc.forEach(a0 => {
      value.forEach(a1 => {
        tmp.push(a0.concat([a1]))
      })
    })
    return tmp
  }, [[]])
}


function Theme({ open, setOpen, applyTheme }) {

  const [theme, setTheme] = useState(defaultWorkSpaceTheme)

  return (
    <dialog open={open} style={{ backgroundColor: '#fff', zIndex: 2, width: '25%' }}>
      <p>Select your colours</p>
      <div>
        <label htmlFor='border'>Border</label>
        <input type='color' id='border' value={theme.border.color} onChange={({ target: { value }}) => setTheme(prev => ({ ...prev, border: { ...prev.border, color: value } }))} />
      </div>
      <div>
        <label htmlFor='task-failure'>Failed Task Instances</label>
        <input type='color' id='task-failure' value={theme.taskInstance.failure} onChange={({ target: { value }}) => setTheme(prev => ({ ...prev, taskInstance: { ...prev.taskInstance, failure: value } }))} />
      </div>
      <div>
        <label htmlFor='task-success'>Successfull Task Instances</label>
        <input type='color' id='task-success' value={theme.taskInstance.success} onChange={({ target: { value }}) => setTheme(prev => ({ ...prev, taskInstance: { ...prev.taskInstance, success: value } }))} />
      </div>
      <div>
        <label htmlFor='task-hint'>Next Demonstration Hint</label>
        <input type='color' id='task-hint' value={theme.taskInstance.hint} onChange={({ target: { value }}) => setTheme(prev => ({ ...prev, taskInstance: { ...prev.taskInstance, hint: value } }))} />
      </div>
      <button onClick={() => {
        setOpen(false)
        applyTheme(theme)
      }}>Apply</button>
    </dialog>
  )
}


function PlanInfo({ style, position, plans }) {
  
  const [modalOpen, setModalOpen] = useState(false)

  return (
    <>
      <span style={style} onClick={() => setModalOpen(true)} />
      <dialog open={modalOpen} style={{ backgroundColor: '#fff', zIndex: 2 }}>
        <h3 style={{ display: 'flex', justifyContent: 'space-between', margin: 0 }}>Plans
          <span className='material-symbols-rounded' style={{ cursor: 'pointer' }} onClick={() => setModalOpen(false)}>
            close
          </span>
        </h3>
        <div>
          <table>
            <thead>
              <tr style={{ textAlign: 'center' }}>
                <th style={{ fontSize: 14 }}>Demo #</th>
                <th style={{ fontSize: 14 }}>Joint #</th>
                <th style={{ fontSize: 14 }}>Screw segment #</th>
                <th style={{ fontSize: 14 }}>Visualise</th>
              </tr>
            </thead>
            <tbody>
              {plans.map(({ demonstration_id, is_successful, failed_joint_angle, failed_screw_segment, plan, screw_segments }, index) =>
                <tr key={index} style={{ textAlign: 'center', backgroundColor: is_successful ? '#c5ffae': '#ff9797' }}>
                  <td>{demonstration_id}</td>
                  <td>{is_successful || failed_joint_angle === -1 ? '' : failed_joint_angle}</td>
                  <td>{is_successful || failed_joint_angle === -1 ? '' : failed_screw_segment}</td>
                  <td>
                    <span className='material-symbols-rounded' style={{ cursor: 'pointer', verticalAlign: 'text-bottom' }} onClick={() => {
                      const state = { plan, position, screw_segments }
                      localStorage.setItem('navigation_state', JSON.stringify(state))
                      window.open('/visualise', '_blank', 'noreferrer')
                    }}>
                      visibility
                    </span>
                  </td>
                </tr>
              )}
            </tbody>
          </table>
        </div>
      </dialog>
    </>
  )
}


function useAsyncAction () {
  const [isPending, setPending] = useState(false)

  const actionInitiator = async fn => {
    setPending(true)
    await fn()
    setPending(false)
  }

  return [isPending, actionInitiator]
}


function Switch({ disabled, handleToggle }) {
  const [active, setActive] = useState(false)

  return (
    <div className={`switch-container ${disabled ? 'disabled' : ''}`}>
      <label className='switch'>
        <input type='checkbox' checked={active} onChange={event => {
          event.preventDefault()
          handleToggle(active, () => setActive(!active))
        }}/>
        <span className={`slider ${disabled ? 'disabled' : ''}`}>
          <span className='title'>{active ? 'Stop Taking Demonstration' : 'Start Taking Demonstration'}</span>
          <span className='ball'>
            <span className='icon'>
              <svg width={25} height={25} viewBox='0 0 24 24' fill='none' aria-hidden='true' xmlns='http://www.w3.org/2000/svg'>
                <path d={active ? 'M 8 18 V 6 l 2 0 l 0 12 Z M 14 18 V 6 l 2 0 l 0 12 Z' : 'M8 18V6l8 6-8 6Z'} strokeWidth={2} stroke='currentColor' strokeLinecap='round' strokeLinejoin='round'/>
              </svg>
            </span>
          </span>
        </span>
      </label>
    </div>
  )
}


export default function App() {

  const firstCellRef = useRef(null)

  const [demoConfigAvailable, setDemoConfigAvailable] = useState(false)
  const [robotConfigAvailable, setRobotConfigAvailable] = useState(false)
  const [demonstrationAvailable, setDemonstrationAvailable] = useState(false)

  const [workSpaceConfig, setWorkSpaceConfig] = useState(null)

  const [robotImageDimensions, setRobotImageDimensions] = useState({ width: 0, height: 0 })

  const [processing, setProcessing] = useState(false)
  const [waitTime, setWaitTime] = useState(10)
  const [isPending, startAction] = useAsyncAction()

  const [probabilityHeatMap, setProbabilityHeatMap] = useState({})
  const [taskInstancesToRender, setTaskInstancesToRender] = useState([])
  const [demonstrationsToRender, setDemonstrationsToRender] = useState([])
  const [nextDemonstrationToRender, setNextDemonstrationToRender] = useState([])

  const [themeOpen, setThemeOpen] = useState(false)
  const [workSpaceTheme, setWorkSpaceTheme] = useState(defaultWorkSpaceTheme)

  const [demonstrationProcessPID, setDemonstrationProcessPID] = useState(null)

  const [userClickPose, setUserClickPose] = useState(null)
  const [freezeUserClickPose, setFreezeUserClickPose] = useState(false)
  const [showObjectPoseHint, setShowObjectPoseHint] = useState(true)


  const loadConfiguration = useCallback(fileMap => {
    Promise.all(fileMap.map(async ([path, module]) => {
      const data = (await module()).default
      return [path.split('/').at(-1), data]
    }))
    .then(fileData => {
      fileData.forEach(([file, data]) => localStorage.setItem(robotConfigurationFiles[file], data))
      setRobotConfigAvailable(true)
      console.log('Configuration files are loaded!')
    })
  }, [])


  useEffect(() => {
    setRobotConfigAvailable(
      Object.values(robotConfigurationFiles)
      .map(key => localStorage.getItem(key))
      .every(value => value !== null)
    )
    const config = sessionStorage.getItem(demonstrationConfigurationFile)
    if (config !== null) {
      const conf = JSON.parse(config),
            { dimensions } = conf
      setWorkSpaceConfig({
        ...conf,
        ...Object.assign(
          ...dimensions
          .filter(dimension => ['x', 'y'].some(name => name === dimension.name))
          .map(d => ({ [d.name]: { min: d.min, max: d.max, n_segments: d.n_segments }}))
        )
      })
    }
    setDemoConfigAvailable(config !== null)
    setDemonstrationAvailable(sessionStorage.getItem('demonstrations') !== null)
  }, [])

  return (
    <div style={{ display: 'flex', flexDirection: 'column', justifyContent: 'center', alignItems: 'center', height: '100%', position: 'fixed', width: '100%' }}>
      <Theme open={themeOpen} setOpen={setThemeOpen} applyTheme={theme => setWorkSpaceTheme(prev => ({ ...prev, ...theme }))}/>
      {processing && <div style={{ width: '100%', height: '100%', backgroundColor: '#00000088', position: 'absolute', top: 0, left: 0, zIndex: 1, display: 'flex', justifyContent: 'center', alignItems: 'center' }}><div className='spinner' /></div>}
      {
        !robotConfigAvailable || !demoConfigAvailable
        ?
        <button className={`button ${isPending ? 'disabled' : ''}`} onClick={() => {
          if (isPending) return
          startAction(() =>
            fetch(`${backEndHost}/init`)
            .then(response => {
              if (response.ok) {
                loadConfiguration(Object.entries(robotConfiguration))
                sessionStorage.setItem(demonstrationConfigurationFile, JSON.stringify(demonstrationConfiguration))
                const { dimensions } = demonstrationConfiguration
                setWorkSpaceConfig({
                  ...demonstrationConfiguration,
                  ...Object.assign(
                    ...dimensions
                    .filter(dimension => ['x', 'y'].some(name => name === dimension.name))
                    .map(d => ({ [d.name]: { min: d.min, max: d.max, n_segments: d.n_segments }}))
                  )
                })
                setDemoConfigAvailable(true)
              }
            }).catch(error => alert(error))
          )
        }}>
          Initialize the Robot
          <span className='material-symbols-rounded' style={{ fontSize: 60, marginLeft: 10 }}>precision_manufacturing</span>
        </button>
        :
        <>
          <div style={{ display: 'flex', flexDirection: 'row', justifyContent: 'center', alignItems: 'center', position: 'absolute', top: 20 }}>
            <Switch disabled={isPending} handleToggle={(isActive, callBack) => {
              if (isPending) return
              if (!isActive) {
                if (userClickPose === null) {
                  alert('Please click inside the workspace to set the object position before collecting the demonstration')
                  return
                }
                setFreezeUserClickPose(true)
                startAction(() => {
                  const { initial_joint_config } = JSON.parse(sessionStorage.getItem(demonstrationConfigurationFile))
                  return fetch(`${backEndHost}/start`, {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ joint_config: initial_joint_config, wait_time: waitTime })
                  })
                  .then(response => {
                    if (response.ok)
                      return response.json()
                  })
                  .then(({ pid }) => setDemonstrationProcessPID(pid))
                  .catch(error => alert(error))
                  .finally(() => callBack())
                })
              } else {
                startAction(() =>
                  fetch(`${backEndHost}/stop/${demonstrationProcessPID}`)
                  .then(response => {
                    if (response.ok)
                      return response.json()
                  })
                  .then(({ demonstration }) => {
                    setUserClickPose(null)
                    setShowObjectPoseHint(true)
                    setFreezeUserClickPose(false)

                    const objectPose = nj.array([
                      [ 1, 0, 0,  userClickPose.xx    ],
                      [ 0, 1, 0,  userClickPose.yy    ],
                      [ 0, 0, 1, -0.06447185171756116 ],
                      [ 0, 0, 0,  1                   ],
                    ]).tolist().map(row => row.join(',')).join('\r\n')

                    const savedDemonstrations = JSON.parse(sessionStorage.getItem('demonstrations') ?? '{}')

                    let joint_limits = localStorage.getItem('joint_limits')
                    joint_limits = nj.array(joint_limits.split('\n').map(row => row.split(',').map(cell => parseFloat(cell))))

                    const demoIndices = Object.keys(savedDemonstrations)
                    const index = demoIndices.length === 0 ? 1 : Math.max(...demoIndices) + 1
                    savedDemonstrations[index] = {}
                    
                    savedDemonstrations[index]['joint_angle'] = demonstration
                    savedDemonstrations[index]['object_pose'] = objectPose
                    // sphere of radius equals to 1.5 times the minimum distance between object and guiding pose
                    savedDemonstrations[index]['roi'] = 1.5
                    savedDemonstrations[index].score = calculate_plan_score(
                      joint_limits,
                      nj.array(demonstration.split('\n').slice(1).map(row => row.split(',').map(cell => parseFloat(cell))))
                    )

                    sessionStorage.setItem('demonstrations', JSON.stringify(savedDemonstrations))

                    const demonstrations = Object.entries(savedDemonstrations).map(([index, demo]) => ({
                      id: parseInt(index),
                      joint_angles: demo.joint_angle,
                      object_poses: demo.object_pose,
                      score: demo.score,
                      region_of_interest: demo.roi
                    }))

                    const { initial_joint_config, n_objects, dimensions } = workSpaceConfig
                    /*
                      dimensions = [
                        {'name': 'x',   'min': 0.6602,    'max': 1.2602,      'n_segments': 4},
                        {'name': 'y',   'min': -0.185,    'max': 0.7560,      'n_segments': 4},
                        {'name': 'θ',   'min':  0,        'max': 2 * Math.PI, 'n_segments': 1}
                      ]
                      
                      ∀ p ∈ (x, y, θ) i.e. dimensions ∃ p' ∈ SE(3) s.t.
                      
                      p: (x, y, θ) implies p': | cos(θ)  -sin(θ)  0  x |  for any arbitrary z ∈ R
                                               | sin(θ)   cos(θ)  0  y |
                                               |      0        0  1  z |
                                               |      0        0  0  1 |
                      
                      which rotates points in the xy plane counter-clockwise through an angle θ w.r.t. the positive x axis
                    */

                    const intervals = dimensions
                                      .map(({ min, max, n_segments }) =>
                                        linspace(min, max, n_segments + 1)
                                        .reduce((acc, value, index, array) => {
                                          if (index < array.length - 1)
                                            acc.push([value, array[index + 1]])
                                          else if (array.length === 1)
                                            acc.push([value, value])
                                          return acc
                                        }, [])
                                      )
                    const segments = cartesianProduct(intervals, n_objects)
                    const bandit_arms = Object.assign(...segments.map((segment, index) => ({ [index + 1]: { segment, demos: [] }})))
                    // arm ∈ [1, (d1 * d2 * d3) ** n_objects]

                    const renderPoints = []
                    demonstrations.forEach((demo, index) => {
                      const [x, y, z] = demo.object_poses.split('\n').map(line => line.trim()).filter(line => line.length).slice(0,3).map(row=> parseFloat(row.split(',').at(-1)))
                      const arm = Object.values(bandit_arms).find(arm => {
                        const [[x_min, x_max], [y_min, y_max], _] = arm.segment
                        return (x_min <= x) && (x <= x_max) && (y_min <= y) && (y <= y_max)
                      })
                      if (arm === undefined) {
                        alert(`Object pose used in demonstration #${index + 1} is outside the workspace`)
                        return
                      }
                      renderPoints.push([x, y, z, {
                        id: demo.id,
                        score: demo.score,
                        joint_angles: demo.joint_angles.split('\n').map(line => line.trim()).filter(line => line.length).slice(1).map(row=> row.split(',').slice(0,8).map(num => parseFloat(num)))
                      }])
                      arm.demos.push(demo)
                    })

                    setDemonstrationsToRender(renderPoints)
                    setProcessing(true)

                    /*
                      This algorithm gives ε-optimal arm with probability 1 − δ
                      Each arm must be sampled at least 1/2ε² * ln(2K/δ) times.
                      Both ε and δ should be small numbers
                    */
                    naivePAC({
                      dimensions,
                      initial_joint_config,
                      n_objects,
                      demonstrations,
                      arms: bandit_arms,
                      epsilon: ε,
                      delta: δ,
                      onFinishCallback: ({ metadata, worst_arm_index, worst_arm_failure_probability, next_demonstration }) => {
                        setDemonstrationAvailable(true)
                        if (next_demonstration !== null)
                          setNextDemonstrationToRender(next_demonstration)

                        if (worst_arm_failure_probability < 1 + ε - β)
                          alert('No more demonstration is required')

                        setTaskInstancesToRender(
                          Object.values(metadata).map(({ task_instances, failed_indices, failure_scores, plans }) => {
                            task_instances = task_instances.map(([[x, y, z]], index) => [x, y, z, { plans: plans[index], failed: false }])
                            failed_indices.forEach((index, i) => {
                              task_instances[index].at(-1).failed = true
                              task_instances[index].at(-1).score = failure_scores[i]
                            })
                            return task_instances
                          }).flat()
                        )

                        setProbabilityHeatMap(
                          Object.fromEntries(
                            Object.entries(metadata)
                            .map(([arm_id, { task_instances, failed_indices }]) => [arm_id, failed_indices.length / task_instances.length ])
                          )
                        )

                        setProcessing(false)
                      }
                    })
                  })
                  .catch(error => alert(error))
                  .finally(() => callBack())
                )
              }
            }}/>
            <span>Wait <input type='number' name='tentacles' min={4} max={10} value={waitTime} onChange={e => setWaitTime(Number(e.target.value))} /> seconds</span>
          </div>
          <>
            {!demonstrationAvailable ? <h3>Robot's Workspace</h3> : <h3>Robot's Belief</h3>}
            <div style={{ display: 'flex', flexDirection: 'column', margin: '0 auto', justifyContent: 'center', alignItems: 'center', position: 'relative' }}>
              <img
                src={robotImageUrl}
                onLoad={({ target }) => setRobotImageDimensions({ width: target.width, height: target.height })}
                style={{
                  position: 'absolute', zIndex: -1, opacity: 0.2,
                  width: robotImageWidth,
                  bottom: - robotImageDimensions.height / 2,
                  right: - robotImageDimensions.width / 2 - workSpaceConfig.y.min * workSpaceTheme.scale
                }}
              />
              {
                Array(workSpaceConfig.x.n_segments).fill().map((_, row) => {
                  const cellWidth = (workSpaceConfig.y.max - workSpaceConfig.y.min) * workSpaceTheme.scale / workSpaceConfig.y.n_segments,
                        cellHeight = (workSpaceConfig.x.max - workSpaceConfig.x.min) * workSpaceTheme.scale / workSpaceConfig.x.n_segments
                  return (
                    <div key={row} style={{ width: cellWidth * workSpaceConfig.y.n_segments, height: cellHeight, display: 'flex', flexDirection: 'row' }}>
                      {
                        Array(workSpaceConfig.y.n_segments).fill().map((_, column) => {
                          const probability = probabilityHeatMap[workSpaceConfig.x.n_segments * workSpaceConfig.y.n_segments - row * workSpaceConfig.x.n_segments - column]
                          return  (
                            <div
                              ref={row === 0 && column === 0 ? firstCellRef : null}
                              key={column}
                              style={Object.assign(
                                {
                                  width: cellWidth,
                                  height: cellHeight,
                                  borderRight: `${workSpaceTheme.border.width}px solid ${workSpaceTheme.border.color}`,
                                  borderBottom: `${workSpaceTheme.border.width}px solid ${workSpaceTheme.border.color}`,
                                  backgroundColor: `rgba(${workSpaceTheme.heatmapColor.red}, ${workSpaceTheme.heatmapColor.green}, ${workSpaceTheme.heatmapColor.blue}, ${probability ?? 0})`
                                },
                                row === 0 ? { borderTop: `${workSpaceTheme.border.width}px solid ${workSpaceTheme.border.color}` } : {},
                                column === 0 ? { borderLeft: `${workSpaceTheme.border.width}px solid ${workSpaceTheme.border.color}` } : {}
                              )}
                              onClick={event => {
                                if (freezeUserClickPose) return
                                const offsets = firstCellRef.current.getBoundingClientRect(),
                                      x = event.pageX - offsets.left,
                                      y = event.pageY - offsets.top
                                if (x >=0 && y >= 0 && x <= workSpaceConfig.x.n_segments * cellWidth && y <= workSpaceConfig.y.n_segments * cellHeight) {
                                  const xx = workSpaceConfig.x.min + (workSpaceConfig.y.n_segments * cellHeight - y) * (workSpaceConfig.x.max - workSpaceConfig.x.min) / (workSpaceConfig.y.n_segments * cellHeight)
                                  const yy = workSpaceConfig.y.min + (workSpaceConfig.x.n_segments * cellWidth - x) * (workSpaceConfig.y.max - workSpaceConfig.y.min) / (workSpaceConfig.x.n_segments * cellWidth)
                                  setUserClickPose({ x, y, xx, yy })
                                }
                              }}
                            />                            
                          )
                        }) 
                      }
                    </div>
                  )
                })
              }
              {
                taskInstancesToRender.map(([x, y, z, { plans, failed, score }], index) => {
                  const isNextDemo = nextDemonstrationToRender.some(([xn, yn]) => Math.abs(xn - x) < 1e-8  && Math.abs(yn - y) < 1e-8)
                  return (
                    <PlanInfo
                      key={index}
                      style={Object.assign({
                        cursor: 'pointer',
                        position: 'absolute',
                        opacity: isNextDemo ? 1 : 0.8,
                        backgroundColor: isNextDemo ? workSpaceTheme.taskInstance.hint : (failed ? workSpaceTheme.taskInstance.failure : workSpaceTheme.taskInstance.success),
                        width: workSpaceTheme.taskInstance.size,
                        height: workSpaceTheme.taskInstance.size,
                        borderRadius: workSpaceTheme.taskInstance.size / 2,
                        top: (workSpaceConfig.x.max - x) * workSpaceTheme.scale - workSpaceTheme.taskInstance.size / 2,
                        left: (workSpaceConfig.y.max - y) * workSpaceTheme.scale - workSpaceTheme.taskInstance.size / 2
                      }, isNextDemo ? { zIndex: 1 } : {})}
                      position={[x, y, z]}
                      plans={plans}
                    />
                  )
                })
              }
              {
                demonstrationsToRender.map(([x, y, z, { id, score, joint_angles }], index) =>
                  <span
                    key={index}
                    title={`Score: ${score}`}
                    style={{
                      position: 'absolute',
                      cursor: 'pointer',
                      textAlign: 'center',
                      opacity: 0.8,
                      color: workSpaceTheme.demonstration.textColor,
                      fontSize: workSpaceTheme.demonstration.size,
                      lineHeight: `${workSpaceTheme.demonstration.size}px`,
                      width: workSpaceTheme.demonstration.size, height: workSpaceTheme.demonstration.size,
                      borderRadius: workSpaceTheme.demonstration.size / 2,
                      backgroundColor: workSpaceTheme.demonstration.color,
                      top: (workSpaceConfig.x.max - x) * workSpaceTheme.scale - workSpaceTheme.demonstration.size / 2,
                      left: (workSpaceConfig.y.max - y) * workSpaceTheme.scale - workSpaceTheme.demonstration.size / 2
                    }}
                    onClick={() => {
                      joint_angles = nj.array(joint_angles)
                      const state = { plan: joint_angles.slice(null, [1,8]).tolist(), timestamps: joint_angles.slice(null, [0,1]).tolist().flat(), position: [x, y, z] }                        
                      localStorage.setItem('navigation_state', JSON.stringify(state))
                      window.open('/visualise', '_blank', 'noreferrer')
                    }}
                  >
                    {id}
                  </span>
                )
              }
              { showObjectPoseHint && userClickPose !== null && (
                <span
                  style={{
                    position: 'absolute',
                    opacity: 0.4,
                    width: workSpaceTheme.demonstration.size * 2, height: workSpaceTheme.demonstration.size * 2,
                    borderRadius: workSpaceTheme.demonstration.size,
                    backgroundColor: 'black',
                    top: userClickPose.y - workSpaceTheme.demonstration.size,
                    left: userClickPose.x - workSpaceTheme.demonstration.size
                  }}
                />
              )}
            </div>
          </>
          <>
            <div style={{ display: 'flex', flexDirection: 'row', position: 'absolute', top: 20, left: 20 }}>
              {demoConfigAvailable &&  (
                <>
                  <label htmlFor='rows' style={{ marginRight: 10 }}>Number of Rows: </label>
                  <select id='rows' defaultValue={workSpaceConfig.x.n_segments} onChange={({ target: { value }}) => {
                    setWorkSpaceConfig(config => ({
                      ...config,
                      dimensions: config.dimensions.map(d => d.name === 'x' ? { ...d, n_segments: parseInt(value) } : d),
                      x: { ...config.x, n_segments: parseInt(value) }
                    }))
                    setDemonstrationAvailable(false)
                    setProbabilityHeatMap({})
                    setTaskInstancesToRender([])
                    setDemonstrationsToRender([])
                    setNextDemonstrationToRender([])
                    sessionStorage.removeItem('demonstrations')
                  }}>
                    {
                      [1,2,3,4,5].map((val, index) =>
                        <option key={index} value={val}>{val}</option>
                      )
                    }
                  </select>
                  <label htmlFor='columns' style={{ marginLeft: 20, marginRight: 10 }}>Number of Columns: </label>
                  <select id='columns' defaultValue={workSpaceConfig.y.n_segments} onChange={({ target: { value }}) => {
                    setWorkSpaceConfig(config => ({
                      ...config,
                      dimensions: config.dimensions.map(d => d.name === 'y' ? { ...d, n_segments: parseInt(value) } : d),
                      y: { ...config.y, n_segments: parseInt(value) }
                    }))
                    setDemonstrationAvailable(false)
                    setProbabilityHeatMap({})
                    setTaskInstancesToRender([])
                    setDemonstrationsToRender([])
                    setNextDemonstrationToRender([])
                    sessionStorage.removeItem('demonstrations')
                  }}>
                    {
                      [1,2,3,4,5].map((val, index) =>
                        <option key={index} value={val}>{val}</option>
                      )
                    }
                  </select>
                </>
              )}
            </div>
            <div style={{ display: 'flex', flexDirection: 'row', position: 'absolute', top: 20, right: 20 }}>
              {demonstrationAvailable &&  (
                <>
                  <div style={{ position: 'relative' }}>
                    <span className='material-symbols-rounded' title='Change colours' style={{ cursor: 'pointer', fontSize: 40 }} onClick={() => setThemeOpen(true)}>
                      palette
                    </span>
                  </div>
                  <div style={{ position: 'relative' }}>
                    <span className='material-symbols-rounded' title='Reset saved demonstrations' style={{ cursor: 'pointer', fontSize: 40 }} onClick={() => {
                      sessionStorage.removeItem('demonstrations')
                      setDemonstrationAvailable(false)
                      setProbabilityHeatMap({})
                      setTaskInstancesToRender([])
                      setDemonstrationsToRender([])
                      setNextDemonstrationToRender([])
                    }}>
                      delete
                      <span className='material-symbols-rounded' style={{ position: 'absolute', bottom: 0, right: 0 }}>scatter_plot</span>
                    </span>
                  </div>
                </>
              )}
              {demoConfigAvailable && robotConfigAvailable && (
                <div style={{ position: 'relative' }}>
                  <span className='material-symbols-rounded' title='Reset the robot configuration' style={{ cursor: 'pointer', fontSize: 40 }} onClick={() => {
                    localStorage.clear()
                    sessionStorage.clear()
                    setRobotConfigAvailable(false)
                    setDemoConfigAvailable(false)
                    setDemonstrationAvailable(false)
                    setProbabilityHeatMap({})
                    setTaskInstancesToRender([])
                    setDemonstrationsToRender([])
                    setNextDemonstrationToRender([])
                  }}>
                    delete
                    <span className='material-symbols-rounded' style={{ position: 'absolute', bottom: 0, right: 0 }}>precision_manufacturing</span>
                  </span>
                </div>
              )}
            </div>
          </>
        </>
      }
    </div>
  )
}
