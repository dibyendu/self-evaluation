import { useCallback, useEffect, useMemo } from 'react'

import nj from 'https://esm.run/numjs'
import { rotationMatrix } from 'https://esm.run/mathjs'

import * as d3 from 'https://esm.run/d3@6'

import * as THREE from 'https://esm.run/three'
import { GUI } from 'https://esm.run/three/examples/jsm/libs/lil-gui.module.min.js'
import { MapControls } from 'https://esm.run/three/examples/jsm/controls/MapControls.js'
import { OrbitControls } from 'https://esm.run/three/examples/jsm/controls/OrbitControls.js'


import './css/visualization.css'
import imagePath from './img/baxter_arm.png'




const jointSpaceHeight = 260,
      robotImageWidthPercent = 22

const jointNames = ['S0', 'S1', 'E0', 'E1', 'W0', 'W1', 'W2']

const imageCircleMap = {
  'S0': { cx: 75, cy: 170, rx: 40, ry: 20, rotate: 0 },
  'S1': { cx: 130, cy: 30, rx: 40, ry: 25, rotate: 45 },
  'E0': { cx: 25, cy: 145, rx: 20, ry: 35, rotate: -45 },
  'E1': { cx: 130, cy: 155, rx: 40, ry: 30, rotate: -35 },
  'W0': { cx: 165, cy: 160, rx: 35, ry: 20, rotate: -25 },
  'W1': { cx: 260, cy: 150, rx: 30, ry: 20, rotate: 0 },
  'W2': { cx: 260, cy: 200, rx: 30, ry: 20, rotate: 0 }
}

const axes = [
  { axis: 'x', color: 0xff0000 },
  { axis: 'y', color: 0x00ff00 },
  { axis: 'z', color: 0x0000ff }
]

const downsamplingRange = {
  25: 1,
  50: 2,
  75: 3,
  100: 4,
  200: 5,
  400: 6
}



export default function App() {

  const jointVisualizationLimits = useMemo(() => {
    const joint_limits = nj.array(localStorage.getItem('joint_limits').split('\n').map(row => row.split(',').map(cell => parseFloat(cell))))
    return jointNames.map((name, index) => {
      const [min_val, max_val] = joint_limits.slice([index, index + 1],null).flatten().tolist()
      return { name,  values: new Array(2).fill({ time: 0, min_val, max_val }) }
    })
  }, [])


  const [gst0, manipulator] = useMemo(() => {
    const jnt_axis = nj.array(localStorage.getItem('joint_axes').split('\n').map(row => row.split(',').map(cell => parseFloat(cell)))),
          jnt_q = nj.array(localStorage.getItem('joint_q').split('\n').map(row => row.split(',').map(cell => parseFloat(cell)))),
          joint_limits = nj.array(localStorage.getItem('joint_limits').split('\n').map(row => row.split(',').map(cell => parseFloat(cell)))),
          gst0 = nj.array(localStorage.getItem('gst0').split('\n').map(row => row.split(',').map(cell => parseFloat(cell))))

    const manipulator = jointNames.map((name, index) => {
      const _jnt_axis = nj.concatenate(jnt_axis.slice(null,[index, index + 1]).flatten(), [0]),
            normalized = Math.sqrt(nj.sum(nj.power(_jnt_axis, new Array(_jnt_axis.shape[0]).fill(2))))
      return {
        joint_type: 'Revolute',
        joint_name: name,
        joint_axis: nj.divide(_jnt_axis, new Array(_jnt_axis.shape[0]).fill(normalized)),
        joint_limits: joint_limits.slice([index, index + 1],null).flatten(),
        joint_q: nj.concatenate(jnt_q.slice(null,[index, index + 1]).flatten(), [0])
      }
    })

    return [gst0, manipulator]
  }, [])


  const forwardKinematics = useCallback(jointConfig => {
    let g_base_tool = nj.identity(4, 'float')
    manipulator.forEach((joint, index) => {
      const g = nj.identity(4, 'float')
      if (joint.joint_type === 'Revolute') { // Determine exponential for revolute joint
        const rotation_matrix = nj.array(rotationMatrix(jointConfig[index], joint.joint_axis.slice([3]).tolist()))
        g.slice([3], [3]).assign(rotation_matrix, false)
        g.slice([3], [3,4]).assign(nj.dot(nj.identity(3, 'float').subtract(rotation_matrix), joint.joint_q.slice([3])).reshape([3,1]), false)
      } else if (joint.joint_type === 'Prismatic') { // Determine exponential for prismatic joint
        const transl_axis = joint.joint_axis.slice([3])
        g.slice([3], [3,4]).assign(nj.array(new Array(transl_axis.shape[0]).fill(jointConfig[index])).multiply(transl_axis).reshape([3,1]), false)                          
      }
      g_base_tool = nj.dot(g_base_tool, g)
    })
    return nj.dot(g_base_tool, gst0)
  }, [gst0, manipulator])


  const createJointAngleChart = useCallback(data => {

    d3.select('#joint-space').selectAll('*').remove()
    d3.select('#robot-image').selectAll('*').remove()


    let margin = { top: 40, right: 40, bottom: 20, left: 60 },
        svg = d3.select('svg'),
        width = svg.attr('width') - margin.left - margin.right,
        height = svg.attr('height') - margin.top - margin.bottom


    let angleHovered = null

    const jointLimits = jointVisualizationLimits.map(({ name, values: [minVal, maxVal] }) => ({ name,  values: [minVal, { ...maxVal, time: data.at(-1).time }] }))

    //get the min and max value from joint limits
    let min_joint_val = jointLimits[0]['values'][0].min_val 
    let max_joint_val = jointLimits[0]['values'][0].max_val
    for(let i = 1; i < jointLimits.length; i++){
        min_joint_val = Math.min(min_joint_val, jointLimits[i]['values'][0].min_val)
        max_joint_val = Math.max(max_joint_val, jointLimits[i]['values'][0].max_val)
    }

    svg.attr('width', width + margin.left + margin.right)
        .attr('height', height + margin.top + margin.bottom)

    svg.append('defs')
        .append('clipPath')
        .attr('id', 'clip')
        .append('rect')
        .attr('width', width)
        .attr('height', height)

    let x = d3.scaleLinear().range([0, width]),
        y = d3.scaleLinear().range([height, 0]),
        z = d3.scaleOrdinal(d3.schemeCategory10),
        x2 = d3.scaleLinear().range([0, width]),
        y2 = d3.scaleLinear().range([height, 0])

    let line = d3.line()
                  .curve(d3.curveBasis)
                  .x(d => x(d.time))
                  .y(d => y(d.angle))

    let line2 = d3.line()
                  .curve(d3.curveBasis)
                  .x(d => x(d.time))
                  .y(d => y(d.angle))

    let focus = svg.append('g').attr('transform', `translate(${margin.left},${margin.top})`)
    let context = svg.append('g').attr('transform', `translate(${margin.left},${margin.top})`)
    let fdata = jointNames.map(name => ({
      name: name,
      values: data.map(d => ({ time: d.time, angle: d[name] }))
    }))

    x.domain([0, d3.max(data, d => d.time)])
    y.domain([min_joint_val, max_joint_val])
    z.domain(fdata.map(c => c.name))
    x2.domain(x.domain())
    y2.domain(y.domain())

    const colorLegend = (selection, props) => {
      const { z, circleRadius, spacing, textOffset } = props

      const groups = selection.selectAll('g').data(z.domain())
      const groupsEnter = groups.enter()
                                .append('g')
                                .attr('class', 'tick')
      groupsEnter
      .merge(groups)
      .attr('transform', (d, i) => `translate(${i * spacing + 80}, 0)`)
      groups.exit().remove()

      groupsEnter
      .append('circle')
      .merge(groups.select('circle'))
      .attr('r', circleRadius)
      .attr('fill', z)
      .attr('transform', (d, i) => `translate(${i * spacing + 20}, 0)`)

      groupsEnter.
      append('text')
      .merge(groups.select('text'))
      .text(d => d)
      .attr('transform', (d, i) => `translate(${i * spacing + 16}, 0)`)
      .style('font-size', '16px')
      .attr('dy', '0.4em')
      .attr('x', textOffset)
      .on('mouseenter', ({ target: { textContent: d }}) => {
        angleHovered = d
        const { cx, cy, rx, ry, rotate } = imageCircleMap[d]
        jointNames.forEach(joint =>
          focus.selectAll('#' + joint.replace(' ', '') + ' path')
                .style('stroke', z(joint))
                .style('stroke-width', joint !== angleHovered ? 0.5 : 5)
                .style('opacity', joint !== angleHovered ? 0.5 : 1)
        )
        focus.select(`#${d}limit`).style('opacity', 0.5)
        renderImage(cx, cy, rx, ry, 4, rotate, imagePath, z(d))
      })
      .on('mouseleave', ({ target: { textContent: d }}) => {
        angleHovered = null
        const { cx, cy, rx, ry, rotate } = imageCircleMap[d]
        jointNames.forEach(joint =>
          focus.selectAll('#' + joint.replace(' ', '') + ' path')
                .style('stroke', z(joint))
                .style('stroke-width', 2)
                .style('opacity', 0.8)
        )
        focus.select(`#${d}limit`).style('opacity', 0)
        renderImage(cx, cy, rx, ry, 0, rotate, imagePath)
      })
    }

    focus.append('g')
          .call(d3.axisBottom(x))
          .attr('class', 'x axis')
          .attr('transform', `translate(0,${height})`)
    focus.append('text')
          .style('text-anchor', 'end')
          .attr('y', height + 10)
          .attr('x', width + 40)
          .text('Time')

    focus.append('g')
        .attr('class', 'y axis')
        .call(d3.axisLeft(y))
    focus.append('text')
        .attr('transform', 'rotate(-90)')
        .attr('y', -40)
        .attr('x', -height / 2)
        .attr('fill', 'black')
        .style('text-anchor', 'middle')
        .text('Joint Angle ( Θᵣ )')

    let focuslineGroups = focus.selectAll('.line')
                                .data(fdata)
                                .enter().append('g')
                                .attr('id', d => d.name.replace(' ', ''))
    let focuslines = focuslineGroups.append('path')
                                    .attr('class', 'line')
                                    .attr('d', d => line(d.values))
                                    .style('stroke', d => z(d.name))
                                    .style('stroke-width', 2)
                                    .style('fill', 'none')
                                    .attr('clip-path', 'url(#clip)')

    focus.append('g')
          .attr('transform', `translate(50,-20)`)
          .call(colorLegend, { z, circleRadius: 10, spacing: 45, textOffset: 15 })

    let contextlineGroups = context.selectAll('.line')
                                    .data(fdata)
                                    .enter()
                                    .append('g')

    let contextLines = contextlineGroups.append('path')
                                        .attr('class', 'line')
                                        .attr('clip-path', 'url(#clip)')
    context.append('g')
            .attr('class', 'x axis2')
            .attr('transform', `translate(0,${height})`)
            .call(d3.axisBottom(x2).ticks(4))

    const brush = d3
                  .brushX()
                  .extent([[x.range()[0], 0], [x.range()[1], height]])
                  .on('brush end', ({ selection = null }) => {
                    x.domain(selection === null ? x2.domain() : [x2.invert(selection[0]), x2.invert(selection[1])])
                    focus.selectAll('path.line').attr('d', d => line(d.values))
                    focus.select('.x.axis').call(d3.axisBottom(x))
                    focus.select('.y.axis').call(d3.axisLeft(y))
                  })

    context.append('g')
            .attr('class', 'x brush')
            .call(brush)
            .selectAll('rect')
            .attr('y', -7)
            .attr('height', height + 7)

    const minAngle = d => d.min_val,
          maxAngle = d => d.max_val
    let areaGenerator = d3.area()
                          .x(d => x(d.time))
                          .y0(d => y(maxAngle(d)))
                          .y1(d => y(minAngle(d)))

    let area = focus.selectAll('.area')
                    .data(jointLimits)
                    .enter().append('g')
                    .attr('class', 'jointsArea')

    area.append('path')
        .attr('d', d => areaGenerator(d.values))
        .style('fill', d => z(d.name))
        .style('opacity', 0)
        .attr('id', d => d.name.replace(d.name, `${d.name}limit`))

    renderImage(0, 0, 0, 0, 0, 0, imagePath)
  }, [jointVisualizationLimits])


  const renderImage = useCallback((cx, cy, rx, ry, strokeWidth, rotate, imgPath, strokeColor='transparent') => {
    let svg = d3.select('#robot-image'),
              imgWidth = window.innerWidth * robotImageWidthPercent / 100,
              imgHeight = jointSpaceHeight,
              g = svg.append('g').attr('transform', 'translate(0,0)')

    g
    .append('image')
    .attr('href', imgPath)
    .attr('width', imgWidth)
    .attr('height', imgHeight)
    .attr('x', 0)
    .attr('y', 0)

    g
    .append('g')
    .attr('transform', 'translate(0,0)')
    .append('ellipse')
    .attr('transform', `rotate(${rotate})`)
    .attr('cx', cx)
    .attr('cy', cy)
    .attr('rx', rx)
    .attr('ry', ry)
    .attr('stroke', strokeColor)
    .attr('stroke-width', strokeWidth)
    .attr('fill', 'none')
    .transition()
    .duration(8000)
  }, [])


  // useEffect(() => {
  //   if (forwardKinematics === undefined) return

  //   const { plan, timestamps, position: objectPosition, screw_segments } = JSON.parse(localStorage.getItem('navigation_state'))
  //   localStorage.removeItem('navigation_state')


  //   const _timestamps = timestamps === undefined ? new Array(plan.length).fill().map((_, i) => i * 0.01) : timestamps

  //   const jointVisualizationData = _timestamps.map((t, i, arr) =>
  //     i === 0 ?
  //     { ...Object.fromEntries(jointNames.map((n, j) => [n, `${plan[i][j]}`])), time: 0 } :
  //     { ...Object.fromEntries(jointNames.map((n, j) => [n, `${plan[i][j]}`])), time: t - arr[0] }
  //   )

  //   createJointAngleChart(jointVisualizationData)

  //   const segmentedPlan = (screw_segments === undefined || screw_segments.length === 0)
  //                         ? [plan]
  //                         : [plan.slice(0, screw_segments[0] + 1)].concat(screw_segments.map((i, idx, array) => plan.slice(i + 1, idx < array.length - 1 ? array[idx + 1] + 1 : plan.length)))

  //   const screwSegments = segmentedPlan.map(segment =>
  //     segment.map(jointConfig => {
  //       const se3_pose = forwardKinematics(jointConfig),
  //             translation = nj.flatten(se3_pose.slice([3], [3,4])).tolist(),
  //             rotation = new THREE.Euler().setFromRotationMatrix(new THREE.Matrix4(...se3_pose.tolist().flat()).transpose())
  //       return { translation, rotation: [rotation.x, rotation.y, rotation.z] }
  //     })
  //   ).filter(segment => segment.length !== 0)


  //   let container
  //   let camera, scene, renderer

  //   const distanceScale = 500,
  //         poseObjectSize = 8,
  //         bowlRadius = 80

  //   function init() {
  //     container = document.getElementById('task-space')

  //     scene = new THREE.Scene()
  //     scene.background = new THREE.Color(0xf0f0f0)

  //     camera = new THREE.PerspectiveCamera(
  //       70,
  //       window.innerWidth / (window.innerHeight - jointSpaceHeight),
  //       1,
  //       10000
  //     )
  //     camera.position.set(0, 200, 800)
  //     scene.add(camera)

  //     scene.add(new THREE.AmbientLight(0xf0f0f0, 3))
  //     const light = new THREE.SpotLight(0xffffff, 4.5)
  //     light.position.set(0, 1500, 200)
  //     light.angle = Math.PI * 0.2
  //     light.decay = 0
  //     light.castShadow = true
  //     light.shadow.camera.near = 200
  //     light.shadow.camera.far = 2000
  //     light.shadow.bias = -0.000222
  //     light.shadow.mapSize.width = 1024
  //     light.shadow.mapSize.height = 1024
  //     scene.add(light)

  //     const planeGeometry = new THREE.PlaneGeometry(2000, 2000)
  //     planeGeometry.rotateX(-Math.PI / 2)
  //     const planeMaterial = new THREE.ShadowMaterial({ color: 0x000000, opacity: 0.2 })

  //     const plane = new THREE.Mesh(planeGeometry, planeMaterial)
  //     plane.position.y = -200
  //     plane.receiveShadow = true
  //     scene.add(plane)

  //     axes.forEach(({ axis, color }) => {
  //       const origin = new THREE.Vector3(-2000 / 2, -200, -2000 / 2),
  //             lineEnd = origin.clone()
  //       lineEnd[axis] += 2000 / 4
  //       const axisMaterial = new THREE.LineBasicMaterial( { color, linewidth: 2 })
  //       const axisGeometry = new THREE.BufferGeometry().setFromPoints([origin, lineEnd])
  //       const axisObject = new THREE.LineSegments(axisGeometry, axisMaterial)
  //       scene.add(axisObject)
  //     })

  //     const helper = new THREE.GridHelper(2000, 100)
  //     helper.position.y = -199
  //     helper.material.opacity = 0.25
  //     helper.material.transparent = true
  //     scene.add(helper)

  //     renderer = new THREE.WebGLRenderer({ antialias: true })
  //     renderer.setPixelRatio(window.devicePixelRatio)
  //     renderer.setSize(window.innerWidth, window.innerHeight - jointSpaceHeight)
  //     renderer.shadowMap.enabled = true
  //     container.appendChild(renderer.domElement)

  //     // Controls
  //     const controls = new OrbitControls(camera, renderer.domElement)
  //     controls.damping = 0.2
  //     controls.addEventListener('change', render)

  //     window.addEventListener('resize', () => {
  //       camera.aspect = window.innerWidth / window.innerHeight
  //       camera.updateProjectionMatrix()
  //       renderer.setSize(window.innerWidth, window.innerHeight)
  //       render()
  //     })

  //     const bowlColor = `#${Math.floor(Math.random()*16777215).toString(16)}`,
  //           bowlGeometry = new THREE.SphereGeometry(bowlRadius, 30, 30, 0, Math.PI),
  //           bowlMaterial = new THREE.MeshLambertMaterial({ color: bowlColor, emissive: 0x000000, side: THREE.DoubleSide, flatShading: true }),
  //           bowl = new THREE.Mesh(bowlGeometry, bowlMaterial)
  //     bowl.rotateX(Math.PI / 2)
  //     bowl.position.copy(new THREE.Vector3(...objectPosition.map(p => p * distanceScale)))
  //     bowl.castShadow = bowl.receiveShadow = true
  //     scene.add(bowl)

  //     const bowlRimGeometry = new THREE.TorusGeometry(bowlRadius, 4, 20, 100),
  //           bowlRimMaterial = new THREE.MeshLambertMaterial({ color: bowlColor, emissive: 0x000000, side: THREE.DoubleSide, flatShading: true }),
  //           bowlRim = new THREE.Mesh(bowlRimGeometry, bowlRimMaterial)
  //     bowlRim.rotateX(Math.PI / 2)
  //     bowlRim.position.copy(new THREE.Vector3(...objectPosition.map(p => p * distanceScale)))
  //     scene.add(bowlRim)

  //     screwSegments.forEach((segment, index) => drawSegment(segment, index))        

  //     render()
  //   }


  //   function drawSegment(segment, segmentIndex) {
  //     const color = `#${Math.floor(Math.random()*16777215).toString(16)}`

  //     const points = segment.map(({ translation, rotation: [x,y,z] }) =>
  //       [new THREE.Vector3(...translation.map(p => p * distanceScale)), {x,y,z}]
  //     )

  //     const range = Object.entries(downsamplingRange)
  //     .map(([k, v]) => [parseInt(k), v])
  //     .toSorted(([a], [b]) => a - b)
  //     .map(([k ,v], i, arr) =>
  //       i === 0 ? new Array(k).fill(0) : new Array(k - arr[i-1][0]).fill(arr[i-1][1]).concat(i === arr.length - 1 ? [v] : [])
  //     )
  //     .flat()
  //     const nIntermediatePoints = points.length >= range.length ? range.at(-1) : range[points.length]
  //     const landMarks = new Array(nIntermediatePoints).fill().map((_, i) => (i + 1) * parseInt(points.length / (nIntermediatePoints + 1))).concat(points.length ? [points.length - 1] : [])
  //     if (segmentIndex === 0 && points.length !== 0)
  //       landMarks.unshift(0)

  //     points.forEach(([point, rotation], index, array) => {
  //       if (index < array.length - 1) {
  //         const path = new THREE.LineCurve3(point, array[index+1][0]),
  //               pathObject = new THREE.Line(
  //                 new THREE.BufferGeometry().setFromPoints([point, array[index+1][0]]),
  //                 new THREE.LineBasicMaterial({ color, linewidth: 10 })
  //               )
  //         pathObject.castShadow = true
  //         scene.add(pathObject)
  //       }
  //       if (landMarks.includes(index)) {
  //         const object = createPose(point, rotation)
  //         scene.add(object)
  //       }
  //     })
  //   }


  //   function createPose(position, rotation) {
  //     const geometry = new THREE.BoxGeometry(poseObjectSize, poseObjectSize, poseObjectSize).toNonIndexed()
  //     const material = new THREE.MeshLambertMaterial({ vertexColors: true })

  //     const color = new THREE.Color()
  //     const colors = []
  //     const colorMap = {
  //       0: 0xff0000, // +x
  //       1: 0x440000, // -x
  //       2: 0x00ff00, // +y
  //       3: 0x004400, // -y
  //       4: 0x0000ff, // +z
  //       5: 0x000044, // -z
  //     }
  //     for (let i = 0; i < geometry.getAttribute('position').count; i += 6) {
  //       color.set(colorMap[i / 6])
  //       for (let j = 0; j < 6; j++)
  //         colors.push(color.r, color.g, color.b)
  //     }        
  //     geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3))

  //     const object = new THREE.Mesh(geometry, material)
  //     object.position.copy(position) 

  //     Object.entries(rotation).forEach(([axis, angle]) => {
  //       object.rotation[axis] = angle
  //     })

  //     axes.forEach(({ axis, color }) => {
  //       const objectPose = new THREE.Vector3(object.position.x, object.position.y, object.position.z),
  //             origin = new THREE.Vector3(0, 0, 0),
  //             lineEnd = origin.clone()
  //       lineEnd[axis] += poseObjectSize * 2
  //       const axisObject = new THREE.LineSegments(
  //         new THREE.BufferGeometry().setFromPoints([origin, lineEnd]),
  //         new THREE.LineDashedMaterial({ color, dashSize: 1, gapSize: 1 })
  //       )

  //       Object.entries(rotation).forEach(([axis, angle]) => {
  //         axisObject.rotation[axis] = angle
  //       })

  //       axisObject.position.add(objectPose)
  //       axisObject.computeLineDistances()
  //       scene.add(axisObject)
  //     })

  //     object.castShadow = object.receiveShadow = true
  //     scene.add(object)
  //     return object
  //   }


  //   function render() {
  //     renderer.render(scene, camera)
  //   }

  //   init()
  // }, [forwardKinematics])


  useEffect(() => {
    const positionScale = 1.5,
          bowlRadius = 0.25,
          poseObjectSize = 0.02

    if (forwardKinematics === undefined) return

    const container = document.getElementById('task-space')

    let { plan, timestamps, position: objectPosition, screw_segments } = JSON.parse(localStorage.getItem('navigation_state'))
    localStorage.removeItem('navigation_state')


    objectPosition = objectPosition.map(p => p * positionScale)

    timestamps = timestamps === undefined ? new Array(plan.length).fill().map((_, i) => i * 0.01) : timestamps
    const jointVisualizationData = timestamps.map((t, i, arr) =>
      i === 0 ?
      { ...Object.fromEntries(jointNames.map((n, j) => [n, `${plan[i][j]}`])), time: 0 } :
      { ...Object.fromEntries(jointNames.map((n, j) => [n, `${plan[i][j]}`])), time: t - arr[0] }
    )

    const segmentedPlan = (screw_segments === undefined || screw_segments.length === 0)
                          ? [plan]
                          : [plan.slice(0, screw_segments[0] + 1)].concat(screw_segments.map((i, idx, array) => plan.slice(i + 1, idx < array.length - 1 ? array[idx + 1] + 1 : plan.length)))

    const screwSegments = segmentedPlan.map(segment =>
      segment.map(jointConfig => {
        const se3_pose = forwardKinematics(jointConfig),
              translation = nj.flatten(se3_pose.slice([3], [3,4])).tolist().map(p => p * positionScale),
              rotation = new THREE.Euler().setFromRotationMatrix(new THREE.Matrix4(...se3_pose.tolist().flat()).transpose())
        return { translation, rotation: [rotation.x, rotation.y, rotation.z] }
      })
    ).filter(segment => segment.length !== 0)


    const scene = new THREE.Scene()
    scene.background = new THREE.Color(0xf0f0f0)

    const camera = new THREE.PerspectiveCamera(90, window.innerWidth / (window.innerHeight - jointSpaceHeight), 0.01, 100)
    camera.position.set(0, -1, 1.5 + 0.01)
    camera.up.set(0, 0, 1)   // spin around z-axis
    scene.add(camera)

    scene.add(new THREE.AmbientLight(0xf0f0f0, 1.5))
    const light = new THREE.SpotLight(0xffffff, 4.5)
    light.position.set(0, 0, 2)
    light.decay = 0
    light.castShadow = true
    scene.add(light)

    const renderer = new THREE.WebGLRenderer({ antialias: true })
    renderer.setPixelRatio(window.devicePixelRatio)
    renderer.setSize(window.innerWidth, window.innerHeight - jointSpaceHeight)
    renderer.shadowMap.enabled = true
    container.appendChild(renderer.domElement)

    const orbitControls = new OrbitControls(camera, renderer.domElement)
    orbitControls.enableDamping = true
    orbitControls.damping = 0.2
    orbitControls.enabled = true
    orbitControls.addEventListener('change', render)

    const mapControls = new MapControls(camera, renderer.domElement)
    mapControls.enableDamping = true
    mapControls.dampingFactor = 0.05
    mapControls.enabled = false
    mapControls.screenSpacePanning = false
    mapControls.addEventListener('change', render)

    orbitControls.zoomToCursor = mapControls.zoomToCursor = false

    window.addEventListener('resize', () => {
      camera.aspect = window.innerWidth / window.innerHeight
      camera.updateProjectionMatrix()
      renderer.setSize(window.innerWidth, window.innerHeight)
      render()
    }, false)

    const frameGroup = new THREE.Group()

    const planeGeometry = new THREE.PlaneGeometry(2, 4),
          planeMaterial = new THREE.ShadowMaterial({ color: 0x000000, opacity: 0.2 }),
          plane = new THREE.Mesh(planeGeometry, planeMaterial)
    plane.receiveShadow = true
    frameGroup.add(plane)

    const helpers = [
      new THREE.GridHelper(2, 40, 0x888888),
      new THREE.GridHelper(2, 40, 0x888888)
    ]
    helpers.forEach(helper => {
      helper.rotateX(Math.PI / 2)
      helper.material.opacity = 0.25
      helper.material.transparent = true
    })
    helpers[0].position.y = 1
    helpers[1].position.y = -1
    frameGroup.add(...helpers)

    axes.forEach(({ axis, color }) => {
      const origin = new THREE.Vector3(-1, -2, 0),
            lineEnd = origin.clone()
      lineEnd[axis] += 0.2
      const axisGeometry = new THREE.BufferGeometry().setFromPoints([origin, lineEnd]),
            axisMaterial = new THREE.LineBasicMaterial( { color, linewidth: 2 }),
            axisObject = new THREE.LineSegments(axisGeometry, axisMaterial)
      frameGroup.add(axisObject)
    })


    const objectGroup = new THREE.Group()

    const bowlColor = `#${Math.floor(Math.random()*16777215).toString(16)}`,
          bowlGeometry = new THREE.SphereGeometry(bowlRadius, 30, 30, 0, -Math.PI),
          bowlMaterial = new THREE.MeshLambertMaterial({ color: bowlColor, emissive: 0x000000, side: THREE.DoubleSide, flatShading: true }),
          bowl = new THREE.Mesh(bowlGeometry, bowlMaterial)
    bowl.position.copy(new THREE.Vector3(...objectPosition))
    bowl.castShadow = bowl.receiveShadow = true
    objectGroup.add(bowl)

    const bowlRimGeometry = new THREE.TorusGeometry(bowlRadius, bowlRadius * 5 / 100, 20, 100),
          bowlRimMaterial = new THREE.MeshLambertMaterial({ color: bowlColor, emissive: 0x000000, side: THREE.DoubleSide, flatShading: true }),
          bowlRim = new THREE.Mesh(bowlRimGeometry, bowlRimMaterial)
    bowlRim.position.copy(new THREE.Vector3(...objectPosition))
    objectGroup.add(bowlRim)

    const gui = new GUI({ title: 'Navigation Controls', container: document.getElementById('control-gui') })

    gui.add(orbitControls, 'zoomToCursor').name('Zoom to Cursor').onChange(value => {
      mapControls.zoomToCursor = value
    })

    const orbitControlsFolder = gui.addFolder('Rotation'),
          orbitEnabledController = orbitControlsFolder.add(orbitControls, 'enabled').name('Enable')

    const mapControlsFolder = gui.addFolder('Translation'),
          mapEnabledController = mapControlsFolder.add(mapControls, 'enabled').name('Enable')
    mapControlsFolder.add(mapControls, 'screenSpacePanning').name('Screen Space Panning')

    orbitEnabledController.onChange(value => {
      mapControls.enabled = value ? false : mapControls.enabled
      mapEnabledController.updateDisplay()
    })
    mapEnabledController.onChange(value => {
      orbitControls.enabled = value ? false : orbitControls.enabled
      orbitEnabledController.updateDisplay()
    })

    screwSegments.forEach((segment, index) => drawSegment(segment, index))

    function render() {
      renderer.render(scene, camera)
    }


    function drawSegment(segment, segmentIndex) {
      const color = `#${Math.floor(Math.random()*16777215).toString(16)}`

      const points = segment.map(({ translation, rotation: [x,y,z] }) =>
        [new THREE.Vector3(...translation), {x,y,z}]
      )

      const range = Object.entries(downsamplingRange)
      .map(([k, v]) => [parseInt(k), v])
      .toSorted(([a], [b]) => a - b)
      .map(([k ,v], i, arr) =>
        i === 0 ? new Array(k).fill(0) : new Array(k - arr[i-1][0]).fill(arr[i-1][1]).concat(i === arr.length - 1 ? [v] : [])
      )
      .flat()
      const nIntermediatePoints = points.length >= range.length ? range.at(-1) : range[points.length]
      const landMarks = new Array(nIntermediatePoints).fill().map((_, i) => (i + 1) * parseInt(points.length / (nIntermediatePoints + 1))).concat(points.length ? [points.length - 1] : [])
      if (segmentIndex === 0 && points.length !== 0)
        landMarks.unshift(0)


      console.log(points.length)
      console.log(landMarks)


      points.forEach(([point, rotation], index, array) => {
        if (index < array.length - 1) {
          const path = new THREE.LineCurve3(point, array[index+1][0]),
                pathObject = new THREE.Line(
                  new THREE.BufferGeometry().setFromPoints([point, array[index+1][0]]),
                  new THREE.LineBasicMaterial({ color, linewidth: 10 })
                )
          pathObject.castShadow = true
          objectGroup.add(pathObject)
        }
        if (landMarks.includes(index)) {
          const object = createPose(point, rotation)
          objectGroup.add(object)
        }
      })
    }


    function createPose(position, rotation) {
      const geometry = new THREE.BoxGeometry(poseObjectSize, poseObjectSize, poseObjectSize).toNonIndexed(),
            material = new THREE.MeshLambertMaterial({ vertexColors: true }),
            color = new THREE.Color()
      
      const colors = []
      const colorMap = {
        0: 0xff0000, // +x
        1: 0x440000, // -x
        2: 0x00ff00, // +y
        3: 0x004400, // -y
        4: 0x0000ff, // +z
        5: 0x000044, // -z
      }
      for (let i = 0; i < geometry.getAttribute('position').count; i += 6) {
        color.set(colorMap[i / 6])
        for (let j = 0; j < 6; j++)
          colors.push(color.r, color.g, color.b)
      }        
      geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3))

      const object = new THREE.Mesh(geometry, material)
      object.position.copy(position) 

      Object.entries(rotation).forEach(([axis, angle]) => {
        object.rotation[axis] = angle
      })

      axes.forEach(({ axis, color }) => {
        const objectPose = new THREE.Vector3(object.position.x, object.position.y, object.position.z),
              origin = new THREE.Vector3(0, 0, 0),
              lineEnd = origin.clone()
        lineEnd[axis] += poseObjectSize * 1.5
        const axisObject = new THREE.LineSegments(
          new THREE.BufferGeometry().setFromPoints([origin, lineEnd]),
          new THREE.LineDashedMaterial({ color, dashSize: 1, gapSize: 1 })
        )

        Object.entries(rotation).forEach(([axis, angle]) => {
          axisObject.rotation[axis] = angle
        })

        axisObject.position.add(objectPose)
        axisObject.computeLineDistances()
        objectGroup.add(axisObject)
      })

      object.castShadow = object.receiveShadow = true
      objectGroup.add(object)
      return object
    }


    frameGroup.rotation.z = objectGroup.rotation.z = Math.PI / 2

    objectGroup.translateX(-bowl.position.x)
    objectGroup.translateY(-bowl.position.y)
    frameGroup.translateZ(-bowlRadius * positionScale)

    scene.add(frameGroup)
    scene.add(objectGroup)

    render()

    createJointAngleChart(jointVisualizationData)
  }, [forwardKinematics])


  return (
    <>
      <>
        <svg id='joint-space' width={window.innerWidth * (100 - robotImageWidthPercent) / 100} height={jointSpaceHeight}></svg>
        <svg id='robot-image' width={window.innerWidth * robotImageWidthPercent / 100} height={jointSpaceHeight}></svg>
      </>
      <div style={{ position: 'relative' }}>
        <div id='task-space'></div>
        <div id='control-gui' style={{ position: 'absolute', top: 0, right: 0 }}></div>
      </div>
    </>
  )
}
