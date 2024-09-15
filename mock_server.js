import { oakCors } from 'https://deno.land/x/cors/mod.ts'
import { Application, Router, Status } from 'https://deno.land/x/oak/mod.ts'


const demonstrations_path = 'demonstrations'

const config = JSON.parse(Deno.readTextFileSync('demonstration_config.json')),
      {min: xMin, max: xMax} = config.dimensions.find(c => c.name === 'x'),
      {min: yMin, max: yMax} = config.dimensions.find(c => c.name === 'y')


function boxMullerTransform() {
  const u1 = Math.random(),
        u2 = Math.random()
  
  const z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(2.0 * Math.PI * u2),
        z1 = Math.sqrt(-2.0 * Math.log(u1)) * Math.sin(2.0 * Math.PI * u2)
  
  return { z0, z1 }
}

function getNormallyDistributedRandomNumber(mean, stddev) {
  const { z0, _ } = boxMullerTransform()
  return z0 * stddev + mean
}


const router = new Router()

router
.get('/init', context => {
  context.response.body = { status: true }
})
.get('/pose', context => {
  const x = Math.random() * (xMax - xMin) + xMin,
        y = Math.random() * (yMax - yMin) + yMin

  const exponent = Math.floor(Math.random() * (7 - 3 + 1) + 3),
        variance = 1.0 / 10**exponent

  const sampleX = [],
        sampleY = []
  for (let i = 0; i < 10; i += 1) {
      sampleX.push(getNormallyDistributedRandomNumber(x, Math.sqrt(variance)))
      sampleY.push(getNormallyDistributedRandomNumber(y, Math.sqrt(variance)))
  }

  const meanX = sampleX.reduce((acc, i) => acc + i, 0) / sampleX.length,
        meanY = sampleY.reduce((acc, i) => acc + i, 0) / sampleY.length,
        varX = sampleX.reduce((acc, i) => acc + (i - meanX)**2, 0) / sampleX.length,
        varY = sampleY.reduce((acc, i) => acc + (i - meanY)**2, 0) / sampleY.length

  context.response.body = { success: true, x: meanX, y: meanY, variance: [varX, varY] }
})
.post('/start', async context => {    
  // const { object_location: { x, y }, task } = await context.request.body.json()
  const task = context.request.headers.get('X-Self-Evaluation-Task')
  const { x, y } = JSON.parse(context.request.headers.get('X-Self-Evaluation-Object-Location'))
  let demoIndex = 1
  let minDistance = Number.MAX_VALUE
  for await (const dirEntry of Deno.readDir(`${demonstrations_path}/interactive_${task}`)) {
    const pose = Deno.readTextFileSync(`${demonstrations_path}/interactive_${task}/${dirEntry.name}/object_poses.csv`),
          [demoX, demoY] = pose.split('\n').slice(0, 2).map(str => parseFloat(str.trim().split(',').at(-1))),
          dist = (x - demoX)**2 + (y - demoY)**2
    if (dist < minDistance) {
      minDistance = dist
      demoIndex = parseInt(dirEntry.name.substr(4))
    }
  }
  context.response.body = { pid: demoIndex }
})
.get('/stop/:index', context => {
  const task = context.request.headers.get('X-Self-Evaluation-Task')
  const pose = Deno.readTextFileSync(`${demonstrations_path}/interactive_${task}/demo${context.params.index}/object_poses.csv`),
        [demoX, demoY] = pose.split('\n').slice(0, 2).map(str => parseFloat(str.trim().split(',').at(-1))),
        demonstration = Deno.readTextFileSync(`${demonstrations_path}/interactive_${task}/demo${context.params.index}/joint_angles.csv`)
  context.response.body = { demonstration, pose: { x: demoX, y: demoY }}
})

const app = new Application()

app.use(async (context, next) => {
  await next()
  context.response.type = 'json'
  context.response.status = Status.OK
})
app.use(oakCors({
  origin: '*',
  optionsSuccessStatus: 200
}))
app.use(router.routes())
app.use(router.allowedMethods())

await app.listen({ port: 8000 })
