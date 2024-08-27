import { oakCors } from 'https://deno.land/x/cors/mod.ts'
import { Application, Router, Status } from 'https://deno.land/x/oak/mod.ts'


const config = JSON.parse(Deno.readTextFileSync('demonstration_config.json')),
      {min: xMin, max: xMax} = config.dimensions.find(c => c.name === 'x'),
      {min: yMin, max: yMax} = config.dimensions.find(c => c.name === 'y')


const router = new Router()

router
.get('/init', context => {
  context.response.body = { status: true }
})
.get('/pose', context => {
  context.response.body = { success: true, x: Math.random() * (xMax - xMin) + xMin, y: Math.random() * (yMax - yMin) + yMin }
})
.post('/start', async context => {    
  const { object_location: { x, y }} = await context.request.body.json()
  let demoIndex = 1
  let minDistance = Number.MAX_VALUE
  for await (const dirEntry of Deno.readDir('demonstrations')) {
    const pose = Deno.readTextFileSync(`demonstrations/${dirEntry.name}/object_poses.csv`),
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
  const demonstration = Deno.readTextFileSync(`demonstrations/demo${context.params.index}/joint_angles.csv`)
  context.response.body = { demonstration }
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
