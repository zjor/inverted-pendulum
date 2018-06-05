const sin = Math.sin
const cos = Math.cos

const width = 400
const heigth = 400
const boxWidth = 40
const boxHeight = 20
const boxY = heigth / 2

const l = 10
const g = 9.8
let x0 = 0.0

let x = 0.0
let theta = Math.PI

const thetaThreshold = Math.PI / 15

// z = theta'
let z = 0.0
// y = x'
let y = 0.0

let time = 0.0

let lastControl = 0.0

function draw(ctx) {
	ctx.clearRect(0, 0, width, heigth)
	ctx.beginPath()
	ctx.moveTo(0, (heigth + boxHeight) / 2)
	ctx.lineTo(width, (heigth + boxHeight) / 2)

	const scale = 10.0
	const tX = x * scale + width / 2

	ctx.rect(tX - boxWidth / 2, boxY - boxHeight / 2, boxWidth, boxHeight)

	const scaledL = l * scale
	const rodX = scaledL * Math.sin(theta) + tX
	const rodY = boxY - scaledL * Math.cos(theta)
	ctx.moveTo(tX, boxY)
	ctx.lineTo(rodX, rodY)	
	ctx.stroke()

	ctx.beginPath()
	ctx.arc(rodX, rodY, 10, 0, 2.0 * Math.PI)
	ctx.stroke()

	const thetaVisual = Math.round(((theta * 180.0 / Math.PI) % 360) * 100) / 100
	ctx.strokeText("Theta: " + thetaVisual, 5, 15)
	ctx.strokeText("F: " + Math.round(lastControl * 100) / 100, 5, 25)
	ctx.strokeText("Controllable: " + isControllable(theta), 5, 35)

	if (time == 0.0) {
		time = performance.now()
	} else {
		const now = performance.now()
		integrate((now - time) / 1000)
		time = now
	}
}

const Kp = 100.0
const Kd = 50.0

const xKp = 2.0
const xKd = 3.5

const w = Math.sqrt(g / l)
const A = 1.0

function isControllable(th) {
	let nTh = th % (Math.PI * 2.0)
	nTh = (nTh > Math.PI) ? 2 * Math.PI - nTh : nTh
	return nTh < thetaThreshold
}

function control(th, dth, x, dx) {
	if (isControllable(th)) {
		return Kp * th + Kd * dth + xKp * (x - x0) + xKd * dx
	} else {
		return A * cos(w * time / 1000)
	}
}

function integrate(h) {
	const ddx = control(theta, z, x, y)
	lastControl = ddx

	const th1 = theta + h * z
	const z1 = z + h * (g * sin(theta) - ddx * cos(theta)) / l
	const th2 = theta + h / 2 * (z + z1)
	const z2 = z + h / 2 * g / l * (sin(theta) + sin(th1) - ddx * (cos(theta) + cos(th1)))
	
	const x1 = x + h * y
	const y1 = y + h * ddx

	theta = th2
	z = z2

	if (isControllable(theta))	{
		y = y1
		x = x1
	} else {
		y = A * sin(w * time / 1000) / w
		x = - A * cos(w * time / 1000) / w / w	
	}
}

$(() => {
	let canvas = document.getElementById("canvas")
	let ctx = canvas.getContext("2d")
	let started = true

	const drawCycle = (ctx, timeout) => {
		draw(ctx)
		if (started) {
			setTimeout(() => { drawCycle(ctx, timeout) }, timeout)
		}
	}			

	$('#start').click(() => {
		started = true
		drawCycle(ctx, 50)
	})

	$('#stop').click(() => {
		started = false
	})

	$('#left').click(() => {
		x0 -= 10
	})

	$('#right').click(() => {
		x0 += 10
	})

})