const { sin, cos, sqrt, abs, PI } = Math

const width = 800
const heigth = 300
const boxWidth = 40
const boxHeight = 20
const boxY = heigth / 2

const m = 1.0
const l = 10
const g = 9.8
let x0 = 0.0

let x = -10.0
let theta = PI - 0.3

// z = theta'
let z = 0.0
// y = x'
let y = 0.0

let t = 0.0
const h = 0.05

const round = (n) => Math.round(n * 100.0) / 100.0

function draw(ctx) {
	ctx.clearRect(0, 0, width, heigth)
	ctx.beginPath()
	ctx.moveTo(0, (heigth + boxHeight) / 2)
	ctx.lineTo(width, (heigth + boxHeight) / 2)

	const scale = 10.0
	const tX = x * scale + width / 2

	ctx.rect(tX - boxWidth / 2, boxY - boxHeight / 2, boxWidth, boxHeight)

	const scaledL = l * scale
	const rodX = scaledL * sin(theta) + tX
	const rodY = boxY - scaledL * cos(theta)
	ctx.moveTo(tX, boxY)
	ctx.lineTo(rodX, rodY)	
	ctx.stroke()

	ctx.beginPath()
	ctx.arc(rodX, rodY, 10, 0, 2.0 * Math.PI)
	ctx.stroke()

	const thetaVisual = round((theta * 180.0 / PI) % 360)
	ctx.strokeText("Theta: " + thetaVisual, 5, 15)
	ctx.strokeText("Position: " + round(x), 5, 30)
	ctx.strokeText("Energy: " + round(energy().total), 5, 45)

	integrate(h)
	t += h
	
}

const Kp = 80.0
const Kd = 100.0

const xKp = 0.5
const xKd = 1.0

function energy() {
	const p = l * (1 + cos(theta)) * m * g
	const k = m * (z * l) ** 2 / 2
	return {potential: p, kinetic: k, total: p + k}
}

function normTheta(th) {
	let nTh = th % (PI * 2.0)
	return (nTh > PI) ? nTh - 2 * PI : nTh
}

function control(th, dth, x, dx, t) {
	const threshold = PI / 6
	const complement = PI - threshold

	if (abs(normTheta(th)) < threshold && abs(dth) < 0.2) {
		return Kp * normTheta(th) + Kd * dth + xKp * (x - x0) + xKd * dx
	} else {
		if (abs(th) > PI / 2) {
			return 2.0 * abs(complement - th) / complement * dth
		} else {
			return 0.0
		}
	}
}

function integrate(h) {
	const ddx = control(theta, z, x, y, t)

	console.log("h=", h, x, y, ddx, 'dth=', z)

	const th1 = theta + h * z
	const z1 = z + h * (g * sin(theta) - ddx * cos(theta)) / l
	const th2 = theta + h / 2 * (z + z1)
	const z2 = z + h / 2 * (g * (sin(theta) + sin(th1)) - ddx * (cos(theta) + cos(th1))) / l

	// const ddth = (g * sin(theta) - ddx * cos(theta)) / l

	// const th1 = theta + h * z
	// const z1 = z + h * ddth

	const x1 = x + h * y
	const y1 = y + h * ddx

	const x2 = x + h / 2 * (y + y1)
	const y2 = y + h / 2 * (ddx + control(theta, z, x, y, t))


	theta = th2
	z = z2
	y = y2
	x = x2	
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
		drawCycle(ctx, 5)
	})

	$('#stop').click(() => {
		started = false
	})
})