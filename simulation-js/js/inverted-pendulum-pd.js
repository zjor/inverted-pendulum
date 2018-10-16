const sin = Math.sin
const cos = Math.cos

const width = 400
const heigth = 400
const boxWidth = 40
const boxHeight = 20
const boxY = heigth / 2

const m = 1.0
const l = 10
const g = 9.8
let x0 = 0.0

let x = 0.0
let theta = Math.PI / 20

// z = theta'
let z = 0.0
// y = x'
let y = 0.0

let time = 0.0

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
	

	if (time == 0.0) {
		time = performance.now()
	} else {
		const now = performance.now()
		integrate((now - time) / 1000)
		time = now
	}
}

const Kp = 49.29
const Kd = 49.77

const xKp = 0.44
const xKd = 1.94

function normTheta(th) {
	let nTh = th % (Math.PI * 2.0)
	return (nTh > Math.PI) ? nTh - 2 * Math.PI : nTh
}

function energy() {
	const p = l * (1 + cos(theta)) * m * g
	const k = m * (z * l) ** 2 / 2
	return {potential: p, kinetic: k, total: p + k}
}

function control(th, dth, x, dx) {
	return Kp * normTheta(th) + Kd * dth + xKp * (x - x0) + xKd * dx
}

function integrate(h) {
	const ddx = control(theta, z, x, y)
	console.log(ddx, theta, z, (x - x0), y, "E: ", energy())

	const ddth = (g * sin(theta) - ddx * cos(theta)) / l

	const th1 = theta + h * z
	const z1 = z + h * ddth
	
	const x1 = x + h * y
	const y1 = y + h * ddx

	theta = th1
	z = z1
	y = y1
	x = x1
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