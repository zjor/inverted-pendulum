const sin = Math.sin
const cos = Math.cos

const width = 400
const heigth = 300
const boxWidth = 40
const boxHeight = 20
const boxY = heigth / 2

const m = 1.0
const l = 10
const g = 9.8
let x0 = 0.0

let x = -10.0
let theta = Math.PI

// z = theta'
let z = 0.0
// y = x'
let y = 0.0

let time = 0.0

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
	const rodX = scaledL * Math.sin(theta) + tX
	const rodY = boxY - scaledL * Math.cos(theta)
	ctx.moveTo(tX, boxY)
	ctx.lineTo(rodX, rodY)	
	ctx.stroke()

	ctx.beginPath()
	ctx.arc(rodX, rodY, 10, 0, 2.0 * Math.PI)
	ctx.stroke()

	const thetaVisual = round((theta * 180.0 / Math.PI) % 360)
	ctx.strokeText("Theta: " + thetaVisual, 5, 15)
	ctx.strokeText("Energy: " + round(energy().total), 5, 45)

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

const hdth = []
const hddth = []

function energy() {
	const p = l * (1 + cos(theta)) * m * g
	const k = m * (z * l) ** 2 / 2
	return {potential: p, kinetic: k, total: p + k}
}

function normTheta(th) {
	let nTh = th % (Math.PI * 2.0)
	return (nTh > Math.PI) ? nTh - 2 * Math.PI : nTh
}

const last = (list) => list[list.length - 1]

function control(th, dth, x, dx) {
	
	if (hdth.length < 2) {
		return 1.0
	} else {
		if (last(hdth) > 0 && last(hddth) > 0) {
			return 1.0
		} else if (last(hdth) > 0 && last(hddth) < 0) {
			return -1.0
		} else if (last(hdth) < 0 && last(hddth) > 0) {
			return 1.0
		} else if (last(hdth) < 0 && last(hddth) < 0) {
			return -1.0

		}
		return 0.0
	}	
}

function integrate(h) {
	const ddx = control(theta, z, x, y)

	console.log(last(hdth), last(hddth), ddx)

	// const th1 = theta + h * z
	// const z1 = z + h * (g * sin(theta) - ddx * cos(theta)) / l
	// const th2 = theta + h / 2 * (z + z1)
	// const z2 = z + h / 2 * g / l * (sin(theta) + sin(th1) - ddx * (cos(theta) + cos(th1)))

	const ddth = (g * sin(theta) - ddx * cos(theta)) / l

	const th1 = theta + h * z
	const z1 = z + h * ddth

	
	const x1 = x + h * y
	const y1 = y + h * ddx

	theta = th1
	z = z1
	y = y1
	x = x1
	
	hdth.push(z)
	hddth.push(ddth)

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
})