const { sin, cos, sqrt, abs, PI } = Math
const agm = (x, y) => {
	let [a, g] = [x, y]
	while (Math.abs(a - g) > 0.001) {
		[a, g] = [0.5 * (a + g), Math.sqrt(a * g)]
	}
	return a
}
const getW = (angle, l, g) => {
	console.log(`angle = ${angle}; L = ${l}; g = ${g}`)
	return agm(1.0, Math.cos(angle / 2)) * Math.sqrt(g / l)
}

const width = 400
const heigth = 300
const boxWidth = 40
const boxHeight = 20
const boxY = heigth / 2

const m = 1.0
const l = 10
const g = 9.8
let x0 = 0.0

let x = 0.0
let theta = PI

// z = theta'
let z = 0.0
// y = x'
let y = 0.0

let t = 0.0
const h = 0.01

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

	integrate(h)
	t += h
	
}

const Kp = 100.0
const Kd = 50.0

const xKp = 2.0
const xKd = 3.5

function energy() {
	const p = l * (1 + cos(theta)) * m * g
	const k = m * (z * l) ** 2 / 2
	return {potential: p, kinetic: k, total: p + k}
}

function normTheta(th) {
	let nTh = th % (Math.PI * 2.0)
	return (nTh > Math.PI) ? nTh - 2 * Math.PI : nTh
}

let w = Math.sqrt(g / l)
const T0 = 2.0 * Math.PI / w
let T = T0
let A = 4.0
let isSwinging = false
let swingTime = 0.0

function control(th, dth, x, dx, t) {
	if (isSwinging) {
		if (swingTime > 3 * T / 4 && abs(dx) < 0.05) {
			isSwinging = false
			console.log("Swing stopped")			
		}
		swingTime += h
		return A * cos(w * swingTime)
	} else {
		if (abs(dth) < 0.01 && th >= PI) {
			isSwinging = true
			swingTime = 0.0
			w = getW(abs(PI - th), l, g)
			T = 2.0 * Math.PI / w
			A = 4.0 * T0 / T
			console.log("new W =", w, "new T =", T)
		}
		if (abs(normTheta(th)) < PI / 6) {
			return Kp * normTheta(th) + Kd * dth + xKp * (x - x0) + xKd * dx
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