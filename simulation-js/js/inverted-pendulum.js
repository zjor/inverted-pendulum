const sin = Math.sin
const cos = Math.cos

const width = 400
const heigth = 400
const boxWidth = 40
const boxHeight = 20
const boxY = heigth / 2

const l = 20
const g = 9.8

// b = m / (M + m)
const b = 0.99

let x = width / 2
let theta = Math.PI / 16

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

	ctx.rect(x - boxWidth / 2, boxY - boxHeight / 2, boxWidth, boxHeight)
	
	const scaledL = l * 7
	const rodX = scaledL * Math.sin(theta) + x
	const rodY = boxY - scaledL * Math.cos(theta)
	ctx.moveTo(x, boxY)
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

function integrate(h) {
	let th1 = theta + h * z
	let z1 = z + h * g / l * sin(theta)
	let th2 = theta + h / 2 * (z + z1)
	let z2 = z + h / 2 * g / l * (sin(theta) + sin(th1))

	let x1 = x + h * y
	let y1 = y - h * b * sin(theta) * (g * cos(theta) - l * z * z)
	let x2 = x + h / 2 * (y + y1)
	let y2 = y - h / 2 * b * (sin(theta) * (g * cos(theta) - l * z * z) + sin(th1) * (g * cos(th1) - l * z * z))

	theta = th2
	z = z2
	x = x2
	y = y2
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