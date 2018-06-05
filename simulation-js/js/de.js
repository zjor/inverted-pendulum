const width = 400
const heigth = 400

const x = [1.0]
const y = [0.0]
let t = 0.0


function draw(ctx) {
	ctx.clearRect(0, 0, width, heigth)
	ctx.beginPath()
	integrate()
	const x0 = translate(x[0], y[0])
	ctx.moveTo(x0[0], x0[1])
	for (let i = 1; i <= x.length; i++) {
		const xi = translate(x[i], y[i])
		ctx.lineTo(xi[0], xi[1])
	}
	ctx.stroke()
}

function integrate() {
	if (t == 0.0) {
		t = performance.now()
	} else {
		const t1 = performance.now()
		const h = (t1 - t) / 1000.0
		const xi = x[x.length - 1]
		const yi = y[y.length - 1]

		const x1 = xi + h * yi
		const y1 = yi - h * xi

		const x2 = xi + h * (yi + y1) / 2
		const y2 = yi - h * (xi + x1) / 2

		x.push(x2)
		y.push(y2)

		// x.push(xi + h * yi)
		// y.push(yi - h * xi)
		t = t1
	}
}

function translate(x, y) {
	return [x * (width / 3) + width / 2, heigth / 2 - y * (heigth / 3)]
}

$(() => {
	let canvas = document.getElementById("e-canvas")
	let ctx = canvas.getContext("2d")
	let started = true

	const drawCycle = (ctx, timeout) => {
		draw(ctx)
		if (started) {
			setTimeout(() => { drawCycle(ctx, timeout) }, timeout)
		}
	}			

	$('#e-start').click(() => {
		started = true
		drawCycle(ctx, 50)
	})

	$('#e-stop').click(() => {
		started = false
	})

})

