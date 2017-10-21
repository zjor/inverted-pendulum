# Free pendulum simulation
0; # hack to force script not function m-file

#{
Equation of motion: 
	Th'' = g/L * sin(Th)

ODE system:
	dx = y
	dy = g/L * sin(x)
#}

function xdot = f(x, t)

	L = 1.0;
	g = 9.8;

	xdot = zeros(2, 1);

	# dx
	xdot(1) = x(2);

	# dy
	xdot(2) = g / L * sin(x(1));

endfunction

t = linspace(0, 10, 100);
y = lsode("f", [pi/12; 0.0], t);

plot(t, y'(1, :), 'g-', t, y'(2, :), 'r-');
