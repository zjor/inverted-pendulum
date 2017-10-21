# Free pendulum simulation
0; # hack to force script not function m-file

#{
Equation of motion: 
	Th'' = g/L * sin(Th) - x''/L * cos(Th)

ODE system:
	u = Kp * th + Kd * dth

	dth = y
	dy = (g * sin(th) - u * cos(th)) / L
	dv = u
	dx = v
#}

function xdot = f(x, t)

	L = 1.0;
	g = 9.8;
	Kp = 25.0;
	Kd = 5.0;

	xdot = zeros(2, 1);

	# dth
	xdot(1) = x(2);

	u = Kp * x(1) + Kd * x(2);

	# dy aka d2th
	xdot(2) = (g * sin(x(1)) - u * cos(x(1))) / L; 

endfunction

t = linspace(0, 10, 4000);
y = lsode("f", [pi/4; 0.0], t);

plot(t, y'(1, :), 'g-', t, y'(2, :), 'r-');
