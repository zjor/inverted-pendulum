# Single variable ODE solver example
0; # hack to force script not function m-file

function xdot = f(x, t)
	xdot = zeros(2, 1);
	xdot(1) = x(2);
	xdot(2) = -x(1);
endfunction

t = linspace(0, 6000, 100000);
y = lsode("f", [0.0; 1.0], t);

plot(y'(1, :), y'(2, :));
