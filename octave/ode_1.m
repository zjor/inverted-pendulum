# Single variable ODE solver example
0; # hack to force script not function m-file

function xdot = f(x, t)
	k = 1.0;
	xdot = k * x;
endfunction

t = linspace(0, 4, 20);
y = lsode("f", 1.0, t);

plot(t, y);
