import numpy as np
from numpy.linalg import matrix_rank
from control import ctrb, obsv

g = 9.8
L = 0.43	# pendulum length
d = 0.01	# friction coefficent

A = np.matrix([
	[.0,			1.,		.0,		.0],
	[- g / L,	- d / L,	.0,		.0],
	[.0,			.0,		.0,		1.],
	[.0,			.0,		.0, 	.0]
])

B = np.matrix([
	[.0,	1./L,	.0,		1.]
])

C = np.matrix([
	[1.0,	.0,		1.,		.0]
	])

ctrb_matrix = ctrb(A, np.transpose(B))
obsv_matrix = obsv(A, C)

print "Ctrb matrix rank: %d" % matrix_rank(ctrb_matrix)
print "Obsv matrix rank: %d" % matrix_rank(obsv_matrix)
