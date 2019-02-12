import matplotlib.pyplot as plt

'''
h(t) = 1/S I[Q(t) - q(t)]dt + h0
Q(t) - control
q(t) - load
The goal is to keep h(t) ~ h0
'''

h0 = 10.0
Q0 = 5.0
q0 = 5.0
dt = 0.1
S = 10.0


h = [h0]
Q = [Q0]
q = [q0]

K = 0.5

N = 500

def gen_q(i):
	if i == 2:
		return q0 * 2
	else:
		return q0

for i in range(0, N):
	h1 = h[i] + 1 / S * (Q[i] - q[i]) * dt
	Q1 = Q[i] - K * (h1 - h0)
	q1 = gen_q(i)

	h.append(h1)
	Q.append(Q1)
	q.append(q1)


line_h, = plt.plot(range(0, N + 1), h, label = 'Level')
line_Q, = plt.plot(range(0, N + 1), Q, label = 'Pump')
line_q, = plt.plot(range(0, N + 1), q, label = 'Consumption')

plt.legend([line_h, line_Q, line_q])

plt.show()	



