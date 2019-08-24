# DC-motor simulations

## Equations of motion

### Electrical equation

![U = L\frac{di}{dt} + Ri + kw](https://latex.codecogs.com/gif.latex?U%20%3D%20L%5Cfrac%7Bdi%7D%7Bdt%7D%20&plus;%20Ri%20&plus;%20kw)

where 
- U - input voltage
- L - rotor inductance
- R - rotor coils resistance
- kw - back EMF term proportional to w

### Mechanical equation

![J\frac{dw}{dt} + Bw = T_M - T_L = ki - mr^2\frac{dw}{dt}](https://latex.codecogs.com/gif.latex?J%5Cfrac%7Bdw%7D%7Bdt%7D%20&plus;%20Bw%20%3D%20T_M%20-%20T_L%20%3D%20ki%20-%20mr%5E2%5Cfrac%7Bdw%7D%7Bdt%7D)

where
- J - momentum of inertia of the rotor
- B - friction due to rotation
- T<sub>M</sub> - torque of the motor
- T<sub>L</sub> - torque of the load
- m - mass of the load
- r - radius of the shaft

Note that the friction due to the movement of the mass is neglected.

### Assumptions

![\frac{L}{R} << \frac{J}{B}, => i = \frac{U}{R} - \frac{k}{Rw}](https://latex.codecogs.com/gif.latex?%5Cfrac%7BL%7D%7BR%7D%20%3C%3C%20%5Cfrac%7BJ%7D%7BB%7D%2C%20%3D%3E%20i%20%3D%20%5Cfrac%7BU%7D%7BR%7D%20-%20%5Cfrac%7Bk%7D%7BRw%7D)

thus the DC-motor can be described with one equation

![\frac{dw}{dt}(J + mr^2) + w(B + \frac{k^2}{R}) = \frac{kU}{R}](https://latex.codecogs.com/gif.latex?%5Cfrac%7Bdw%7D%7Bdt%7D%28J%20&plus;%20mr%5E2%29%20&plus;%20w%28B%20&plus;%20%5Cfrac%7Bk%5E2%7D%7BR%7D%29%20%3D%20%5Cfrac%7BkU%7D%7BR%7D)

## The system to control

In this example I want to position a cart pulled by the motor at certain location, so that the system transitions from the state [x1, w1 (typically w1 = 0)] to [0, 0]

### Equations of the system

![\begin{cases}\dot{x} = wr\\\dot{w}=-\alpha w + \beta U\end{cases}](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bcases%7D%5Cdot%7Bx%7D%20%3D%20wr%5C%5C%5Cdot%7Bw%7D%3D-%5Calpha%20w%20&plus;%20%5Cbeta%20U%5Cend%7Bcases%7D)

where

![\alpha = \frac{B + k^2/R}{J + mr^2}](https://latex.codecogs.com/gif.latex?%5Calpha%20%3D%20%5Cfrac%7BB%20&plus;%20k%5E2/R%7D%7BJ%20&plus;%20mr%5E2%7D) and 
![\beta = \frac{k}{R(J + mr^2)}](https://latex.codecogs.com/gif.latex?%5Cbeta%20%3D%20%5Cfrac%7Bk%7D%7BR%28J%20&plus;%20mr%5E2%29%7D)

### Controllability

![A=\begin{bmatrix}0 & r\\ 0 & -\alpha\end{bmatrix}](https://latex.codecogs.com/gif.latex?A%3D%5Cbegin%7Bbmatrix%7D0%20%26%20r%5C%5C%200%20%26%20-%5Calpha%5Cend%7Bbmatrix%7D)

![B=\begin{bmatrix}0\\ \beta\end{bmatrix}](https://latex.codecogs.com/gif.latex?B%3D%5Cbegin%7Bbmatrix%7D0%5C%5C%20%5Cbeta%5Cend%7Bbmatrix%7D)

Controllability matrix 
![\begin{bmatrix}B && AB\end{bmatrix} = \begin{bmatrix}0 && r\beta\\\beta && -\alpha\beta\end{bmatrix}](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7DB%20%26%26%20AB%5Cend%7Bbmatrix%7D%20%3D%20%5Cbegin%7Bbmatrix%7D0%20%26%26%20r%5Cbeta%5C%5C%5Cbeta%20%26%26%20-%5Calpha%5Cbeta%5Cend%7Bbmatrix%7D)
, rank equals 2 thus the system is controllable

## Step response
![Step Response](step_response.png)

## PID controller
![PID results](pid.png)

## LQR controller
![LQR results](lqr.png)

## TODO

- find alpha, beta parameters for the real motor based in step response
- take into account load friction and stall torque
