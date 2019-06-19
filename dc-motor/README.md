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
- Tm - torque of the motor
- Tl - torque of the load

### Assumptions

![\frac{L}{R} << \frac{J}{B}, => i = \frac{U}{R} - \frac{k}{Rw}](https://latex.codecogs.com/gif.latex?%5Cfrac%7BL%7D%7BR%7D%20%3C%3C%20%5Cfrac%7BJ%7D%7BB%7D%2C%20%3D%3E%20i%20%3D%20%5Cfrac%7BU%7D%7BR%7D%20-%20%5Cfrac%7Bk%7D%7BRw%7D)

thus the DC-motor can be described with one equation

![\frac{dw}{dt}(J + mr^2) + w(B + \frac{k^2}{R}) = \frac{kU}{R}](https://latex.codecogs.com/gif.latex?%5Cfrac%7Bdw%7D%7Bdt%7D%28J%20&plus;%20mr%5E2%29%20&plus;%20w%28B%20&plus;%20%5Cfrac%7Bk%5E2%7D%7BR%7D%29%20%3D%20%5Cfrac%7BkU%7D%7BR%7D)

## Step response
![Step Response](step_response.png)
