# PID Controller: How to use

## Step 1: Importing

```python
from pidcontroller import PID
```

You have now imported the PID Controller

## Step 2: Make a system
We will use a first order process system to simulate a real life application
```python
class Process:
    def __init__(self, k=1, tau=1):
        self.k = k
        self.tau = tau
        self.y = 0
    def update(self, u, dt):
        self.y += (self.k * (u - self.y) / self.tau) * dt
        return self.y
process = Process()
```
It is not the most complicated but it will work

## Step 3: Defining constants
Start by making the pid controller. First, define the set point and plug it in to a new PID object
```python
set_point = 100
pid = PID(set_point)
```
This package supports two types of the PID controller equation.

The parallel form,

$$u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de}{dt}$$

where...  
$K_p$, $K_i$, and $K_d$ are all constants,  
and $K_pe(t)$ is the proportional action,  
$K_i \int e(t) dt$ is the integral action,  
and $K_d \frac{de}{dt}$ is the derivative action

and the ideal form,

$$u(t) = K_p \bigg(e(t) + \frac{1}{\tau_i} \int e(t) dt + \tau_d \frac{de(t)}{dt}\bigg)$$

where...  
$K_p$ is the proportional gain,  
$\tau_i$ and $\tau_d$ are integral and derivative time constants respectively  
$K_pe(t)$ is the proportional action,  
$K_i \int e(t) dt$ is the integral action,  
and $K_d \frac{de}{dt}$ is the derivative action

$t$ is the time and $e(t)$ and $u(t)$ are the error and output at time $t$ respectively in both

You can input the parallel constants with
```python
pid.parallel_constants = kp, ki, kd
```
and the ideal constants with
```python
pid.ideal_constants = kp, ti, td
```

## Step 4: Using the Controller
You can use a simple ```for``` loop to use the controller
```python
import time
INTERVAL = 0.1
y = process.update(0, INTERVAL)
for _ in range(150):  # Or the amount of times you want to use the controller
    time.sleep(INTERVAL)
    u = pid.compute(y, INTERVAL)  # Compute the value
    process.update(u, INTERVAL)  # Use the value to update system
```

This will be sufficient to make sure the value reaches the set point

You can also turn on derivative filtering with the filter time constant and filter flag parameters
```python
pid = PID(set_point, tf=filter_time_constant)
...
pid.compute(y, INTERVAL, filter_derivative=True)
```

or anti-windup with the minimum and maximum bounds for the output and the anti-windup flag parameters
```python
pid = PID(set_point, u_min=minimum, u_max=maximum)
...
pid.compute(y, INTERVAL, anti_windup=True)
```

um thats it hope u liked