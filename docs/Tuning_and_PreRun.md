# Tuning, Pre-Run & System Identification

## 1. System Identification (SysID)
The drone operates with standard BLDC motors modeled as first-order lag systems. Motor dynamics introduce a time delay constant $\tau$ to the generated thrust.

To guarantee accurate feedforward compensation in flight, the system runs an **Online Recursive Least Squares (RLS) Filter**. 
During the initial "Pre-Run" hovering phase (t = 0 to 2 seconds), the system continuously estimates the $\tau$ lag constant by comparing commanded thrust to actual measured acceleration. 

### RLS Equation
The RLS filter updates the parameter estimate $\theta[k]$ (which corresponds to $\tau$) using the current measurement $y[k]$ and regressors $\phi[k]$:

$$ K[k] = P[k-1] \phi[k] (\lambda + \phi^T[k] P[k-1] \phi[k])^{-1} $$
$$ \theta[k] = \theta[k-1] + K[k] (y[k] - \phi^T[k] \theta[k-1]) $$
$$ P[k] = \frac{1}{\lambda} (P[k-1] - K[k] \phi^T[k] P[k-1]) $$

The identified $\tau$ typically converges perfectly to $\sim 0.035 - 0.040s$ for the `racing_2207` motors.

## 2. Geometric Tuning Gains

Once $\tau$ is estimated, the SE(3) Adaptive Controller switches to full trajectory tracking. The following tuned gains are designed to provide highly robust flight envelopes, capable of suppressing up to 8.0 m/s crosswinds:

* **Translational PD (kp_position, kd_position):** `4.0` / `3.0`
* **Rotational PD (kp_rotation, kd_angular):** `1.5` / `0.3`
* **Velocity Drag Integral (kv_velocity):** `[3.0, 3.0, 4.0]`
* **Roll/Pitch Feedforward (k_rp):** `[6.0, 6.0, 0.0]`
* **Yaw Feedforward (k_yaw):** `1.5`
* **Angular Velocity Gain (k_omega):** `[0.35, 0.35, 0.3]`

These constants are located and modifiable in `config/quadrotor_sim.yaml`.

## 3. Pre-Run Benchmarks

The benchmark suite sweeps system robustness automatically, simulating extreme wind up to `8.0 m/s` across a mass gradient (from `0.77 kg` to `1.6 kg`). 
- **Nominal Payload (Under 1.0 kg):** Perfectly stable. Max steady-state tracking error stays bounded at `< 10.6 cm`.
- **Failure Threshold (Above 1.3 kg):** At roughly `1.3 kg` and higher winds, the motor thrust requirement saturates the 5.0 Newton maximum limit, causing the physics integrator to fail or the drone to drift uncontrollably.
