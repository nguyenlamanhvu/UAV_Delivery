# Specification: Realistic Wind Turbulence for Drake Drone Simulation

## 1. Overview
This specification details the implementation of a highly realistic wind turbulence feature for a drone simulation in Drake. To support academic research and ensure maximum physical realism, the system will use the **Dryden Wind Turbulence Model** to generate stochastic gusts, which will then be applied as an **External Aerodynamic Spatial Force** directly onto the drone's `MultibodyPlant`.

## 2. System Architecture
The feature will be implemented as a custom Drake `LeafSystem` (e.g., `DrydenWindForceSystem`) inserted between a white noise generator and the drone's physics engine.

### Data Flow Pipeline:
1. **Noise Generation:** 3 independent `drake::systems::RandomSource` components generate Gaussian white noise for the X, Y, and Z axes.
2. **Shaping Filters (Dryden Model):** The `DrydenWindForceSystem` applies linear transfer functions (shaping filters) to the white noise to color it according to the Dryden spatial/temporal spectra.
3. **Mean Wind Addition:** The filtered stochastic gusts are added to a constant mean wind vector (defining baseline wind direction and strength).
4. **Aerodynamic Drag Calculation:** The total wind velocity is compared against the drone's current velocity (retrieved from the `MultibodyPlant` state) to compute the relative airspeed.
5. **Force Injection:** Aerodynamic drag equations convert the relative airspeed into a physical force vector, outputting a `SpatialForce` into the `MultibodyPlant`'s `applied_spatial_force_input_port`.

## 3. Mathematical Model (Dryden)
The implementation must adhere to the continuous Dryden velocity spectra as defined in **MIL-F-8785C** or **MIL-HDBK-1797B**.

*   **Inputs:** 
    *   White noise signals $n_u, n_v, n_w$.
    *   Airspeed ($V$).
    *   Altitude ($h$).
*   **Parameters:** 
    *   Scale lengths ($L_u, L_v, L_w$), which are a function of altitude. For low-altitude drone flight (boundary layer), $L_w \approx h$.
    *   Turbulence intensities ($\sigma_u, \sigma_v, \sigma_w$), which dictate the severity of the storm/gusts.
*   **Transfer Functions:** The white noise must be passed through the specific $H_u(s)$, $H_v(s)$, and $H_w(s)$ transfer functions (or their discrete state-space equivalents via Tustin transform) to generate the turbulent velocity components $u_g, v_g, w_g$.

## 4. Implementation Details for the Coding Agent
*   **Language:** C++ (or Python bindings if prototyping).
*   **Drake Classes Used:** `drake::systems::LeafSystem`, `drake::systems::RandomSource`, `drake::multibody::MultibodyPlant`.
*   **State Space:** The `LeafSystem` will require internal continuous or discrete state to process the transfer functions over time.

## 5. References and Resources
The coding agent should refer to the following resources for exact mathematical formulations and C++ implementation examples:

1.  **Mathematical Foundation:** [MIL-F-8785C (Military Specification - Flying Qualities of Piloted Airplanes)](http://everyspec.com/MIL-SPECS/MIL-SPECS-MIL-F/MIL-F-8785C_12231/)
2.  **C++ Implementation Reference:** [goromal/wind-dynamics on GitHub](https://github.com/goromal/wind-dynamics) - Provides a C++ class implementation of the Dryden model in pseudospectral form.
3.  **Simulation Framework Reference:** [JSBSim Open Source Flight Dynamics Engine](https://github.com/JSBSim-Team/jsbsim) - See the `FGWinds` class for a highly validated C++ implementation of the Dryden model.
4.  **MathWorks Documentation:** [Dryden Wind Turbulence Model (Continuous)](https://www.mathworks.com/help/aeroblks/drydenwindturbulencemodelcontinuous.html) - Useful for verifying the transfer function equations.
