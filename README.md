# sim-f16
 F-16 nonlinear simulation with nonlinear aerodynamics. Lookup tables adapted from Lewis & Stevens, "Aircraft Control and Simulation."
 
`f16_dynamics.m` implements the dynamics of the F-16 in script form.

`f16_simulink.slx` implements the dynamics of `f16_dynamics.m` in Simulink for easier visualization of block diagram connections. A pilot model is also included here for closed-loop simulations. This is the primary simulation tool.

`f16_DAQ.m` is a script used to run batch simulations of the MATLAB script F-16 model.

`f16_simulink_DAQ.m` is a script used to run batch simulations of the Simulink F-16 model. (primary tool for data acquisition)

`f16_trimmer` is a helper function that calls `f16_dynamics.m` to trim the aircraft to steady flight at a user-specified condition (specify velocity and flight path angle). 

`f16_linearize` is a helper function that calls `f16_dynamics.m` to linearize the dynamics about a trim point.

"F-16 Lookups" directory includes lookup functions for aerodynamic and engine models.

"F-16 Data" directory includes information about the data that `f16_simulink_DAQ.m` is programmed to collect and save.


