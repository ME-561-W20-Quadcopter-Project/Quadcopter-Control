# A Comparison of Quadcopter Drone Control Methods <!-- omit in toc -->

## Table of Contents <!-- omit in toc -->
- [Contributors](#contributors)
- [Documents](#documents)
- [Source Code](#source-code)

## Contributors

### Project Team <!-- omit in toc -->

- Sravan Balaji ([balajsra@umich.edu](mailto:balajsra@umich.edu))
- Aditya Iyer ([adiyer@umich.edu](mailto:adiyer@umich.edu))
- Lakshmanan Periakaruppan ([lperiaka@umich.edu](mailto:lperiaka@umich.edu))
- Naman Shah ([namanvs@umich.edu](mailto:namanvs@umich.edu))
- Sumedh Vaishampayan ([sumi@umich.edu](mailto:sumi@umich.edu))

### EECS / MECHENG 561 W20 Course Staff <!-- omit in toc -->

- Ram Vasudevan ([ramv@umich.edu](mailto:ramv@umich.edu))
- Sid Dey ([siddey@umich.edu](mailto:siddey@umich.edu))

## Documents

1. [Project Proposal](1.%20ME%20561%20Project%20Proposal.pdf)
2. [Final Report - Overleaf (Read-Only)](https://www.overleaf.com/read/kyjvdsxkfnmg)

## Source Code

### [LQR.m](https://github.com/ME-561-W20-Quadcopter-Project/Quadcopter-Control/blob/master/src/LQR.m) <!-- omit in toc -->

Finite and infinite time horizon LQR implementation in MATLAB. Gains determined for linearized discrete time system, then simulated on nonlinear system (see LQRNonlinearSim.slx below).

### [LQRNonlinearSim.slx](https://github.com/ME-561-W20-Quadcopter-Project/Quadcopter-Control/blob/master/src/LQRNonlinearSim.slx) <!-- omit in toc -->

Simulink nonlinear model that is run from within LQR.m. Takes gain matrix **K** and initial condition **x_0** as input.

### [PlantModel.m](https://github.com/ME-561-W20-Quadcopter-Project/Quadcopter-Control/blob/master/src/PlantModel.m) <!-- omit in toc -->

Parameters for PID controller (see PlantModelSim.slx below).

### [PlantModelSim.slx](https://github.com/ME-561-W20-Quadcopter-Project/Quadcopter-Control/blob/master/src/PlantModelSim.slx) <!-- omit in toc -->

PID control of nonlinear and linear system.
