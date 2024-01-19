# Self-tuning Model Predictive Control for Wake Flows

## Overview

This repository contains MATLAB code for the application of model predictive control (MPC) with control parameters not manually selected but identified through a Bayesian optimization method. The selected control plant is the chaotic wake of the fluidic pinball at Reynolds number Re_D = 150. A plant model is obtained using the SINDYc technique. Control is applied for drag reduction and lift stabilization, with the challenge posed by the presence of measurement noise in the control sensors. The effect of noise is mitigated using the Local Polynomial Regression technique.

Unlike the associated research paper, the plant here also corresponds with the plant model obtained with the SINDYc technique.


## Key Features

- MPC with Bayesian optimization for parameter identification.
- SINDYc technique for plant model estimation.
- Control for drag reduction and lift stabilization for chaotic fluidic pinball wake
- Local Polynomial Regression for noise mitigation.


## Contact Information

- Author: Luigi Marra
- Email: lmarra@pa.uc3m.es


## Repository Information

- Repository URL: https://github.com/Lmarra1/Self-tuning-model-predictive-control-for-wake-flows


## Associated Research Paper

- Title: Self-tuned model predictive control for wake flow
- Authors: Luigi Marra, Andrea Meilán-Vila, Stefano Discetti
- Journal: Journal of Fluid Mechanics
- Year: 2024
- DOI: 10.1017/jfm.2024.47


## Requirements

- MATLAB
- R (for execution of .R scripts)
- R package: lokern


## Instructions

The code is written in MATLAB.

For certain functionalities, R is required.

1. Install R: [Download R](https://www.r-project.org/)
2. Install RStudio (optional but recommended): [Download RStudio](https://www.rstudio.com/products/rstudio/download/)
3. Identify the path to `Rscript.exe`.
4. Open the Matlab function 'SetUp.m' and paste the Rscript.exe path where specified
5. Run the MATLAB script 'SetUp.m'
6. Run the code 'MPC_main.m'
7. ...ENJOY!


Part of the code was implemented based on the MPC code of the article:
Kaiser, E., Kutz, J. N., & Brunton, S. L. (2018). Sparse identification of nonlinear 
dynamics for model predictive control in the low-data limit. 
Proceedings of the Royal Society A, 474(2219), 20180335.


--------------------
If the installation of the 'lokern' package fails, you can manually run the following command in R or Rstudio:

    ---
    install.packages("lokern")
    ---



## Funding 
The authors acknowledge the support from the research project PREDATOR-CM-UC3M. This project has been funded by the call "Estímulo a la Investigación de Jóvenes Doctores/as" within the frame of the Convenio Plurianual CM-UC3M and the V PRICIT (V Regional Plan for Scientific Research and Technological Innovation).


