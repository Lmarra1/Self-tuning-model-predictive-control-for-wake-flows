# Self-tuning Model Predictive Control for Wake Flows

## Overview

This repository contains MATLAB code for the application of model predictive control (MPC) with control parameters identified through Bayesian optimization. The control plant used is the chaotic wake of the fluidic pinball at Reynolds number Re = 150. The plant model is obtained using the SINDYc technique. Control is applied for drag reduction and lift stabilization, considering the challenge of measurement noise in control sensors. Noise mitigation is achieved using the Local Polynomial Regression technique.

## Key Features

- MPC with Bayesian optimization for parameter identification.
- SINDYc technique for plant model estimation.
- Control for drag reduction and lift stabilization for chaotic fluidic pinball wake
- Local Polynomial Regression for noise mitigation.


## Contact Information

- Author: Luigi Marra
- Email: lmarra@pa.uc3m.es


## Repository Information

- Repository URL: [https://github.com/Lmarra1/Self-tuning-model-predictive-control-for-wake-flows](https://github.com/Lmarra1/Self-tuning-model-predictive-control-for-wake-flows)


## Associated Research Paper

- Title: Self-tuned model predictive control for wake flow
- Authors: Luigi Marra, Andrea Meilán-Vila, Stefano Discetti
- Journal: Journal of Fluid Mechanics
- Year: 2024
- DOI: TBD


## Requirements

- MATLAB
- R (for execution of .R scripts)
- R package: lokern


## Instructions

The code is written in MATLAB.

For certain functionalities, R is required. You'll need to install the `lokern` package in R. Follow the steps below:

1. Install R: [Download R](https://www.r-project.org/)
2. Install RStudio (optional but recommended): [Download RStudio](https://www.rstudio.com/products/rstudio/download/)
3. Open R or RStudio.
4. Install the `lokern` package by running the following command:

    ---
    install.packages("lokern")
    ---

5. Identify the path to `Rscript.exe`.
6. Open the Matlab function 'SetUp.m' and paste the Rscript.exe path where specified
7. Run the MATLAB script 'SetUp.m'
8. Run the code 'MPC_main.m'
9. ...ENJOY!


