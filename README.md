# MWE Dynamical Optimization
This repository contains a minimal working example (MWE) of how to solve a dynamic optimization problem in Matlab using a single-shooting approach implemented with fmincon and ode45.

The example system is

dx/dt = -x + u,

and the objective is to minimize

int_t0^tf (y - yref)^2 dt,

where y = x.

The problem only contains one decision variable, which is the constant value of the manipulated input to the system, u, i.e., it does not contain multiple control intervals as is common in optimal control problems.
