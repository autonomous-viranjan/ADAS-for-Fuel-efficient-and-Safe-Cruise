# ADAS for Fuel-efficient and Safe Cruise

Hierarchical Controller for longitudinal drive assistance.

Nonlinear MPC (NLMPC) at higher level computes efficient acceleration input based on minimizing fuel consumption and maintaining safety distance requirements. Fuel consumption is modeled as a function of - velocity and throttle input - using data from (https://www.anl.gov/es/downloadable-dynamometer-database). The safety distance requirement increases with vehicle speed.

Reference tracker at lower level uses driver model based Predictive Control for driver assistance. It solves a convex optimization problem.

The NLMPC is solved using YALMIP MPC Toolbox (https://yalmip.github.io/) and the reference tracking convex optimization problem is solved using CVX solver (http://cvxr.com/cvx/).
