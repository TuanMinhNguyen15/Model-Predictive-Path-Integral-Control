# Model Predictive Path Integral (MPPI) Control

In a nutshell, MPPI is the stochastic version of Model Predictive Control (MPC). Unlike MPC where the optimal control sequence $u_0, u_1,..., u_{N-1}$ is determined by solving a convex optimization problem, the "optimial" control sequence of MPPI is found by calculating the weight sum of many randomly sampled control sequences in which the lower the objective cost resulted from a sampled control sequence, the larger the weight of that sampled control sequence will be. Because MPPI is a sample-based planner, the more samples that MPPI has, the better and more reliable its weighted-sum control sequence will be, and, therefore, we can use parallel computing such as multi-threading to simulstaneously sample thousands of control sequences. Moreover, because MPPI doesn't care if the system is linear or the problem is convex, MPPI provides us the flexibility to control nonlinear systems whose objective functions can be non-convex!

## MPPI Formulation

* State: $x \in  R^n$
* Input: $u \in R^m$
* Prediction Horizon: $N \in R^+$
* Trajectory: $X \equiv \\{x_0, x_1,..., x_N\\}$
* Control Sequence: $U \equiv \\{u_0, u_1,..., u_{N-1}\\}$
* Cost Function: $V(X,U) \in R$

Given the discretized system model:

  $$ x_{n+1} = f(x_n,u_n) $$
  
  , and suppose at every timestep we sample K control sequences which would result in K trajectories for a given initial state $x_0$. For each sampled control sequence U and its corresponding trajectory X, we can use the cost function V(X,U) to evaluate the system's behavior such that the lower the cost V, the better the sampled control sequence U at driving the system trajectory X to achieve some desirable objectives. 
  
 1st sample: $U^0 \equiv \\{u^0_0, u^0_1,..., u^0_{N-1}\\} \Rightarrow X^0 \equiv \\{x^0_0, x^0_1,..., x^0_N\\} \Rightarrow V^0 $
 
 2nd sample: $U^1 \equiv \\{u^1_0, u^1_1,..., u^1_{N-1}\\} \Rightarrow X^1 \equiv \\{x^1_0, x^1_1,..., x^1_N\\} \Rightarrow V^1 $
 
 ...
 
 Kth sample: $U^{K-1} \equiv \\{u_0^{K-1}, u_1^{K-1},..., u_{N-1}^{K-1}\\} \Rightarrow X^{K-1} \equiv \\{x_0^{K-1}, x_1^{K-1},..., x_N^{K-1}\\} \Rightarrow V^{K-1}$
 
 **Note:**  supperscript represents sample index, while subscript represents timestep index.
 
 Given an initial guess of or a previously obtained optimal control sequence $U^\* \equiv \\{ u_0^\*,u_1^\*,...,u_{N-1}^\* \\}   $, we can sample the K control sequences near $U^\*$ and the next optimal control sequence is calculated by the following weighted sum:
 
 $$  U_{new}^\* = U^\* + \frac{1}{2}$$
 
 
 
 
 
