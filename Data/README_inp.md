# Input File Creation

The input file is created as is using the following template:

``` text
Line 1: number of degrees of freedom of the system. 
Blank Line
Line 3: Target_1_impact_time, target_1_launch_time, target_1_initial_states, target_1_initial_covariance, target_1_t_0, target_1_u(t_0), target_1_t_1, target_1_u(t_1), ...target_1_t_m, target_1_u(t_m)
Line 4: Target_2_impact_time, target_2_launch_time, target_2_initial_states, target_2_initial_covariance, target_2_t_0, target_2_u(t_0), target_2_t_1, target_2_u(t_1), ...target_2_t_k, target_2_u(t_k)
Line 5:...
Line 6:...
.
.
.
Line 2+N: Target_N_impact_time, target_N_launch_time, target_N_initial_states, target_N_initial_covariance, target_N_t_0, target_N_u(t_0), target_N_t_1, target_N_u(t_1), ...target_N_t_j, target_N_u(t_j)
Blank Line
Line 3+N: Interceptor_1_launch_time, Interceptor_1_initial_states, Interceptor_1_initial_covariance, Interceptor_1_t_0, Interceptor_1_u(t_0)
Line 4+N: ...
.
.
.
Line 3+2N:Interceptor_N_launch_time, Interceptor_N_initial_states, Interceptor_N_initial_covariance, Interceptor_N_t_0, Interceptor_N_u(t_0)
Blank Line
Line 5+2N: Sensor noise covariance
Blank Line
Line 7+2N: Proportional navigation gain.
Blank Line
Line 9+2N: Step size for simulation.
Blank Line
Line 11+2N: Tolerance for stopping the simulation.
```

* `Line 1`: is slef explanatory. This refers to number of degrees of freedom (2D sim or 3D sim).
* `Line 3- Line 2+N`: **Initial Target States**. Specify the impact time, launch time/ first observed time, initial states and initial covariance. `target_i_t_0` is the time at which first control signal is applied to the target and `target_i_u(t_0)` is the control signal. Similarly `target_i_t_1` and `target_i_u(t_1)` are second control input time and signals, respectively. **The control input specification is optional**.
* `Line 3+N - 3+2N`: **Initial Interceptor States**. Specify the launch time, initial state, initial covariance, initial control input time and initial control input signal. Control input specification for interceptor is similar to that of the target. **The control input specification is optional**.
* `Line 5+2N`: **Sensor Noise Covariance**. The covariance of the radar noise. Typically fixed beforehand.
* `Line 7+2N`: **PN Gain**. Typically around 3-5. Must be tuned appropriately.
* `Line 9+2N`: **Simulation step size**. Step size for integration for both interceptor and target.
* `Line 11+2N`: **Tolerance**. Distance at which the simulation stops and target is deemed to be hit. Further the simulation also stops when it is observed that the interceptor is moving away from the target for 30 consecutive cycles.


## Example

Let us consider the input file given by:

```text
2

200.0, 0,  0.5,1.0,0.02,0.01,  0.01,0,0,0, 0,0.01,0,0, 0,0,0.001,0, 0,0,0,0.001, 0.0, 0.0,0.0
200.0, 0,  1.0,0.5,-0.02,0.01,   0.01,0,0,0, 0,0.01,0,0, 0,0,0.001,0, 0,0,0,0.001, 5.0, 1.0,0.0

0,  -0.5,-1.0,0.1,0.2,  1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1, 0.0, 0.0,0.0
0,  0.0,-0.5,0.1,0.2,  1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1, 0.0, 0.0,0.0

0,0,1,0, 0,0,0,1, 0,0,0,0, 0,0,0,0, 0.01,0,0,0, 0,0.01,0,0, 0,0,0.001,0, 0,0,0,0.001,  0,0, 0,0, 1,0, 0,1

0.01,0,0,0.01

3

0.01

0.001
```

Here we have

* dof = `2`. It is a 2-dimensional system.
* For target 1:
  * `t_f = 200`
  * `t_0 = 0`
  * `x(t_0) = [0.5,1.0,0.02,0.01]`
  * ```text
       P(t_0) = [0.01,0,0,0,
                  0,0.01,0,0,
                  0,0,0.001,0,
                  0,0,0,0.001]
      ```
  * `u(0) = [0,0]`
* For target 2:
  * `t_f = 200`
  * `t_0 = 0`
  * `x(t_0) = [1.0,0.5,-0.02,0.01]`
  * ```text
       P(t_0) = [0.01,0,0,0, 
                  0,0.01,0,0, 
                  0,0,0.001,0, 
                  0,0,0,0.001]
      ```
    * `u(5) = [1.0.0.0]`
* For interceptor 1:  
  * `t_0 = 0`
  * `x(t_0) = [-0.5,-1.0,0.1,0.2]`
  * ```text
       P(t_0) = [1,0,0,0,
                  0,1,0,0,
                  0,0,1,0,
                  0,0,0,1]
      ```
    * `u(0) = [0,0,0]`
* For interceptor 2:  
  * `t_0 = 0`
  * `x(t_0) = [0.0,-0.5,0.1,0.2]`
  * ```text
       P(t_0) = [1,0,0,0,
                  0,1,0,0,
                  0,0,1,0,
                  0,0,0,1]
      ```
    * `u(0) = [0,0,0]`
* Sensor noise
    ```text 
    R = [0.01,0,
        0,0.01]
    ```
* PN gain `N = 3`
* Step size `dt = 0.01`
* Tolerance `Tol = 0.01`
