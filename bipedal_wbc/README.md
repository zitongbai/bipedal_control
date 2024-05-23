# Bipedal WBC

bipedal_wbc is a ROS package implementing a whole-body controller for a bipedal robot.

# Theory
## Some notation
Generalized coordinates:
$$
\mathbf q = \begin{bmatrix} 
        \mathbf{q}_b \\
        \mathbf{q}_j
    \end{bmatrix}
$$

where $\mathbf{q}_b$ is the unactuated base coordinates and $\mathbf{q}_j$ is the actuated joint coordinates.

Generalized velocities:

$$
\mathbf v = \begin{bmatrix} 
        \mathbf{v}_b \\
        \mathbf{v}_j
    \end{bmatrix}
$$

where $\mathbf{v}_b\in \mathbb R^6$ is the unactuated base velocities and $\mathbf{v}_j\in \mathbb R^{n_j}$ is the actuated joint velocities. Please not that $\mathbf{v}_b$ is not equal to the time derivative of $\mathbf{q}_b$.

For MPC, the state is defined as:

$$
\mathbf x = \begin{bmatrix} 
        \mathbf{h}_{\text{com}} \\
        \mathbf{q}
    \end{bmatrix}
$$

where $\mathbf{h}_{\text{com}}\in \mathbb R^6$ is the collection of the normalized centroidal momentum and $\mathbf{q}$ is the generalized coordinates.

For MPC, the input is defined as:
$$
\mathbf u = \begin{bmatrix} 
        \mathbf{F}_c \\
        \mathbf{v}_j
    \end{bmatrix}
$$

where $\mathbf{F}_c$ is the contact forces and $\mathbf{v}_j$ is the joint velocities.

For WBC, the decision variables are defined as:

$$
\mathbf x = \begin{bmatrix} 
        \dot{\mathbf{v}} \\
        \mathbf{F}_c \\
        \mathbf{\tau}
    \end{bmatrix}
$$

where $\dot{\mathbf{v}}$ is the generalized acceleration, $\mathbf{F}_c$ is the contact forces and $\mathbf{\tau}$ is the joint torques.



## Tasks

A task T can be defined as a set of linear equality and/or inequality constraints on the solution vector $\mathbf x$:
$$
\mathbf T: \left\{ 
    \begin{aligned}
        \mathbf A \mathbf x = \mathbf b \\
        \mathbf D \mathbf x \leq \mathbf f
    \end{aligned}
    \right.

$$

### Equality constraints

#### 1. Equation of motion (EoM)
The equation of motion (EoM) $\mathbf M \dot{\mathbf v} + b + g + \mathbf{J}_c^\top \mathbf{F}_c = \mathbf{S}^\top \tau$ can be formulated as:
$$
\mathbf A = \begin{bmatrix}
        \hat{\mathbf M} & \hat{\mathbf J}_c^\top & -\mathbf S^\top 
    \end{bmatrix}
\quad
\mathbf b = -\hat{\mathbf b} - \hat{\mathbf g}
$$

#### 2. No motion at contact point
No motion at contact point. The linear and rotational acceleration of the end-effector $i$ (or any other point and link) is coupled to the generalized accelerations through the geometric Jacobians: $ \dot{\mathbf w}_i = \mathbf J_i \dot{\mathbf v} + \dot{\mathbf{J}}_i{\mathbf v} $, where $\mathbf{w}_i$ is the wrench (stack of positon and velocity) of end-effector $i$. For contact points $i$, the acceleration is zero, i.e., $\dot{\mathbf w}_i = 0$. This can be formulated as:
$$
\mathbf A = \begin{bmatrix}
        \hat{\mathbf{J}}_i & 0 & 0
    \end{bmatrix}
\quad
\mathbf b = -\hat{\dot{\mathbf J}}_i \mathbf v
$$

#### 3. Zero contact forces for no contact points
For no contact points, the contact forces $\mathbf F_{c, i}$ are zero. This can be formulated as:
$$
\mathbf I_3 \cdot \mathbf F_{c, i} = 0
$$
$\mathbf I_3$ is a 3x3 identity matrix and should lie in proper positions in a matrix $\mathbf A$. Matrix $\mathbf b = 0$

#### 4. Zero contact forces for contact points
For contact points, we also set the contact forces to zero to minimize the contact forces.

#### 5. Floating base tracking task
We have already planned a trajectory for the robot, including
* 0 order 
    * floating base position and orientation $\mathbf q_b = [\mathbf r_{\mathcal I \mathcal B}, \chi_{\mathcal I \mathcal B}]$
    * joint positions $\mathbf q_j$
* 1 order
    * centroidal momentum $\mathbf h_{\text{com}}$ 
    * joint velocities $\mathbf v_j$

However, in the decision variables of WBC, there are only 2 order variables. Consider the centroidal momentum matrix (CMM) $A(\mathbf q) \in \mathbb R^{6\times (6+n_j)}$ which maps the generalized velocities to the centroidal momentum: 
$$
\mathbf h_{\text{com}} = A(\mathbf q) \mathbf v = 
\begin{bmatrix}
A_b(\mathbf q) & A_j(\mathbf q)
\end{bmatrix}
\begin{bmatrix}
    \mathbf{v}_b \\
    \mathbf{v}_j
\end{bmatrix}
$$

<!-- This could be rearranged as:
$$
\mathbf v_b = A_b^{-1}(\mathbf q) 
\begin{bmatrix}
\mathbf h_{\text{com}} -  A_j(\mathbf q) \mathbf v_j
\end{bmatrix}
$$ -->

Taking the time derivative of the above equation, we get:
$$
\dot{\mathbf h}_{\text{com}} = \dot{A}(\mathbf q) \mathbf v + A(\mathbf q) \dot{\mathbf v}
$$

This could be rearranged as:
$$
\dot{\mathbf v} = A_b^{-1}\left(
    \dot{\mathbf h}_{\text{com}} - \dot{A}(\mathbf q) \mathbf v - \dot{A}_j(\mathbf q) \dot{\mathbf v}_j
\right)
$$

The above equation can be used to track the centroidal momentum. The tracking task can be formulated as:
$$
\mathbf A = \begin{bmatrix}
    \mathbf I_6 & 0 & 0 
\end{bmatrix}
\quad
\mathbf b =  A_b^{-1}\left(
    \dot{\mathbf h}_{\text{com}} - \dot{A}(\mathbf q) \mathbf v - \dot{A}_j(\mathbf q) \dot{\mathbf v}_j
\right)
$$

#### 6. Swing foot tracking task
We have already planned a trajectory for joints, including joint positions and velocities. Through forward kinematics, we can obtain the desired position and velocity of the swing foot (the stack of position and velocity is a wrench $\mathbf{w}_i$). They can be used to construct a PD controller, which gives a desired acceleration $\dot{\mathbf{w}}^*_i$ for the swing foot (contact point) $i$. Equation $ \dot{\mathbf w}_i = \mathbf J_i \dot{\mathbf v} + \dot{\mathbf{J}}_i{\mathbf v} $ (which is used in Equality Constraint 2) is used here to formulate the tracking task: 
$$
\mathbf A = \begin{bmatrix}
        \hat{\mathbf{J}}_i & 0 & 0
    \end{bmatrix}
\quad
\mathbf b = \dot{\mathbf{w}}^*_i -\hat{\dot{\mathbf J}}_i \mathbf v

$$

### Inequality constraints

#### 1. Joint Torque limits

Torque limits $\mathbf{\tau}_{\text{min}} \leq \mathbf{\tau} \leq \mathbf{\tau}_{\text{max}}$ can be formulated as:
$$
\mathbf D = \begin{bmatrix}
        0 & 0 & \mathbf I_{n_j \times n_j} \\
        0 & 0 & -\mathbf I_{n_j \times n_j}
    \end{bmatrix}
\quad
\mathbf f= \begin{bmatrix}
        \mathbf{\tau}_{\text{max}} \\
        -\mathbf{\tau}_{\text{min}}
    \end{bmatrix}
$$

#### 2. Friction cone

Contact forces $\mathbf{F}_c$ must be limited to lie within the friction cone. To obtain linear constraints, we approximate the friction cone with a pyramid. For each contact point $i$, the friction cone can be formulated as:
$$
\begin{bmatrix}
        0 & 0 & -1 \\
        1 & 0 & -\mu \\
        -1 & 0 & -\mu \\
        0 & 1 & -\mu \\
        0 & -1 & -\mu
    \end{bmatrix} 
    \begin{bmatrix}
        \mathbf F_{c, i}^x \\ 
        \mathbf F_{c, i}^y \\
        \mathbf F_{c, i}^z
    \end{bmatrix} \leq 0

$$
The above $5\times 3$ matrix should lie in proper positions in a matrix $\mathbf D$. Matrix $\mathbf f = 0$

**TODO**: Currently, the friction cone constraint assumes that the contact points are on the horizontal plane. This should be generalized to any plane. ($\mathbf J_c$ is computed in the `pinocchio::LOCAL_WORLD_ALIGNED` frame, which means that the contact forces are expressed in the same frame.)