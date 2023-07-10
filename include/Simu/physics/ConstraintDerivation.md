# Notation
 - The dot notation $\dot{f}$ denotes the derivative with respect to time $\dot{f} = \frac{df}{dt}$
 - Matrices will be marked in bold and vectors denoted by an arrow $\bold{A} = \begin{bmatrix}\vec{a_0} & \vec{a_1}\end{bmatrix}$
 - The identity matrix will be denoted $\bold{E}$ since $\bold{I}$ is used for the moment of inertia. Since we are in 2D, $I$ is a scalar.
 - The angular velocity may appear as a vector or a scalar: $\vec{\omega} = \left[\begin{smallmatrix}0 \\ 0 \\ \omega\end{smallmatrix}\right] = \omega\vec{k}$

# Constraints

Let $\vec{s} = \begin{bmatrix}\vec{x_0} \\ \theta_0 \\ ...\end{bmatrix}$ be the state vector of $n$ bodies where $\vec{x_0}$ is the centroid of the body at index $0$ and $\theta_0$ its orientation.

A constraint function $C(\vec{s})$ is a function that constrains any number of bodies. $C(\vec{s})$ may be vector valued: $\bold{C}: \mathbb{R}^{3n} \rightarrow \mathbb{R}^m$ where $n$ is the number of bodies and $m$ the dimension of the constraint.

The dimension of a constraint indicates how many degrees of freedom are removed from the system of bodies.

Simu works with sequential impulses and thus operates on velocity. Defining a new constraint requires finding the valid velocities such that the constraint stays satisfied.

The basic constraint $\bold{C}(\vec{s})$ is a constraint on positions. The velocity constraint is found as $\dot{\bold{C}}(\vec{s})$. Those velocity constraints can always be arranged as 
$$
\begin{align*}
    \dot{\bold{C}}(\vec{s}) &= \frac{\partial \bold{C}}{\partial \vec{s}} \cdot \frac{d\vec{s}}{dt}\\
    &= \bold{J}\vec{v}
\end{align*}
$$
Where $\bold{J}$ is the jacobian of $\bold{C}(\vec{s})$ and $\vec{v} = \begin{bmatrix}\vec{v_0} \\ \omega_0 \\ ...\end{bmatrix}$.

Constraints that are adding energy into the system (motors) will require a bias term:
$$\dot{\bold{C}}(\vec{s}) = \bold{J}\vec{v} + \vec{b}$$

# Constraint format
TODO:
- equality constraints
- inequality constraints

# Solving constraints

Consider a constraint of a single dimension acting on $n$ bodies, $C:\mathbb{R}^{3n} \rightarrow \mathbb{R}$

Intuitively, the constraint impulse $P_C$ should be in the direction that influences $C$ the most, that is, parallel to its gradient $\nabla C$. Let $\lambda$ be a scalar such that $P_C = \bold{J^T}\lambda$ where $\bold{J^T} = \nabla C$ for a one dimensionnal constraint.

// TODO: Newton's Law for impulses, units, etc.

We are searching for the impulse such that 
$$ \vec{v_f} = \vec{v_i} + \bold{M}^{-1}P_C$$
Where $\vec{v_f}$ is the final velocity which will satisfy $\dot{C} = \bold{J}\vec{v_f} = 0$.

By substitution,
$$
\begin{align*}
    \bold{J}\vec{v_f} &= 0\\
    \bold{J}(\vec{v_i} + \bold{M}^{-1}P_C) &= 0\\
    \bold{J}(\vec{v_i} + \bold{M}^{-1}\bold{J^T}\lambda) &= 0\\
    \bold{J}\bold{M}^{-1}\bold{J^T}\lambda &= -\bold{J}\vec{v_i}\\
    \lambda &= \frac{-\bold{J}\vec{v_i}}{\bold{J}\bold{M}^{-1}\bold{J^T}}
\end{align*}
$$

// TODO: Missing bias

# Constraint features
TODO:
Simu can automatically add features to constraints. The final equation to be solved is
$$
    \left(\bold{J}\bold{D}\bold{M}^{-1}\bold{J}^T+\Gamma\right)d\Lambda = -\left(\bold{J}\vec{v_i} + \frac{\Beta}{\Delta t}\bold{C}(\vec{s}) + \Gamma \Lambda\right)
$$
which is of the form $\bold{A}\vec{x} = \vec{b}$

TODO: where ...

## Position restitution
## Damping
## Dominance


# Examples