# Formula for Calculating Shooting Parameters

To calculate the required **Robot Facing (Yaw)**, **Shooter Pitch**, and **Shooter Velocity** to hit a target while moving, minimizing flight time, we use 3D projectile kinematics.

## 1. Variables and Inputs

*   $\mathbf{P}_s$: Shooter Position (from `shooterPose`).
*   $\mathbf{P}_t$: Target Position (from `Target`).
*   $\mathbf{V}_r$: Robot Velocity Vector (field-relative, derived from `currentSpeeds`).
*   $\mathbf{g}$: Gravity vector ($0, 0, -9.81 m/s^2$).
*   $v_{max}$: Maximum launch velocity of the Note/Ball.
*   $\theta_{max}$: Maximum pitch angle.
*   $h_{max}$: Maximum allowed trajectory height.

## 2. Kinematic Equation

The position of the projectile at time $t$ is given by:
$$ \mathbf{P}(t) = \mathbf{P}_s + \mathbf{V}_{total} t + \frac{1}{2} \mathbf{g} t^2 $$

Where $\mathbf{V}_{total}$ is the sum of the robot's velocity and the shot velocity (relative to the robot):
$$ \mathbf{V}_{total} = \mathbf{V}_r + \mathbf{V}_{shot} $$

Substituting $\mathbf{V}_{total}$:
$$ \mathbf{P}(t) = \mathbf{P}_s + (\mathbf{V}_r + \mathbf{V}_{shot}) t + \frac{1}{2} \mathbf{g} t^2 $$

We want the projectile to be at the target $\mathbf{P}_t$ at flight time $t$:
$$ \mathbf{P}_t = \mathbf{P}_s + \mathbf{V}_r t + \mathbf{V}_{shot} t + \frac{1}{2} \mathbf{g} t^2 $$

## 3. Solving for Shot Velocity

Rearranging the equation to solve for the required shot velocity vector $\mathbf{V}_{shot}$ for a given time $t$:

$$ \mathbf{V}_{shot} t = \mathbf{P}_t - \mathbf{P}_s - \mathbf{V}_r t - \frac{1}{2} \mathbf{g} t^2 $$

$$ \mathbf{V}_{shot}(t) = \frac{\mathbf{P}_t - \mathbf{P}_s - \frac{1}{2} \mathbf{g} t^2}{t} - \mathbf{V}_r $$

Let $\mathbf{\Delta P} = \mathbf{P}_t - \mathbf{P}_s$ be the displacement vector to the target.

$$ \mathbf{V}_{shot}(t) = \frac{\mathbf{\Delta P}}{t} - \frac{1}{2} \mathbf{g} t - \mathbf{V}_r $$

## 4. Decomposing the Shot Vector

Once we have $\mathbf{V}_{shot}(t) = (v_x, v_y, v_z)$, we can extract the shooting parameters:

1.  **Shooter Velocity (Magnitude):**
    $$ v_{shot} = \sqrt{v_x^2 + v_y^2 + v_z^2} $$

2.  **Robot Facing (Yaw):**
    $$ \psi = \text{atan2}(v_y, v_x) $$
    This is the field-relative angle the robot must face (assuming a fixed shooter).

3.  **Shooter Pitch:**
    $$ \theta = \text{atan2}(v_z, \sqrt{v_x^2 + v_y^2}) $$

## 5. Optimization (Shortest Flight Time)

To find the shortest flight time $t$, we iterate through possible values of $t$ (starting from a small lower bound) and check if the resulting $\mathbf{V}_{shot}(t)$ satisfies the system constraints:

1.  **Velocity Constraint:** $v_{shot} 
le v_{max}$
2.  **Angle Constraint:** $\theta 
le \theta_{max}$
3.  **Height Constraint:**
    The peak height of the trajectory (relative to the field) is:
    $$ h_{peak} = P_{s,z} + \frac{(V_{total, z})^2}{2|g|} $$
    Constraint: $h_{peak} 
le h_{max}$

We select the smallest $t$ that satisfies all conditions.

## 6. Algorithm Summary

1.  Convert `currentSpeeds` (robot-relative) to $\mathbf{V}_r$ (field-relative) using the robot's current rotation.
2.  Iterate $t$ from $t_{min}$ to $t_{max}$ in small steps.
3.  Calculate $\mathbf{V}_{shot}(t)$.
4.  Calculate $v_{shot}, \theta, \psi$.
5.  Check if $v_{shot} 
le v_{max}$, $\theta 
le \theta_{max}$, and trajectory height $
le h_{max}$.
6.  If valid, return the RobotState $(\psi, v_{shot}, \theta)$.
