# Introduction to Robot Kinematics

As far as I understand kinematics, its a fundamental concept in robotics that models the movement of a **kinetic chain** (rigid bodies connected by joints). 

One can differentiate between **forward kinematics (FK)** and **inverse kinematics (IK)**:
- **Forward Kinematics (FK)**: Given the robot parameters compute the position and orientation of the **end-effector**.
- **Inverse Kinematics (IK)**: Given a desired position and orientation of the end-effector, compute the necessary joint parameters to achieve that pose.

Some key concepts I found important in kinematics:
---

## 1. Coordinate Frames

A **coordinate frame** is defined by:
- an origin
- axes that allow to uniquely describe positions of points on a manifold (e.g. n-dimensional Euclidean space $\mathbb{R}^n$)

In robotics, we often define multiple coordinate frames:
- **World frame**: fixed to the robot's base
- **Joint frames**: attached to each joint
- **Tool / End-effector frame**: attached to the robot's tool or hand

--- 
## 2. Transformations:

To transform points between coordinate frames, we need to be able to represent:
- **Rotations**: changing the orientation of a frame (e.g. SO(2) for 2D rotations, SO(3) for 3D rotations)
- **Translations**: changing the position of a frame
- **Scaling**: changing the size of objects

We can represent linear transformations by using a matrix operation of the form:
$$
y = Ax
$$
where $x \in \mathbb{R}^n$, $y \in \mathbb{R}^m$, $A \in \mathbb{R}^{m \times n}$ is the transformation matrix.

### 2.1 Homogeneous Transformations

Since in robotics, we are working with rigid bodies, we know that distances between points remain constant.
Therefore, every rigid body transformation is a combination of a rotation and a translation. Which has the form
$$
T(x) = R x + t
$$
where $R$ is a rotation matrix and $t$ is a translation vector.

Rigid transformations have the helpful property that they are invertible, with the inverse given by
$$
T^{-1}_{R, t} = T_{R^T, -R^T t}
$$

### 2.2 Transformation between Frames

Let a point have coordinates $p$ in a frame $F$.The origin of $F$ is at $t$ and its axes are given by $X= (x_1, x_2)$, $Y=(y_1, y_2)$ in 2D.
The coordinates of the point in the world frame are obtained by the rigid transformation
$$
p_W = Rp + t, \quad
R =
\begin{bmatrix}
x_1 & y_1 \\
x_2 & y_2
\end{bmatrix},
$$
where $p_W$ are the coordinates of the point in the world frame. This makes intueitive sense, as we first rotate the point (or frame $F$ axes) into the world frame orientation, and then correct for the translation $t$ of the frame origin.


### 2.3 Homogeneous coordinate representations
To conveniently represent both rotations and translations in a single matrix operation, we use **homogeneous coordinates**. A point $p \in \mathbb{R}^n $ is represented in homogeneous coordinates as $ \tilde{p} \in \mathbb{R}^{n+1} $ by appending an additional coordinate. This coordinate either has the value 1 (for positional quantities) or 0 (for directional quantities). This is done because translations should affect positional quantities (points) but not directional quantities (vectors).

E.g. in 2D:
A rigid transformation  $T(x) = Rx + t$ can be expressed as a single linear transformation in homogeneous coordinates. Defining $\hat{x} = (x_1, x_2, 1)^T$, the transformation becomes
$$
\hat{x}_W = \tilde{T}\hat{x}, \qquad
\tilde{T} =
\begin{bmatrix}
R & t \\
0 & 1
\end{bmatrix}.
$$
This representation allows rotation and translation in a single matrix multiplication.

### 2.4 Composition of Transforms

<img src="composition_of_transforms.png" alt="composition of transforms" width="500"/>

(source: https://www.user.tu-berlin.de/mtoussai/teaching/14-Robotics/02-kinematics.pdf)
Multiple transformations can be composed by matrix multiplication, but note that different joints have different degrees of freedom. 

---

## 3. Forward Kinematics (FK)

Forward kinematics is the process of calculating the frames of a robot's links, given a configuration and the robot's kinematic structure. We can then use these link frames to determine the end-effector pose in the world frame under arbitrary configurations & changes in joint parameters.


### 3.1 2D Forward Kinematics for Serial Robots

Consider an $n$-link serial robot with revolute joints. Let the reference transforms of the links be:

$$
T_{1,\text{ref}}, T_{2,\text{ref}}, \dots, T_{n,\text{ref}}
$$

and let the joint angles be $q_1, \dots, q_n$. The world coordinates of a point $x_i$ on link $i$ are obtained recursively as:

$$
T_1(q_1) = T_{1,\text{ref}} R(q_1)
$$

$$
T_i(q_1,\dots,q_i) = T_{i-1}(q_1,\dots,q_{i-1}) (T_{i-1,\text{ref}}^{-1} T_{i,\text{ref}}) R(q_i), \quad i>1
$$

Here $R(q_i)$ is the rotation due to joint $i$, and the parent-relative transform simplifies the recursion.  

For a planar $nR$ manipulator with link lengths $L_1,\dots,L_n$, the end-effector coordinates are:

$$
T_n(q) \begin{bmatrix} L_n \\ 0 \\ 1 \end{bmatrix} = 
\begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} + \sum_{i=1}^{n} \begin{bmatrix} L_i c_{1,\dots,i} \\ L_i s_{1,\dots,i} \\ 0 \end{bmatrix}
$$

where $c_{1,\dots,i} = \cos\sum_{k=1}^i q_k$ and $s_{1,\dots,i} = \sin\sum_{k=1}^i q_k$.


### Branched and Prismatic Joints

For branched robots, the recursion uses the parent index $p[i]$:

$$
T_i(q) = 
\begin{cases} 
T_{p[i]}(q) T_{p[i],\text{ref}}^{i,\text{ref}} R(q_i), & p[i]\neq W \\
T_{i,\text{ref}} R(q_i), & p[i]=W
\end{cases}
$$

For prismatic joints along axis $a_i$, replace $R(q_i)$ with the translation matrix:

$$
P(q_i) = \begin{bmatrix} 1 & 0 & a_{i,x} q_i \\ 0 & 1 & a_{i,y} q_i \\ 0 & 0 & 1 \end{bmatrix}
$$

### 3D Forward Kinematics

In 3D, homogeneous matrices are $4\times4$, and each joint is defined by an axis $a_i = (a_{i,x},a_{i,y},a_{i,z})$.  

- Revolute joints: use axis-angle rotation $L_{1,a}(q_i)$.  
- Prismatic joints: use translation along the axis $L_{0,a}(q_i)$.

The general 3D forward kinematics recursion remains:

$$
T_i(q) = T_{p[i]}(q) T_{p[i],\text{ref}}^{i,\text{ref}} L_{z_i,a_i}(q_i)
$$

where $z_i$ indicates revolute $(1)$ or prismatic $(0)$.


## 4. Inverse Kinematics (IK)