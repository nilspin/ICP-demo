[//]: # (Sourced from : https://cse.buffalo.edu/~jryde/lectures/cse410/MobileRobotMapping_1.html)


# Conventional Notation [1]

* Pose - position + orientation, X
* Map, M
* Sensor data, Z
* Sensor model is \(p(Z|M, X)\)
* Canonical form of map building, map given set of sensor measurements and corresponding poses

$$ p(X_{1 \ldots t}, M | Z_{1 \ldots t}) $$
Recast by Bayes' theorem

$$ p(Z_{1 \ldots t} | X_{1 \ldots t}, M ) p(X_{1 \ldots t}, M) / p(Z_{1 \ldots t}) $$



For a rotation about the x-axis

Given the rotation matrices around the various axes.

$$ R_x(\theta) = \left( \begin{matrix} 1 & 0 & 0 \\ 0 & \cos \theta & -\sin \theta \\ 0 & \sin \theta & \cos \theta \\ \end{matrix} \right), R_y(\theta) = \left( \begin{matrix} \cos \theta & 0 & \sin \theta \\ 0 & 1 & 0 \\ -\sin \theta & 0 & \cos \theta \\ \end{matrix} \right), R_z(\theta) = \left( \begin{matrix} \cos \theta & -\sin \theta & 0 \\ \sin \theta & \cos \theta & 0\\ 0 & 0 & 1\\ \end{matrix} \right), $$
The full rotation matrix for *small* [^2] angles \(\alpha, \beta, \gamma\) about the \(x, y, z\) axes is therefore

$$ R = \left( \begin{matrix} 1 & -\gamma & \beta \\ \gamma & 1 & -\alpha \\ -\beta & \alpha & 1 \\ \end{matrix} \right) $$
The rotation matrix,

$$ R =\left(\begin{matrix}1 & - \gamma & \beta\\\gamma & 1 & - \alpha\\- \beta & \alpha & 1\end{matrix}\right) $$


The translation vector

$$ T =\left(\begin{matrix}t_{x}\\t_{y}\\t_{z}\end{matrix}\right) $$
The error function

$$ E = R P + T - Q =\left(\begin{matrix}p_{x} + t_{x} - q_{x} + \beta p_{z} - \gamma p_{y}\\p_{y} + t_{y} - q_{y} + \gamma p_{x} - \alpha p_{z}\\p_{z} + t_{z} - q_{z} + \alpha p_{y} - \beta p_{x}\end{matrix}\right) $$
$$ =\left(p_{x} + t_{x} - q_{x} + \beta p_{z} - \gamma p_{y}\right)^{2} + \left(p_{y} + t_{y} - q_{y} + \gamma p_{x} - \alpha p_{z}\right)^{2} + \left(p_{z} + t_{z} - q_{z} + \alpha p_{y} - \beta p_{x}\right)^{2} $$
To minimise E equate partial derivatives to zero.

$$ \delta E / \delta \alpha = p_{y} t_{z} + p_{z} q_{y} - p_{y} q_{z} - p_{z} t_{y} - \beta p_{x} p_{y} - \gamma p_{x} p_{z} + \alpha p_{y}^{2} + \alpha p_{z}^{2} = 0 $$
$$ \delta E / \delta \beta = p_{x} q_{z} + p_{z} t_{x} - p_{x} t_{z} - p_{z} q_{x} - \alpha p_{x} p_{y} - \gamma p_{y} p_{z} + \beta p_{x}^{2} + \beta p_{z}^{2} = 0 $$
$$ \delta E / \delta \gamma = p_{x} t_{y} + p_{y} q_{x} - p_{x} q_{y} - p_{y} t_{x} - \alpha p_{x} p_{z} - \beta p_{y} p_{z} + \gamma p_{x}^{2} + \gamma p_{y}^{2} = 0 $$
$$ \delta E / \delta t_x = p_{x} + t_{x} - q_{x} + \beta p_{z} - \gamma p_{y} = 0 $$
$$ \delta E / \delta t_y = p_{y} + t_{y} - q_{y} + \gamma p_{x} - \alpha p_{z} = 0 $$
$$ \delta E / \delta t_z = p_{z} + t_{z} - q_{z} + \alpha p_{y} - \beta p_{x} = 0 $$
Factor out the coefficients of the DOFs appropriately so it can be represented in linear form for solving.

$$ A x + B = 0 $$
Results in a covariance like matrix and linear matrix equation

$$ \left(\begin{matrix}p_{y}^{2} + p_{z}^{2} & - p_{x} p_{y} & - p_{x} p_{z} & 0 & - p_{z} & p_{y}\\- p_{x} p_{y} & p_{x}^{2} + p_{z}^{2} & - p_{y} p_{z} & p_{z} & 0 & - p_{x}\\- p_{x} p_{z} & - p_{y} p_{z} & p_{x}^{2} + p_{y}^{2} & - p_{y} & p_{x} & 0\\0 & p_{z} & - p_{y} & 1 & 0 & 0\\- p_{z} & 0 & p_{x} & 0 & 1 & 0\\p_{y} & - p_{x} & 0 & 0 & 0 & 1\end{matrix}\right) \left(\begin{matrix}\alpha\\\beta\\\gamma\\t_{x}\\t_{y}\\t_{z}\end{matrix}\right)+\left(\begin{matrix}p_{z} q_{y} - p_{y} q_{z}\\p_{x} q_{z} - p_{z} q_{x}\\p_{y} q_{x} - p_{x} q_{y}\\p_{x} - q_{x}\\p_{y} - q_{y}\\p_{z} - q_{z}\end{matrix}\right)= 0 $$

# References

[1]: https://cse.buffalo.edu/~jryde/lectures/cse410/MobileRobotMapping_1.html "Mobile Robot Mapping - Lecture 1
Julian Ryde"

# Footnotes
[^2]: This is linearization. For bigger angles we cannot approximate the non-linear function.


