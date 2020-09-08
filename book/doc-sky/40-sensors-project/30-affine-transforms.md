# Affine Transformations {#sensors-assignment-affine status=draft}

## Background Information

In order to estimate the PiDrone's position (a 2-dimensional column vector $v = [x \; y]^T$) using the camera, you will need to use affine transformations. An affine transformation $f: \mathbb{R}^n \to \mathbb{R}^m$ is any transformation of the form $v \to Av + b$, where $A \in \mathbb{R}^{m \times n}$ and $b \in \mathbb{R}^m$. The affine transformations we are interested in are \emph{rotation}, \emph{scale}, and \emph{translation} in two dimensions. So, the affine transformations we will look at will map vectors in $\mathbb{R}^2$ to other vectors in $\mathbb{R}^2$.

Let's first look at rotation. We can rotate a column vector $v \in \mathbb{R}^2$ about the origin by the angle $\theta$ by premultiplying it by the following matrix:

\[
  \begin{bmatrix}
  \cos \theta & -\sin \theta \\
  \sin \theta & \cos \theta \\
    \end{bmatrix}
\]

Let's look at an example. Below we have the vector $[1, 2]^T$. To rotate the vector $\frac{2\pi}{3}$, we premultiply the vector by the rotation matrix:

\[
  \begin{bmatrix}
  \cos \frac{2\pi}{3} & -\sin \frac{2\pi}{3} \\
  \sin \frac{2\pi}{3} & \cos \frac{2\pi}{3} \\
    \end{bmatrix}
    \begin{bmatrix}
    1 \\
    2 \\
    \end{bmatrix}
    = \begin{bmatrix}
    -2.232 \\
    -0.134 \\
    \end{bmatrix}
\]

A graphical representation of the transformation is shown below. The vector $[1, 2]^T$ is rotated $\frac{2\pi}{3}$ about the origin to get the vector $[-2.232, -0.134]^T$

<figure>
  <figcaption>Rotating one point about the origin</figcaption>
  <img style='width:250px' src='rotation.png'/>
</figure>

Next, let's look at how scale is represented. We can scale a vector $v \in \mathbb{R}^2$ by a scale factor $s$ by premultiplying it by the following matrix: 

\[
  \begin{bmatrix}
  s & 0 \\
  0 & s \\
    \end{bmatrix}
\]

We can scale a single point $[1, 2]^T$ by a factor of .5 as shown below:

\[
  \begin{bmatrix}
  .5 & 0 \\
  0 & .5 \\
    \end{bmatrix}
    \begin{bmatrix}
    1 \\
    2 \\
    \end{bmatrix}
    = \begin{bmatrix}
    .5 \\
    1 \\
    \end{bmatrix}
\]

<figure>
  <figcaption>Scaling one point</figcaption>
  <img style='width:250px' src='scale1.png' />
</figure>

When discussing scaling, it is helpful to consider multiple vectors, rather than a single vector. Let's look at all the points on a rectangle and multiply each of them by the scale matrix individually to see the effect of scaling by a factor of .5:

<figure>
  <figcaption>Scaling multiple points</figcaption>
  <img style='width:250px' src='scale2.png' />
</figure>

Now we can see that the rectangle was scaled by a factor of .5.

What about translation? Remember that an affine transformation is of the form $v \to Av + b$. You may have noticed that rotation and scale are represented by only a matrix $A$, with the vector $b$ effectively equal to 0. We could represent translation by simply adding a vector $b = [dx \; dy]^T$ to our vector $v$. However, it would be convenient if we could represent all of our transformations as matrices, and then obtain a single transformation matrix that scales, rotates, and translates a vector all at once. We could not achieve such a representation if we represent translation by adding a vector.

So how do we represent translation (moving $dx$ in the $x$ direction and $dy$ in the $y$ direction) with a matrix? First, we append a 1 to the end of $v$ to get $v' = [x, y, 1]^T$. Then, we premultiply $v'$ by the following matrix:

\[
  \begin{bmatrix}
  1 & 0 & dx\\
  0 & 1 & dy\\
  0 & 0 & 1\\
    \end{bmatrix}
\]

Even though we are representing our $x$ and $y$ positions with a 3-dimensional vector, we are only ever interested in the first two elements, which represent our $x$ and $y$ positions. The third element of $v'$ is \emph{always} equal to 1. Notice how premultiplying $v'$ by this matrix adds $dx$ to $x$ and $dy$ to $y$.
\[
  \begin{bmatrix}
  1 & 0 & dx\\
  0 & 1 & dy\\
  0 & 0 & 1\\
    \end{bmatrix}
     \begin{bmatrix}
    x \\
    y \\
    1 \\
    \end{bmatrix}
    = 
    \begin{bmatrix}
    x + dx \\
    y + dy\\
    1 \\
    \end{bmatrix}
\]

So this matrix is exactly what we want!

As a final note, we need to modify our scale and rotation matrices slightly in order to use them with $v'$ rather than $v$. A summary of the relevant affine transforms is below with these changes to the scale and rotation matrices.

\[
\boxed{
    \text{Rotation:}
  \begin{bmatrix}
  \cos \theta & -\sin \theta & 0 \\
  \sin \theta & \cos \theta & 0 \\
  0 & 0 & 1 \\
    \end{bmatrix}
    \quad \quad
    \text{Scale:}
  \begin{bmatrix}
  s & 0 & 0 \\
  0 & s & 0 \\
  0 & 0 & 1 \\
    \end{bmatrix}
        \quad \quad
    \text{Translation:}
  \begin{bmatrix}
  1 & 0 & dx \\
  0 & 1 & dy \\
  0 & 0 & 1 \\
    \end{bmatrix}
}
\]

## Estimating Position on the Pidrone

Now that we know how rotation, scale, and translation are represented as matrices, let's look at how you will be using these matrices in the sensors project. 

To estimate your drone's position, you will be using a function from OpenCV called\\
\texttt{esimateRigidTransform}. This function takes in two images $I_1$ and $I_2$ and a boolean $B$. The function returns a matrix estimating the affine transform that would turn the first image into the second image. The boolean $B$ indicates whether you want to estimate the affect of shearing on the image, which is another affine transform. We don't want this, so we set $B$ to \texttt{False}.

\texttt{esimateRigidTransform} returns a matrix in the form of:

\[  E = 
  \begin{bmatrix}
  s\cos \theta & -s\sin \theta & dx \\
  s\sin \theta & s\cos \theta & dy \\
  \end{bmatrix}
\]

This matrix should look familiar, but it is slightly different from the matrices we have seen in this section. Let $R$, $S$, and $T$ be the rotation, scale, and translation matrices from the above summary box. Then, $E$ is the same as $TRS$, where the bottom row of $TRS$ is removed. You can think of $E$ as a matrix that first scales a vector $u = [x, y, 1]^T$ by a factor of $s$, then rotates it by $\theta$, then translates it by $dx$ in the $x$ direction and $dy$ in the $y$ direction, and then removes the 1 appended to the end of the vector to output $u' = [x', y']$.

Wow that was a lot of reading! Now on to the questions...

## Questions
1. Your PiDrone is flying over a highly textured planar surface. The PiDrone's current $x$ position is $x_0$, its current $y$ position is $y_0$, and its current yaw is $\phi_0$. Using the PiCamera, you take a picture of the highly textured planar surface with the PiDrone in this state. You move the PiDrone to a different state ($x_1$ is your $x$ position, $y_1$ is your $y$ position, and $\phi_1$ is your yaw) and then take a picture of the highly textured planar surface using the PiCamera. You give these pictures to \texttt{esimateRigidTransform} and it returns a matrix $E$ in the form shown above. 
    
    Write expressions for $x_1$, $y_1$, and $\phi_1$. Your answers should be in terms of $x_0$, $y_0$, $\phi_0$, and the elements of $E$.

(Hint 1: Your solution does not have to involve matrix multiplication or other matrix operations. Feel free to pick out specific elements of the matrix using normal 0-indexing, i.e. $E[0][2]$. Hint 2: Use the function arctan2 in some way to compute the yaw.) 
