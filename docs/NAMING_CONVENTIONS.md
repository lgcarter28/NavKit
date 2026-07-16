# NavKit Naming Conventions

NavKit uses Groves-style inertial navigation notation in code and logs.

## Kinematic variable names

The convention is, for the mathematical variable $x_{ab}^c$, the programming variable name follows:

``` text
x_ab_c
```

where:

-   `a` and `b` define the object or relative relationship, where `b` is the object frame, and `a` is the frame object `b` is observed relative to.
-   `c` is the resolving frame,
-   `x` is the physical quantity.

Examples:

``` text
w_ib_b   angular rate of body frame b with respect to inertial frame i, resolved in body frame b
f_ib_b   specific force of body frame b with respect to inertial frame i, resolved in body frame b
C_eb     DCM that transforms b-frame components into e-frame components
q_b2e     quaternion that transforms b-frame components into e-frame components
v_eb_e   velocity of body/object b with respect to ECEF e, resolved in ECEF e
```

For coordinate transforms and attitude parameterizations where transform
direction is the primary meaning, prefer explicit `from2to` naming in code:

``` text
dcm_b2e        DCM that transforms body-frame components into ECEF-frame components
rpy_b2e_rad    roll/pitch/yaw parameterization of the body-to-ECEF transform
q_b2e          quaternion with the same body-to-ECEF transform convention
rotvec_b2e_rad small-angle attitude-error vector for the body-to-ECEF transform, resolved in ECEF
rotvec_b2n_rad small-angle attitude-error vector for the body-to-NED transform, resolved in NED
```

This avoids ambiguity around superscript/subscript conventions in code. Vector
kinematics keep the Groves-style object/reference/resolved-frame suffix, for
example `w_ib_b`.

## Simplified notation

When the object/wrt/resolving-frame meaning is obvious or redundant,
names may be simplified.

For example:

``` text
p_eb_e -> p_e
v_eb_e -> v_e
```

Use `p` for position, not `r`.

## Units in code vs logs

C++ variable names use Groves notation and usually omit unit suffixes
when the unit is clear from the type or struct documentation:

``` cpp
Eigen::Vector3d p_e;      // meters
Eigen::Vector3d v_e;      // meters/second
Eigen::Vector3d a_e;      // meters/second^2
Eigen::Quaterniond q_b2e;  // unit quaternion
Eigen::Vector3d w_ib_b;   // radians/second
```

CSV headers include units explicitly:

``` text
time_s,
p_e_x_m,p_e_y_m,p_e_z_m,
v_e_x_mps,v_e_y_mps,v_e_z_mps,
a_e_x_mps2,a_e_y_mps2,a_e_z_mps2,
q_b2e_w,q_b2e_x,q_b2e_y,q_b2e_z,
w_ib_b_x_radps,w_ib_b_y_radps,w_ib_b_z_radps
```
