// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

// Complete two-view Asymptote implementation of the NavKit 3-2-1 Euler
// rotation figure. Build this source with user mode "sequence" for page 1
// and "combined" for page 2, then losslessly concatenate the vector PDFs.

import three;

settings.tex = "pdflatex";
settings.prc = false;
settings.render = 0;  // retain vector 3-D output
texpreamble("\usepackage{amsmath}");
texpreamble("\usepackage{bm}");
texpreamble("\usepackage{lmodern}");
defaultpen(fontsize(9pt));
size(515pt, 0);

// --------------------------------------------------------------------------
// Mathematical parameters (the primary editing surface)
// --------------------------------------------------------------------------

real psi_deg   = 38;  // yaw:   rotation 1 about z^n
real theta_deg = 27;  // pitch: rotation 2 about y^1
real phi_deg   = 32;  // roll:  rotation 3 about x^2

real psi   = psi_deg*pi/180;
real theta = theta_deg*pi/180;
real phi   = phi_deg*pi/180;

real staged_axis_length = 2.32;
real staged_disk_radius = 1.56;
real staged_arc_radius  = 1.08;

// Orthographic projection avoids perspective distortion in a technical
// construction.  screen_right and screen_up support page-layout offsets while
// every frame vector remains a genuine world-space triple.
triple view_direction = unit((8, 16, 9));
triple screen_right   = unit(cross(Z, view_direction));
triple screen_up      = unit(cross(view_direction, screen_right));

currentprojection = orthographic(
    camera = 24*view_direction,
    up     = Z,
    target = O,
    zoom   = 0.90
);

// --------------------------------------------------------------------------
// Publication palette and stroke hierarchy
// --------------------------------------------------------------------------

// Exact matches for the hexadecimal palette in the TikZ source.
pen ink        = rgb(32/255, 40/255, 48/255);     // #202830
pen muted      = rgb(141/255, 153/255, 163/255);  // #8D99A3
pen guide      = muted;
pen nav_blue   = rgb(24/255, 126/255, 168/255);   // #187EA8
pen yaw_teal   = rgb(21/255, 150/255, 125/255);   // #15967D
pen pitch_gold = rgb(208/255, 139/255, 37/255);   // #D08B25
pen body_red   = rgb(201/255, 76/255, 58/255);    // #C94C3A

pen arc_pen = linewidth(1.10pt);

// --------------------------------------------------------------------------
// Reusable 3-D mathematics and drawing helpers
// --------------------------------------------------------------------------

// Rodrigues' formula: a right-handed active rotation of v about axis.
triple rotated(triple v, triple axis, real angle)
{
    triple a = unit(axis);
    return cos(angle)*v + sin(angle)*cross(a, v)
         + (1-cos(angle))*dot(a, v)*a;
}

// A sampled circular arc in the plane normal to axis.  u is the radial
// direction at angle zero, so the arc is mathematically coupled to the frame.
path3 rotation_arc(triple origin, triple axis, triple u,
                   real angle, real radius, int samples=48)
{
    triple a = unit(axis);
    triple e1 = unit(u - dot(u, a)*a);
    triple e2 = unit(cross(a, e1));
    path3 result = origin + radius*e1;

    for (int i=1; i <= samples; ++i) {
        real t = angle*i/samples;
        result = result -- (origin + radius*(cos(t)*e1 + sin(t)*e2));
    }
    return result;
}

triple rotation_arc_outer_endpoint(triple origin, triple axis, triple u,
                                   real angle, real radius)
{
    triple a = unit(axis);
    triple e1 = unit(u - dot(u, a)*a);
    triple e2 = unit(cross(a, e1));
    return origin + radius*(cos(angle)*e1 + sin(angle)*e2);
}

void draw_staged_rotation_disk(triple origin, triple normal, pen color)
{
    path3 rim = circle(origin, staged_disk_radius, unit(normal));
    draw(surface(rim), color + opacity(0.075));
    draw(rim, color + linewidth(0.52pt) + opacity(0.72));
}

void draw_staged_axis(triple origin, triple direction, string name,
                      pen stroke, pair label_align)
{
    triple tip = origin + staged_axis_length*unit(direction);
    draw(origin -- tip, stroke, Arrow3(size=4.6));
    if (name != "") {
        label(name, origin + 1.09*staged_axis_length*unit(direction),
              label_align, stroke + fontsize(8.5pt));
    }
}

void draw_staged_frame(triple origin, triple ex, triple ey, triple ez,
                       string x_name, string y_name, string z_name,
                       pen stroke, pair x_align=SE, pair y_align=W,
                       pair z_align=N)
{
    draw_staged_axis(origin, ex, x_name, stroke, x_align);
    draw_staged_axis(origin, ey, y_name, stroke, y_align);
    draw_staged_axis(origin, ez, z_name, stroke, z_align);
    dot(origin, ink + linewidth(2.2pt));
}

void draw_staged_angle(triple origin, triple axis, triple start_direction,
                       real angle, string symbol, pen color,
                       pair label_align,
                       real label_radius_factor=1.22,
                       triple label_offset=O,
                       real arc_radius_factor=1.0)
{
    path3 a = rotation_arc(origin, axis, start_direction,
                           angle, arc_radius_factor*staged_arc_radius);
    draw(a, color + arc_pen, Arrow3(size=4.2));
    label(symbol,
          rotation_arc_outer_endpoint(
              origin, axis, start_direction, angle,
              label_radius_factor*arc_radius_factor*staged_arc_radius)
          + label_offset,
          label_align, color + fontsize(10pt));
}

void draw_staged_axis_rotation(triple origin, triple axis,
                               triple radial_direction, string symbol,
                               pen color, pair label_align)
{
    triple center = origin + 0.72*staged_axis_length*unit(axis);
    real indicator_angle = 1.55*pi;
    real indicator_radius = 0.27;
    path3 a = rotation_arc(center, axis, radial_direction,
                           indicator_angle, indicator_radius, 64);
    draw(a, color + arc_pen, Arrow3(size=4.0));
    label(symbol,
          rotation_arc_outer_endpoint(center, axis, radial_direction,
                                      indicator_angle,
                                      1.35*indicator_radius),
          label_align, color + fontsize(9.5pt));
}

void draw_stage_title(triple origin, string ordinal, string title,
                      string axis_name)
{
    triple anchor = origin + 2.65*screen_up;
    label("{\bf (" + ordinal + ") " + title + "} about $" + axis_name + "$",
          anchor, N, ink + fontsize(9pt));
}

// --------------------------------------------------------------------------
// Successive intrinsic frames
// --------------------------------------------------------------------------

triple xn = X;
triple yn = Y;
triple zn = Z;

triple x1 = rotated(xn, zn, psi);
triple y1 = rotated(yn, zn, psi);
triple z1 = zn;

triple x2 = rotated(x1, y1, theta);
triple y2 = y1;
triple z2 = rotated(z1, y1, theta);

triple xb = x2;
triple yb = rotated(y2, x2, phi);
triple zb = rotated(z2, x2, phi);

// --------------------------------------------------------------------------
// Page 1: staged yaw, pitch, and roll, strictly left to right
// --------------------------------------------------------------------------

void draw_sequence_page()
{
    triple yaw_origin   = -5.40*screen_right + 0.70*screen_up;
    triple pitch_origin =  0.70*screen_up;
    triple roll_origin  =  5.40*screen_right + 0.70*screen_up;

    draw((-0.02*screen_right + 5.25*screen_up)
         -- (0.02*screen_right + 5.25*screen_up),
         white + linewidth(0.1pt));
    // Native 3-D rendering otherwise crops tightly through the final equation.
    draw((-0.02*screen_right - 6.05*screen_up)
         -- (0.02*screen_right - 6.05*screen_up),
         white + linewidth(0.1pt));

    draw_staged_rotation_disk(yaw_origin, zn, nav_blue);
    draw_staged_frame(yaw_origin, xn, yn, zn,
                      "$x_n$", "$y_n$", "$z_n=z_1$",
                      nav_blue + linewidth(0.82pt) + opacity(0.82));
    draw_staged_axis(yaw_origin, x1, "$x_1$",
                     yaw_teal + linewidth(1.08pt), SE);
    draw_staged_axis(yaw_origin, y1, "$y_1$",
                     yaw_teal + linewidth(1.08pt), W);
    draw_staged_angle(yaw_origin, zn, xn, psi, "$\psi$", nav_blue, NE);
    draw_staged_angle(yaw_origin, zn, yn, psi, "$\psi$", nav_blue, NW,
                      1.27, O, 0.82);
    draw_staged_axis_rotation(yaw_origin, zn, xn, "$\psi$", nav_blue, NE);
    draw_stage_title(yaw_origin, "a", "Yaw", "z_n");

    draw_staged_rotation_disk(pitch_origin, y1, yaw_teal);
    draw_staged_frame(pitch_origin, x1, y1, z1,
                      "$x_1$", "$y_1=y_2$", "$z_1$",
                      yaw_teal + linewidth(0.82pt) + opacity(0.82));
    draw_staged_axis(pitch_origin, x2, "$x_2$",
                     pitch_gold + linewidth(1.08pt), SE);
    draw_staged_axis(pitch_origin, z2, "$z_2$",
                     pitch_gold + linewidth(1.08pt), N);
    draw_staged_angle(pitch_origin, y1, x1, theta, "$\theta$", yaw_teal, E,
                      1.34, 0.22*y1);
    draw_staged_angle(pitch_origin, y1, z1, theta, "$\theta$", yaw_teal, W,
                      1.26, -0.18*y1, 0.82);
    draw_staged_axis_rotation(pitch_origin, y1, x1, "$\theta$", yaw_teal,
                              NE);
    draw_stage_title(pitch_origin, "b", "Pitch", "y_1");

    draw_staged_rotation_disk(roll_origin, x2, pitch_gold);
    draw_staged_frame(roll_origin, x2, y2, z2,
                      "$x_2=x_b$", "$y_2$", "$z_2$",
                      pitch_gold + linewidth(0.82pt) + opacity(0.82),
                      W, W, N);
    draw_staged_axis(roll_origin, yb, "$y_b$",
                     body_red + linewidth(1.08pt), W);
    draw_staged_axis(roll_origin, zb, "$z_b$",
                     body_red + linewidth(1.08pt), N);
    draw_staged_angle(roll_origin, x2, y2, phi, "$\phi$", pitch_gold, NW,
                      1.34, 0.22*x2);
    draw_staged_angle(roll_origin, x2, z2, phi, "$\phi$", pitch_gold, SE,
                      1.28, -0.18*x2, 0.82);
    draw_staged_axis_rotation(roll_origin, x2, y2, "$\phi$", pitch_gold, NE);
    draw_stage_title(roll_origin, "c", "Roll", "x_2");

    triple separator_left  = -8.0*screen_right - 4.25*screen_up;
    triple separator_right =  8.0*screen_right - 4.25*screen_up;
    draw(separator_left -- separator_right,
         guide + linewidth(0.42pt) + opacity(0.85));
    label("Successive right-handed frame construction for NavKit's passive "
          "component transform:",
          -4.55*screen_up, N, muted + fontsize(8pt));
    label("$\mathcal F^{n}"
          "\xrightarrow[\;z_n\;]{\psi}\mathcal F^{1}"
          "\xrightarrow[\;y_1\;]{\theta}\mathcal F^{2}"
          "\xrightarrow[\;x_2\;]{\phi}\mathcal F^{b}$",
          -5.10*screen_up, N, ink + fontsize(9pt));
    label("$\bm v^{b}=\bm C_{n}^{b}\bm v^{n},\qquad"
          "\bm C_{n}^{b}=\bm R_{3}(\psi)\,\bm R_{2}(\theta)\,"
          "\bm R_{1}(\phi)$",
          -5.55*screen_up, N, ink + fontsize(9.5pt));
}

// --------------------------------------------------------------------------
// Page 2: combined intrinsic construction
// --------------------------------------------------------------------------

real combined_axis_length = 2.70;
real combined_disk_radius = 1.86;
real combined_arc_radius  = 1.18;

pen combined_n_pen = nav_blue + linewidth(1.00pt);
pen combined_1_pen = yaw_teal + linewidth(0.74pt)
                   + dashed + opacity(0.88);
pen combined_2_pen = pitch_gold + linewidth(0.74pt)
                   + dashed + opacity(0.88);
pen combined_b_pen = body_red + linewidth(1.16pt);

void draw_combined_disk(triple normal, real radius, pen color)
{
    path3 rim = circle(O, radius, unit(normal));
    draw(surface(rim), color + opacity(0.040));
    draw(rim, color + linewidth(0.58pt) + opacity(0.66));
}

void draw_combined_axis(triple direction, string name, pen stroke,
                        pair label_align, real scale=1.0,
                        triple label_offset=O)
{
    triple tip = scale*combined_axis_length*unit(direction);
    draw(O -- tip, stroke, Arrow3(size=5.0));
    label(name,
          1.075*scale*combined_axis_length*unit(direction) + label_offset,
          label_align, stroke + fontsize(9pt));
}

void draw_combined_angle(triple axis, triple start_direction,
                         real angle, string symbol, pen stroke,
                         pair label_align, real radius,
                         real label_radius_factor=1.19,
                         triple label_offset=O)
{
    path3 a = rotation_arc(O, axis, start_direction, angle, radius);
    draw(a, stroke + linewidth(1.18pt), Arrow3(size=4.4));
    label(symbol,
          rotation_arc_outer_endpoint(O, axis, start_direction, angle,
                                      label_radius_factor*radius)
          + label_offset,
          label_align, stroke + fontsize(10.5pt));
}

void draw_combined_axis_rotation(triple axis, triple radial_direction,
                                 string symbol, pen color,
                                 pair label_align)
{
    triple center = 0.72*combined_axis_length*unit(axis);
    real indicator_angle = 1.55*pi;
    real indicator_radius = 0.29;
    path3 a = rotation_arc(center, axis, radial_direction,
                           indicator_angle, indicator_radius, 64);
    draw(a, color + linewidth(1.18pt), Arrow3(size=4.2));
    label(symbol,
          rotation_arc_outer_endpoint(center, axis, radial_direction,
                                      indicator_angle,
                                      1.35*indicator_radius),
          label_align, color + fontsize(10pt));
}

void draw_combined_page()
{
    draw((-0.02*screen_right + 4.70*screen_up)
         -- (0.02*screen_right + 4.70*screen_up),
         white + linewidth(0.1pt));
    draw((-0.02*screen_right - 4.65*screen_up)
         -- (0.02*screen_right - 4.65*screen_up),
         white + linewidth(0.1pt));

    draw_combined_disk(zn, combined_disk_radius, nav_blue);
    draw_combined_disk(y1, 0.91*combined_disk_radius, yaw_teal);
    draw_combined_disk(x2, 0.83*combined_disk_radius, pitch_gold);

    draw_combined_axis(xn, "$x_n$", combined_n_pen, SE);
    draw_combined_axis(yn, "$y_n$", combined_n_pen, W);
    draw_combined_axis(zn, "$z_n$", combined_n_pen, N);
    draw_combined_axis(x1, "$x_1$", combined_1_pen, SE, 0.92);
    draw_combined_axis(y1, "$y_1=y_2$", combined_1_pen, NW, 0.92,
                       0.09*screen_up - 0.13*screen_right);
    draw_combined_axis(z2, "$z_2$", combined_2_pen, NE, 0.84,
                       0.08*screen_up);
    draw_combined_axis(xb, "$x_2=x_b$", combined_b_pen, SE, 0.96);
    draw_combined_axis(yb, "$y_b$", combined_b_pen, NW);
    draw_combined_axis(zb, "$z_b$", combined_b_pen, NE);

    draw_combined_angle(zn, xn, psi, "$\psi$", nav_blue, NE,
                        0.92*combined_arc_radius);
    draw_combined_angle(zn, yn, psi, "$\psi$", nav_blue, NW,
                        0.72*combined_arc_radius, 1.26);
    draw_combined_angle(y1, x1, theta, "$\theta$", yaw_teal, E,
                        1.04*combined_arc_radius, 1.22, -0.06*y1);
    draw_combined_angle(y1, z1, theta, "$\theta$", yaw_teal, W,
                        0.79*combined_arc_radius, 1.25, -0.08*y1);
    draw_combined_angle(x2, y2, phi, "$\phi$", pitch_gold, NW,
                        1.15*combined_arc_radius, 1.20, 0.10*x2);
    draw_combined_angle(x2, z2, phi, "$\phi$", pitch_gold, SE,
                        0.88*combined_arc_radius, 1.24, -0.08*x2);
    draw_combined_axis_rotation(zn, xn, "$\psi$", nav_blue, NE);
    draw_combined_axis_rotation(y1, x1, "$\theta$", yaw_teal, NE);
    draw_combined_axis_rotation(x2, y2, "$\phi$", pitch_gold, NE);
    dot(O, ink + linewidth(2.5pt));

    label("{\bf Combined intrinsic 3--2--1 Euler frame construction}",
          4.28*screen_up, N, ink + fontsize(12pt));
    label("The translucent disks are the successive yaw, pitch, and roll "
          "planes; all geometry is derived from one set of angles.",
          3.88*screen_up, N, muted + fontsize(8.2pt));

    triple legend_origin = 3.40*screen_right + 2.64*screen_up;
    real legend_row_step = 0.29;
    draw(legend_origin -- (legend_origin + 0.66*screen_right),
         combined_n_pen);
    label("$\mathcal F^n$ initial", legend_origin + 0.78*screen_right,
          E, nav_blue + fontsize(8pt));
    draw((legend_origin - legend_row_step*screen_up)
         -- (legend_origin + 0.66*screen_right
             - legend_row_step*screen_up), combined_1_pen);
    label("$\mathcal F^1$ intermediate",
          legend_origin + 0.78*screen_right - legend_row_step*screen_up,
          E, yaw_teal + fontsize(8pt));
    draw((legend_origin - 2*legend_row_step*screen_up)
         -- (legend_origin + 0.66*screen_right
             - 2*legend_row_step*screen_up), combined_2_pen);
    label("$\mathcal F^2$ intermediate",
          legend_origin + 0.78*screen_right - 2*legend_row_step*screen_up,
          E, pitch_gold + fontsize(8pt));
    draw((legend_origin - 3*legend_row_step*screen_up)
         -- (legend_origin + 0.66*screen_right
             - 3*legend_row_step*screen_up), combined_b_pen);
    label("$\mathcal F^b$ final",
          legend_origin + 0.78*screen_right - 3*legend_row_step*screen_up,
          E, body_red + fontsize(8pt));

    triple separator_left  = -5.15*screen_right - 3.58*screen_up;
    triple separator_right =  5.15*screen_right - 3.58*screen_up;
    draw(separator_left -- separator_right,
         guide + linewidth(0.42pt) + opacity(0.85));
    label("$\mathcal F^{n}"
          "\xrightarrow[\;z_n\;]{\psi}\mathcal F^{1}"
          "\xrightarrow[\;y_1\;]{\theta}\mathcal F^{2}"
          "\xrightarrow[\;x_2\;]{\phi}\mathcal F^{b}$",
          -3.88*screen_up, N, ink + fontsize(9.5pt));
    label("NavKit passive component mapping:\qquad"
          "$\bm v^{b}=\bm C_{n}^{b}\bm v^{n},\qquad"
          "\bm C_{n}^{b}=\bm R_{3}(\psi)\,\bm R_{2}(\theta)\,"
          "\bm R_{1}(\phi)$",
          -4.34*screen_up, N, ink + fontsize(9pt));
}

// Asymptote 3.09 preserves the command-line separator in settings.user, so
// `-u combined` is exposed as `combined;`.
if (settings.user == "combined;") {
    draw_combined_page();
} else {
    draw_sequence_page();
}
