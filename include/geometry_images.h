#ifndef GEOMETRY_IMAGES_H
#define GEOMETRY_IMAGES_H

#include <Eigen/Core>

// Given a 3D mesh (`V`, `F`) with boundary, compute a 2D parameterization that
// minimizes the "least squares conformal" energy:
//
// \\[
// ∫_Ω ‖ ∇v - (∇u)^⊥ ‖² dA,
// \\]
//
// where u and v are the unknown (output) coordinates in the parametric domain `U`.
//
// Inputs:
//   V  #V by 3 list of mesh vertex positions
//   F  #F by 3 list of triangle indices into V
// Outputs:
//   U  #U by 2 list of mesh UV parameterization coordinates
//
void geometry_image(const Eigen::MatrixXd & V,
                    const Eigen::MatrixXi & F,
                    Eigen::MatrixXd & U,
                    Eigen::MatrixXd & Vcut,
                    Eigen::MatrixXi & Fcut);

#endif
