#include "geometry_images.h"

void geometry_image(
    const Eigen::MatrixXd & V,
    const Eigen::MatrixXi & F,
    Eigen::MatrixXd & U)
{
    // Replace with your code
    U = V.leftCols(2);
}
