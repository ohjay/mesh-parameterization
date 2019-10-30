#include "geometry_images.h"
#include "io_utils.h"
#include <igl/read_triangle_mesh.h>
#include <igl/per_vertex_normals.h>
#include <igl/opengl/glfw/Viewer.h>
#include <Eigen/Core>
#include <string>
#include <iostream>

int main(int argc, char *argv[])
{
    // Load input mesh
    Eigen::MatrixXd V, U, UV_geom;
    Eigen::MatrixXi F;
    const char* mesh_filepath = (argc > 1) ? argv[1] : "../data/beetle.obj";
    igl::read_triangle_mesh(mesh_filepath, V, F);

    // Set up viewer
    igl::opengl::glfw::Viewer viewer;
    std::cout << R"(
  [space] Toggle whether displaying 3D surface or 2D parameterization
  C,c     Toggle checkerboard
)";

    // Compute parameterization
    Eigen::MatrixXd Vcut;
    Eigen::MatrixXi Fcut;
    geometry_image(V, F, UV_geom, Vcut, Fcut);
    V = Vcut;
    F = Fcut;

    // Fit parameterization in unit sphere
    const auto normalize = [](Eigen::MatrixXd &U)
    {
        U.rowwise() -= U.colwise().mean().eval();
        U.array() /= (U.colwise().maxCoeff() - U.colwise().minCoeff()).maxCoeff() / 2.0;
    };
    normalize(V);
    normalize(UV_geom);

    bool plot_parameterization = false;
    const auto & update = [&]()
    {
        if (plot_parameterization)
        {
            // Viewer wants 3D coordinates, so pad UVs with column of zeros
            viewer.data().set_vertices(
                (Eigen::MatrixXd(V.rows(), 3) <<
                    U.col(0), Eigen::VectorXd::Zero(V.rows()), U.col(1)).finished());
        }
        else
            viewer.data().set_vertices(V);
        viewer.data().compute_normals();
        viewer.data().set_uv(U * 10);
    };
    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer &, unsigned int key, int)
    {
        switch (key)
        {
            case ' ':
                plot_parameterization ^= 1;
                break;
            case 'l':
                U = UV_geom;
                break;
            case 'C':
            case 'c':
                viewer.data().show_texture ^= 1;
                break;
            default:
                return false;
        }
        update();
        return true;
    };

    // Write parameterization

    U = UV_geom;
    viewer.data().set_mesh(V, F);
    Eigen::MatrixXd N;
    igl::per_vertex_normals(V, F, N);
    viewer.data().set_colors(N.array() * 0.5 + 0.5);
    update();
    viewer.data().show_texture = true;
    viewer.data().show_lines = false;
    viewer.launch();

    return EXIT_SUCCESS;
}
