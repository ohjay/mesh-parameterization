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
    Eigen::MatrixXd V, UV, UV_geom;
    Eigen::MatrixXi F;
    const char* mesh_filepath = (argc > 1) ? argv[1] : "../data/beetle.obj";
    igl::read_triangle_mesh(mesh_filepath, V, F);

    // Compute parameterization
    Eigen::MatrixXd Vcut;
    Eigen::MatrixXi Fcut;
    geometry_image(V, F, UV_geom, Vcut, Fcut);
    V = Vcut;
    F = Fcut;

    // Fit parameterization in unit sphere
    bool do_normalize = false;
    if (do_normalize)
    {
        const auto normalize = [](Eigen::MatrixXd & UV)
        {
            UV.rowwise() -= UV.colwise().mean().eval();
            UV.array() /= (UV.colwise().maxCoeff() - UV.colwise().minCoeff()).maxCoeff() / 2.0;
        };
        normalize(V);
        normalize(UV_geom);
    }

    // Write parameterization
    Eigen::MatrixXd N;
    igl::per_vertex_normals(V, F, N);
    int output_res = 256;
    std::vector<float> vertex_image(output_res * output_res * 3, 0.f);
    std::vector<float> normal_image(output_res * output_res * 3, 0.f);
    for (int vi = 0; vi < V.rows(); vi++)
    {
        // Forward mapping
        double u = (UV_geom(vi, 0) + 1.0) / 2.0;  // [0, 1]
        double v = (UV_geom(vi, 1) + 1.0) / 2.0;  // [0, 1]
        int x = static_cast<int>(u * (output_res - 1));
        int y = static_cast<int>(v * (output_res - 1));
        if (x >= 0 && x < output_res && y >= 0 && y < output_res)
        {
            vertex_image[(y * output_res + x) * 3 + 0] = V(vi, 0);
            vertex_image[(y * output_res + x) * 3 + 1] = V(vi, 1);
            vertex_image[(y * output_res + x) * 3 + 2] = V(vi, 2);
            normal_image[(y * output_res + x) * 3 + 0] = N(vi, 0);
            normal_image[(y * output_res + x) * 3 + 1] = N(vi, 1);
            normal_image[(y * output_res + x) * 3 + 2] = N(vi, 2);
        }
    }
    io_utils::write_pfm("vertices.pfm", vertex_image.data(), output_res, output_res, 3);
    io_utils::write_pfm("normals.pfm",  normal_image.data(), output_res, output_res, 3);

    // Set up viewer
    igl::opengl::glfw::Viewer viewer;
    std::cout << R"(
  [space] Toggle whether displaying 3D surface or 2D parameterization
  C,c     Toggle checkerboard
)";
    bool plot_parameterization = false;
    const auto & update = [&]()
    {
        if (plot_parameterization)
        {
            // Viewer wants 3D coordinates, so pad UVs with column of zeros
            viewer.data().set_vertices(
                (Eigen::MatrixXd(V.rows(), 3) <<
                    UV.col(0), Eigen::VectorXd::Zero(V.rows()), UV.col(1)).finished());
        }
        else
            viewer.data().set_vertices(V);
        viewer.data().compute_normals();
        viewer.data().set_uv(UV * 10);
    };
    viewer.callback_key_pressed = [&](igl::opengl::glfw::Viewer &, unsigned int key, int)
    {
        switch (key)
        {
            case ' ':
                plot_parameterization ^= 1;
                break;
            case 'l':
                UV = UV_geom;
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

    UV = UV_geom;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(N.array() * 0.5 + 0.5);
    update();
    viewer.data().show_texture = true;
    viewer.data().show_lines = false;
    viewer.launch();

    return EXIT_SUCCESS;
}
