#include "io_utils.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

void io_utils::write_png(const char* filename,
                         unsigned char* image_data,
                         const unsigned int & width,
                         const unsigned int & height,
                         const unsigned int & channels)
{
    if (!stbi_write_png(filename, width, height, channels, image_data, width * channels * sizeof(unsigned char)))
        throw std::runtime_error(std::string("Failed to write image: ") + filename);
}

void io_utils::write_pfm(const char* filename,
                         float* image_data,
                         const unsigned int & width,
                         const unsigned int & height,
                         const unsigned int & channels)
{
    std::ofstream outfile(filename, std::ios::out | std::ios::binary);
    if (channels == 3)
        outfile << "PF\n";
    else if (channels == 1)
        outfile << "Pf\n";
    else
        throw std::runtime_error(
            std::string("Number of channels must be 1 or 3! Got ") + std::to_string(channels));
    outfile << width << " " << height << "\n";
    outfile << "-1\n";
    // Write one row at a time
    for (int y = 0; y < height; y++)
        outfile.write((char*) &image_data[y * width * channels], width * channels * sizeof(float));
    outfile.close();
}

std::string base_dir(const std::string& filepath)
{
    size_t slash_pos, backslash_pos;
    slash_pos = filepath.find_last_of('/');
    backslash_pos = filepath.find_last_of('\\');

    size_t break_pos;
    if (slash_pos == std::string::npos && backslash_pos == std::string::npos)
        return std::string();
    else if (slash_pos == std::string::npos)
        break_pos = backslash_pos;
    else if (backslash_pos == std::string::npos)
        break_pos = slash_pos;
    else
        break_pos = std::max(slash_pos, backslash_pos);
    return filepath.substr(0, break_pos + 1);
}

void io_utils::read_triangle_mesh(const char* mesh_filepath,
                                  Eigen::MatrixXd & V,
                                  Eigen::MatrixXi & F)
{
    std::string suffix;
    std::string fn(mesh_filepath);
    if (fn.length() > 4)
        suffix = fn.substr(fn.length() - 4);
    if (suffix == ".obj")
    {
        // Use tinyobjloader to load the mesh, since it can handle groups
        std::string err;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string mesh_filepath_base_dir = base_dir(std::string(mesh_filepath));
        bool success = tinyobj::LoadObj(
            shapes, materials, err, mesh_filepath, mesh_filepath_base_dir.c_str());
        if (!err.empty())
            std::cerr << err << std::endl;
        if (!success)
            throw std::runtime_error(err);

        int num_vertices = 0;
        int num_triangles = 0;
        for (auto & shape : shapes)
        {
            num_vertices  += static_cast<int32_t>(shape.mesh.positions.size()) / 3;
            num_triangles += static_cast<int32_t>(shape.mesh.indices.size()) / 3;
        }
        printf("[io_utils::read_triangle_mesh] V: %d | F: %d\n", num_vertices, num_triangles);

        V = Eigen::MatrixXd::Zero(num_vertices, 3);
        F = Eigen::MatrixXi::Zero(num_triangles, 3);

        uint32_t vrt_offset = 0;
        uint32_t tri_offset = 0;
        for (auto & shape : shapes)
        {
            for (uint64_t vi = 0; vi < shape.mesh.positions.size() / 3; vi++)
            {
                V(vrt_offset + vi, 0) = shape.mesh.positions[vi * 3 + 0];
                V(vrt_offset + vi, 1) = shape.mesh.positions[vi * 3 + 1];
                V(vrt_offset + vi, 2) = shape.mesh.positions[vi * 3 + 2];
            }

            for (uint64_t fi = 0; fi < shape.mesh.indices.size() / 3; fi++)
            {
                F(tri_offset + fi, 0) = shape.mesh.indices[fi * 3 + 0] + vrt_offset;
                F(tri_offset + fi, 1) = shape.mesh.indices[fi * 3 + 1] + vrt_offset;
                F(tri_offset + fi, 2) = shape.mesh.indices[fi * 3 + 2] + vrt_offset;
            }

            vrt_offset += static_cast<uint32_t>(shape.mesh.positions.size()) / 3;
            tri_offset += static_cast<uint32_t>(shape.mesh.indices.size()) / 3;
        }
    }
    else
        throw std::runtime_error(std::string("Unsupported mesh format: ") + suffix);
}
