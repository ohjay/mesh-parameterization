#include "io_utils.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

void io::write_png(const char* filename,
                   unsigned char* image_data,
                   const unsigned int & height,
                   const unsigned int & width,
                   const unsigned int & channels)
{
    if (!stbi_write_png(filename, width, height, channels, image_data, width * channels * sizeof(unsigned char)))
        throw std::runtime_error(std::string("Failed to write image: ") + filename);
}

void io::write_pfm(const char* filename,
                   float* image_data,
                   const unsigned int & height,
                   const unsigned int & width,
                   const unsigned int & channels)
{
    std::ofstream outfile(filename, std::ios::out | std::ios::binary);
    outfile << "PF\n";
    outfile << width << " " << height << "\n";
    outfile << "-1\n";
    // Write one row at a time
    for (int y = 0; y < height; y++)
        outfile.write((char*) &image_data[y * width * channels], width * channels * sizeof(float));
    outfile.close();
}
