#include <string>
#include <iostream>
#include <fstream>

namespace io
{
    void write_png(const char* filename,
                   unsigned char* image_data,
                   const unsigned int & height,
                   const unsigned int & width,
                   const unsigned int & channels);

    void write_pfm(const char* filename,
                   float* image_data,
                   const unsigned int & height,
                   const unsigned int & width,
                   const unsigned int & channels);
}
