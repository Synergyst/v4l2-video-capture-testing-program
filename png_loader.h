#pragma once
#include <vector>
#include <string>

struct PNGImage {
    int width = 0;
    int height = 0;
    std::vector<unsigned char> rgb; // RGB24
};

// Throws on failure with const char* message via exception if throw_on_error=true, else returns false.
bool load_png_rgb24(const std::string& path, PNGImage& out, bool throw_on_error = true);
