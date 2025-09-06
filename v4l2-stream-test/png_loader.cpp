#include "png_loader.h"
#include <png.h>
#include <cstdio>
#include <stdexcept>

static void fail_or_return(bool throw_on_error, const char* msg) {
    if (throw_on_error) throw std::runtime_error(msg);
}

bool load_png_rgb24(const std::string& path, PNGImage& out, bool throw_on_error) {
    FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) { fail_or_return(throw_on_error, "Failed to open PNG file"); return false; }

    png_byte header[8];
    if (std::fread(header, 1, 8, fp) != 8) {
        std::fclose(fp);
        fail_or_return(throw_on_error, "Failed to read PNG signature");
        return false;
    }
    if (png_sig_cmp(header, 0, 8)) {
        std::fclose(fp);
        fail_or_return(throw_on_error, "Not a PNG");
        return false;
    }

    png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        std::fclose(fp);
        fail_or_return(throw_on_error, "png_create_read_struct failed");
        return false;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_read_struct(&png_ptr, nullptr, nullptr);
        std::fclose(fp);
        fail_or_return(throw_on_error, "png_create_info_struct failed");
        return false;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        std::fclose(fp);
        fail_or_return(throw_on_error, "libpng error during read");
        return false;
    }

    png_init_io(png_ptr, fp);
    png_set_sig_bytes(png_ptr, 8);
    png_read_info(png_ptr, info_ptr);

    png_uint_32 width, height;
    int bit_depth, color_type;
    png_get_IHDR(png_ptr, info_ptr, &width, &height, &bit_depth, &color_type, nullptr, nullptr, nullptr);

    // Convert to 8-bit RGB
    if (bit_depth == 16) png_set_strip_16(png_ptr);
    if (color_type == PNG_COLOR_TYPE_PALETTE) png_set_palette_to_rgb(png_ptr);
    if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) png_set_expand_gray_1_2_4_to_8(png_ptr);
    if (png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) png_set_tRNS_to_alpha(png_ptr);
    if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA) png_set_gray_to_rgb(png_ptr);
    if (color_type & PNG_COLOR_MASK_ALPHA || png_get_valid(png_ptr, info_ptr, PNG_INFO_tRNS)) png_set_strip_alpha(png_ptr);

    png_read_update_info(png_ptr, info_ptr);

    png_size_t rowbytes = png_get_rowbytes(png_ptr, info_ptr);
    int channels = (int)png_get_channels(png_ptr, info_ptr);
    if (channels != 3) {
        png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
        std::fclose(fp);
        fail_or_return(throw_on_error, "Unexpected channels after conversion");
        return false;
    }

    std::vector<unsigned char> buffer(rowbytes * height);
    std::vector<png_bytep> row_ptrs(height);
    for (size_t y = 0; y < height; ++y) {
        row_ptrs[y] = (png_bytep)&buffer[y * rowbytes];
    }

    png_read_image(png_ptr, row_ptrs.data());
    png_read_end(png_ptr, nullptr);
    png_destroy_read_struct(&png_ptr, &info_ptr, nullptr);
    std::fclose(fp);

    out.width = (int)width;
    out.height = (int)height;
    out.rgb = std::move(buffer);
    return true;
}
