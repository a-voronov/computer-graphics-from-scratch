#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>

using namespace std;

#pragma pack(push, 1)                       // ensure the structures are packed

struct BMPHeader {
    uint8_t file_type[2] = {'B', 'M'};      // always BM
    uint32_t file_size;                     // file size, in bytes
    uint32_t reserved = 0;                  // reserved, always 0
    uint32_t data_offset;                   // where pixel data starts
};

struct BMPInfoHeader {
    uint32_t size = sizeof(BMPInfoHeader);  // this header size, in bytes
    int32_t width;                          // image width, in pixels
    int32_t height;                         // image height, in pixels
    uint16_t num_color_planes = 1;          // num of color planes, always 1
    uint16_t color_depth = 24;              // color depth, 8 bits per BGR colors = 24
    uint32_t compression_method = 0;        // compression method, uncompressed is 0
    uint32_t raw_image_size = 0;            // raw bitmap size in bytes, uncompressed is 0
    int32_t horizontal_resolution = 2835;   // pixel per meter, default 2835 = 72DPI
    int32_t vertical_resolution = 2835;     // pixel per meter, default 2835 = 72DPI
    uint32_t colors_used = 0;               // color table entries
    uint32_t important_colors = 0;          // important colors number, 0 for all
};

#pragma pack(pop)

struct Color {
    uint8_t red, green, blue;
};

void save_image(const vector<Color>& image, int32_t width, int32_t height, const string& filename) {
    if (width % 4 != 0) {
        throw invalid_argument("Image width should be aligned to a multiple of 4");
    }

    ofstream file(filename, ios::binary);
    if (!file.is_open()) {
        throw runtime_error("Error opening file: " + filename);
    }

    BMPInfoHeader info_header;
    BMPHeader bmp_header;

    info_header.width = width;
    info_header.height = height;
    info_header.raw_image_size = width * height * 3;

    bmp_header.data_offset = info_header.size + sizeof(BMPHeader);
    bmp_header.file_size = bmp_header.data_offset + info_header.raw_image_size;

    file.write((const char*)&bmp_header, sizeof(bmp_header));
    file.write((const char*)&info_header, sizeof(info_header));

    for (int32_t y = height - 1; y >= 0; y--) {
        for (int32_t x = 0; x < width; x++) {
            const Color rgb = image[y * width + x];
            // converting RGB image to BGR for bmp file format
            file.put(rgb.blue);
            file.put(rgb.green);
            file.put(rgb.red);
        }
    }
    file.close();

    if (!file.good()) {
        throw runtime_error("Error writing to file: " + filename);
    }
}
