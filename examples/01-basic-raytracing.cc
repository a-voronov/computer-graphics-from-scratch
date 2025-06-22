#include "bmp.h"

int main() {
    const int32_t width = 600;
    const int32_t height = 600;
    uint8_t image[width * height][3];

    if (save_image(image, width, height, "gen/01-basic-raytracing.bmp")) {
        std::cout << "Image written successfully." << std::endl;
    }

    return 0;
}
