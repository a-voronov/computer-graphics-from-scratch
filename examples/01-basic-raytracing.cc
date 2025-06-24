// Basic Raytracing
//
// Book chapter: https://gabrielgambetta.com/computer-graphics-from-scratch/02-basic-raytracing.html
// Book example: https://github.com/ggambetta/computer-graphics-from-scratch/blob/master/demos/raytracer-01.html

#include "bmp.h"

struct fVec3 {
    float x, y, z;

    fVec3 operator-(const fVec3& other) const {
        return { x - other.x, y - other.y, z - other.z };
    }

    float dot(const fVec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
};

struct Sphere {
    fVec3 center;
    float radius;
    Color color;
};

struct Scene {
    const float viewport_size = 1;
    const float projection_plane_z = 1;
    fVec3 camera_position;
    Color background_color;
    vector<Sphere> spheres;
};

struct ImageSize {
    int32_t width;
    int32_t height;
};

struct Image {
    const ImageSize size;
    // flattened array of pixels
    vector<Color> data;

    Image(int32_t width, int32_t height):
        size{width, height}, data(width * height) {}

    // computes the offset for the data based on x and y coordinates
    int32_t offset(int32_t x, int32_t y) {
        x = size.width / 2 + x;
        y = size.height / 2 - y - 1;

        if (x < 0 || x >= size.width || y < 0 || y >= size.height) {
            throw out_of_range("Pixel coordinates must be within [0 < width, 0 < height]");
        }
        return x + size.width * y;
    }
};

// converts 2D canvas coordinates to 3D viewport coordinates.
fVec3 canvasToViewport(int32_t x, int32_t y, const ImageSize& size, const Scene& scene) {
    return fVec3 {
        (float)x * scene.viewport_size / size.width,
        (float)y * scene.viewport_size / size.height,
        scene.projection_plane_z
    };
}

// computes the intersection of a ray and a sphere. Returns the values of t for the intersections.
pair<float, float> intersectRaySphere(const fVec3& origin, const fVec3& direction, const Sphere& sphere) {
    fVec3 oc = origin - sphere.center;

    float k1 = direction.dot(direction);
    float k2 = 2 * oc.dot(direction);
    float k3 = oc.dot(oc) - (sphere.radius * sphere.radius);

    float discriminant = (k2 * k2) - (4 * k1 * k3);
    if (discriminant < 0) {
        return {INFINITY, INFINITY};
    }

    float t1 = (-k2 + sqrtf(discriminant)) / (2 * k1);
    float t2 = (-k2 - sqrtf(discriminant)) / (2 * k1);

    return {t1, t2};
}

// traces a ray against the set of spheres in the scene.
Color traceRay(const fVec3& origin, const fVec3& direction, float t_min, float t_max, const Scene& scene) {
    float closest_t = INFINITY;
    Sphere closest_sphere;

    for (int32_t i = 0; i < scene.spheres.size(); i++) {
        auto [xt_min, xt_max] = intersectRaySphere(origin, direction, scene.spheres[i]);
        if (xt_min < closest_t && t_min < xt_min && xt_min < t_max) {
            closest_t = xt_min;
            closest_sphere = scene.spheres[i];
        }
        if (xt_max < closest_t && t_min < xt_max && xt_max < t_max) {
            closest_t = xt_max;
            closest_sphere = scene.spheres[i];
        }
    }

    if (closest_t == INFINITY) {
        return scene.background_color;
    }
    return closest_sphere.color;
}

int main() {
    Image image = {600, 600};

    Scene scene = {
        .viewport_size = 1,
        .projection_plane_z = 1,
        .camera_position = {0, 0, 0},
        .background_color = {255, 255, 255},
        .spheres = {
            Sphere{.center = {0, -1, 3}, .radius = 1, .color = {255, 0, 0}},
            Sphere{.center = {-2, 0, 4}, .radius = 1, .color = {0, 255, 0}},
            Sphere{.center = {2, 0, 4}, .radius = 1, .color = {0, 0, 255}},
            Sphere{.center = {0, -5001, 0}, .radius = 5000, .color = {255, 255, 0}}
        }
    };

    for (int32_t x = -image.size.width / 2; x < image.size.width / 2; x++) {
        for(int32_t y = -image.size.height / 2; y < image.size.height / 2; y++) {
            fVec3 direction = canvasToViewport(x, y, image.size, scene);
            Color color = traceRay(scene.camera_position, direction, 1, INFINITY, scene);
            image.data[image.offset(x, y)] = color;
        }
    }

    save_image(image.data, image.size.width, image.size.height, "results/01-basic-raytracing.bmp");

    return 0;
}
