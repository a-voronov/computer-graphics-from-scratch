// Diffuse Reflection
//
// Book chapter: https://gabrielgambetta.com/computer-graphics-from-scratch/03-light.html#diffuse-reflection
// Book example: https://github.com/ggambetta/computer-graphics-from-scratch/blob/master/demos/raytracer-02.html
//
// ```
// clang++ -std=c++17 examples/02-diffuse-reflection.cc -o bin/02-diffuse-reflection
// bin/02-diffuse-reflection
// ```

#include "bmp.h"

struct fVec3 {
    float x, y, z;

    fVec3 operator+(const fVec3& other) const {
        return { x + other.x, y + other.y, z + other.z };
    }

    fVec3 operator-(const fVec3& other) const {
        return { x - other.x, y - other.y, z - other.z };
    }

    fVec3 operator*(float n) const {
        return { x * n, y * n, z * n };
    }

    float dot(const fVec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    float length() const {
        return sqrt(this->dot(*this));
    }

    fVec3 normalized() const {
        return *this * (1.0 / this->length());
    }
};

enum LightType {
    AMBIENT, POINT, DIRECTIONAL
};

struct Light {
    fVec3 position;
    float intensity;
    LightType type;
};

struct Sphere {
    fVec3 center;
    float radius;
    BMPColor color;
};

struct Scene {
    const float viewport_size = 1;
    const float projection_plane_z = 1;
    fVec3 camera_position;
    BMPColor background_color;
    vector<Sphere> spheres;
    vector<Light> lights;
};

struct ImageSize {
    int32_t width;
    int32_t height;
};

struct Image {
    const ImageSize size;
    // flattened array of pixels
    vector<BMPColor> data;

    Image(int32_t width, int32_t height):
        size{width, height}, data(width * height) {}

    // computes the offset for the data based on x and y coordinates
    int32_t offset(int32_t x, int32_t y) const {
        x = size.width / 2 + x;
        y = size.height / 2 - y - 1;

        if (x < 0 || x >= size.width || y < 0 || y >= size.height) {
            throw out_of_range("Pixel coordinates must be within [0 < width, 0 < height]");
        }
        return x + size.width * y;
    }
};

BMPColor operator*(const BMPColor& color, float n) {
    return {
        .red = static_cast<uint8_t>(round(clamp<float>(color.red * n, 0, 255))),
        .green = static_cast<uint8_t>(round(clamp<float>(color.green * n, 0, 255))),
        .blue = static_cast<uint8_t>(round(clamp<float>(color.blue * n, 0, 255)))
    };
}

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

    float a = direction.dot(direction);
    float b = 2 * oc.dot(direction);
    float c = oc.dot(oc) - (sphere.radius * sphere.radius);

    float discriminant = (b * b) - (4 * a * c);
    if (discriminant < 0) {
        return {INFINITY, INFINITY};
    }

    float t1 = (-b + sqrt(discriminant)) / (2 * a);
    float t2 = (-b - sqrt(discriminant)) / (2 * a);

    return {t1, t2};
}

float computeLighting(const fVec3& point, const fVec3& normal, const Scene& scene) {
    float intensity = 0;

    for (int i = 0; i < scene.lights.size(); i++) {
        Light light = scene.lights[i];
        if (light.type == AMBIENT) {
            intensity += light.intensity;
        } else {
            fVec3 vec_l = {0, 0, 0};
            if (light.type == POINT) {
                vec_l = light.position - point;
            } else if (light.type == DIRECTIONAL) {
                vec_l = light.position;
            }

            float n_dot_l = normal.dot(vec_l);
            // since we're not making sure N and L are normalized,
            // we're using the following formula instead of simply `light.intensity * max(0.0f, n_dot_l)`
            if (n_dot_l > 0) {
                intensity += light.intensity * n_dot_l / (normal.length() * vec_l.length());
            }
        }
    }
    return intensity;
}

// traces a ray against the set of spheres in the scene.
BMPColor traceRay(const fVec3& origin, const fVec3& direction, float t_min, float t_max, const Scene& scene) {
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

    fVec3 point = origin + (direction * closest_t);
    fVec3 normal = (point - closest_sphere.center).normalized();

    return closest_sphere.color * computeLighting(point, normal, scene);
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
        },
        .lights = {
            Light{.type = AMBIENT, .intensity = 0.2},
            Light{.type = POINT, .intensity = 0.6, .position = {2, 1, 0}},
            Light{.type = DIRECTIONAL, .intensity = 0.2, .position = {1, 4, 4}}
        }
    };

    for (int32_t x = -image.size.width / 2; x < image.size.width / 2; x++) {
        for(int32_t y = -image.size.height / 2; y < image.size.height / 2; y++) {
            fVec3 direction = canvasToViewport(x, y, image.size, scene);
            BMPColor color = traceRay(scene.camera_position, direction, 1, INFINITY, scene);
            image.data[image.offset(x, y)] = color;
        }
    }

    save_image(image.data, image.size.width, image.size.height, "results/02-diffuse-reflection.bmp");

    return 0;
}
