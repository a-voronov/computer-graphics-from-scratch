// Shadows
//
// Book chapter: https://gabrielgambetta.com/computer-graphics-from-scratch/04-shadows-and-reflections.html#reflections
// Book example: https://github.com/ggambetta/computer-graphics-from-scratch/blob/master/demos/raytracer-05.html
//
// ```
// clang++ -std=c++17 examples/05-reflections.cc -o bin/05-reflections -O3 -fno-fast-math
// bin/05-reflections
// ```

#include "bmp.h"

const float EPSILON = 0.001;

struct Color {
    float red, green, blue;

    BMPColor bmpColor() {
        return {
            .red = static_cast<uint8_t>(round(clamp<float>(red * 255, 0, 255))),
            .green = static_cast<uint8_t>(round(clamp<float>(green * 255, 0, 255))),
            .blue = static_cast<uint8_t>(round(clamp<float>(blue * 255, 0, 255)))
        };
    }
};

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
    float specular;
    float reflective;
    Color color;
};

struct Scene {
    const float viewport_size = 1;
    const float projection_plane_z = 1;
    fVec3 camera_position;
    Color background_color;
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

Color operator*(const Color& color, float n) {
    return {
        .red = color.red * n,
        .green = color.green * n,
        .blue = color.blue * n
    };
}

Color operator+(const Color& lhs, const Color& rhs) {
    return {
        .red = lhs.red + rhs.red,
        .green = lhs.green + rhs.green,
        .blue = lhs.blue + rhs.blue
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

// find the closest intersection between a ray and the spheres in the scene.
pair<Sphere, float> closestIntersection(const fVec3& origin, const fVec3& direction, float t_min, float t_max, const Scene& scene) {
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

    return {closest_sphere, closest_t};
}

fVec3 reflectRay(const fVec3& ray, const fVec3& normal) {
    return normal * 2 * normal.dot(ray) - ray;
}

float computeLighting(const fVec3& point, const fVec3& normal, const fVec3& view, float specular, const Scene& scene) {
    float intensity = 0;

    for (int i = 0; i < scene.lights.size(); i++) {
        Light light = scene.lights[i];
        if (light.type == AMBIENT) {
            intensity += light.intensity;
        } else {
            fVec3 vec_l = {0, 0, 0};
            float shadow_t_max;

            if (light.type == POINT) {
                vec_l = light.position - point;
                shadow_t_max = 1;
            } else if (light.type == DIRECTIONAL) {
                vec_l = light.position;
                shadow_t_max = INFINITY;
            }

            // shadow check
            auto [shadow_sphere, shadow_t] = closestIntersection(point, vec_l, EPSILON, shadow_t_max, scene);
            if (shadow_t != INFINITY) {
                continue;
            }

            // diffuse
            float n_dot_l = normal.dot(vec_l);
            // since we're not making sure N and L are normalized,
            // we're using the following formula instead of simply `light.intensity * max(0.0f, n_dot_l)`
            if (n_dot_l > 0) {
                intensity += light.intensity * n_dot_l / (normal.length() * vec_l.length());
            }

            // specular
            if (specular >= 0) {
                fVec3 reflection = reflectRay(vec_l, normal);
                float r_dot_v = reflection.dot(view);
                if (r_dot_v > 0) {
                    intensity += light.intensity * pow(r_dot_v / (reflection.length() * view.length()), specular);
                }
            }
        }
    }
    return intensity;
}

// traces a ray against the set of spheres in the scene.
Color traceRay(const fVec3& origin, const fVec3& direction, float t_min, float t_max, int8_t recursion_depth, const Scene& scene) {
    auto [closest_sphere, closest_t] = closestIntersection(origin, direction, t_min, t_max, scene);

    if (closest_t == INFINITY) {
        return scene.background_color;
    }

    fVec3 point = origin + (direction * closest_t);
    fVec3 normal = (point - closest_sphere.center).normalized();
    fVec3 view = direction * -1;
    Color local_color = closest_sphere.color * computeLighting(point, normal, view, closest_sphere.specular, scene);

    // if we hit the recursion limit or the object is not reflective, we're done
    float reflective = closest_sphere.reflective;
    if (recursion_depth <= 0 or reflective <= 0) {
        return local_color;
    }
    // compute the reflected color
    fVec3 vec_r = reflectRay(view, normal);
    // adding a small bias to the point to fix 'shadow acne' effect on the yellow ball, which is so big it feels almost flat
    Color reflected_color = traceRay(point + (normal * EPSILON), vec_r, EPSILON, INFINITY, recursion_depth - 1, scene);

    return (local_color * (1 - reflective))
        + (reflected_color * reflective);
}

int main() {
    Image image = {600, 600};

    Scene scene = {
        .viewport_size = 1,
        .projection_plane_z = 1,
        .camera_position = {0, 0, 0},
        .background_color = {0, 0, 0},
        .spheres = {
            Sphere{.center = {0, -1, 3}, .radius = 1, .specular = 500, .reflective = 0.2, .color = {1, 0, 0}},
            Sphere{.center = {-2, 0, 4}, .radius = 1, .specular = 10, .reflective = 0.4, .color = {0, 1, 0}},
            Sphere{.center = {2, 0, 4}, .radius = 1, .specular = 500, .reflective = 0.3, .color = {0, 0, 1}},
            Sphere{.center = {0, -5001, 0}, .radius = 5000, .specular = 1000, .reflective = 0.5, .color = {1, 1, 0}}
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
            Color color = traceRay(scene.camera_position, direction, 1, INFINITY, 3, scene);
            image.data[image.offset(x, y)] = color.bmpColor();
        }
    }

    save_image(image.data, image.size.width, image.size.height, "results/05-reflections.bmp");

    return 0;
}
