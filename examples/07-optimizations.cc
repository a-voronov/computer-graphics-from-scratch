// Shadows
//
// Book chapter: https://gabrielgambetta.com/computer-graphics-from-scratch/05-extending-the-raytracer.html#optimization
// Book example: -
//
// ```
// clang++ -std=c++17 examples/07-optimizations.cc -o bin/07-optimizations
// bin/07-optimizations
// ```

#include "bmp.h"
#include "bvh.h"

const float EPSILON = 0.001;

struct Mat33 {
    Vec3 a, b, c;

    Vec3 operator*(const Vec3& vector) const {
        return { a.dot(vector), b.dot(vector), c.dot(vector) };
    }
};

enum LightType {
    AMBIENT, POINT, DIRECTIONAL
};

struct Light {
    Vec3 position;
    float intensity;
    LightType type;
};

struct Sphere : Object {
    Vec3 center;
    float radius;

    Sphere(const Vec3& c, float r, float spec, float refl, const Color& col) : center(c), radius(r) {
        specular = spec;
        reflective = refl;
        color = col;
    }

    AABB bounds() const {
        return AABB{
            center - Vec3{radius, radius, radius},
            center + Vec3{radius, radius, radius}
        };
    }

    Vec3 centroid() const {
        return center;
    }

    optional<HitInfo> intersect(const Ray& ray, float t_min, float t_max) const {
        Vec3 oc = ray.origin - center;

        float a = ray.direction.dot(ray.direction);
        float b = 2 * oc.dot(ray.direction);
        float c = oc.dot(oc) - (radius * radius);

        float discriminant = (b * b) - (4 * a * c);
        if (discriminant < 0) {
            return nullopt;
        }

        float sqrtD = sqrt(discriminant);
        float closest_t = (-b - sqrtD) / (2 * a);
        if (closest_t < t_min || closest_t > t_max) {
            closest_t = (-b + sqrtD) / (2 * a);
            if (closest_t < t_min || closest_t > t_max) {
                return nullopt;
            }
        }

        Vec3 hitPoint = ray.at(closest_t);
        return HitInfo{
            .point = hitPoint,
            .normal = (hitPoint - center).normalized(),
            .t = closest_t,
            .object = this
        };
    }
};

struct Camera {
    Vec3 position;
    Mat33 rotation;
};

struct Scene {
    const float viewport_size = 1;
    const float projection_plane_z = 1;
    Vec3 camera_position;
    Color background_color;
    vector<Light> lights;
    vector<unique_ptr<Object>> objects;
    unique_ptr<BVHNode> bvh;
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
Vec3 canvasToViewport(int32_t x, int32_t y, const ImageSize& size, const Scene& scene) {
    return Vec3 {
        (float)x * scene.viewport_size / size.width,
        (float)y * scene.viewport_size / size.height,
        scene.projection_plane_z
    };
}

Vec3 reflectRay(const Vec3& ray, const Vec3& normal) {
    return normal * 2 * normal.dot(ray) - ray;
}

float computeLighting(const Vec3& point, const Vec3& normal, const Vec3& view, float specular, const Scene& scene) {
    float intensity = 0;

    for (const Light& light : scene.lights) {
        if (light.type == AMBIENT) {
            intensity += light.intensity;
        } else {
            Vec3 vec_l = {0, 0, 0};
            float shadow_t_max;

            if (light.type == POINT) {
                vec_l = light.position - point;
                shadow_t_max = 1;
            } else if (light.type == DIRECTIONAL) {
                vec_l = light.position;
                shadow_t_max = INFINITY;
            }

            // shadow check
            if (scene.bvh->intersectAnything(Ray{point, vec_l}, EPSILON, shadow_t_max)) {
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
                Vec3 reflection = reflectRay(vec_l, normal);
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
Color traceRay(const Ray& ray, float t_min, float t_max, int8_t recursion_depth, const Scene& scene) {
    if (auto hit = scene.bvh->intersect(ray, t_min, t_max)) {
        Vec3 point = ray.origin + (ray.direction * hit->t);
        Vec3 view = ray.direction * -1;
        Color local_color = hit->object->color * computeLighting(point, hit->normal, view, hit->object->specular, scene);

        // if we hit the recursion limit or the object is not reflective, we're done
        float reflective = hit->object->reflective;
        if (recursion_depth <= 0 or reflective <= 0) {
            return local_color;
        }
        // compute the reflected color
        Vec3 vec_r = reflectRay(view, hit->normal);
        // adding a small bias to the point to fix 'shadow acne' effect on the yellow ball, which is so big it feels almost flat
        Ray reflection_ray = {point + (hit->normal * EPSILON), vec_r};
        Color reflected_color = traceRay(reflection_ray, EPSILON, INFINITY, recursion_depth - 1, scene);

        return (local_color * (1 - reflective))
            + (reflected_color * reflective);
    }
    return scene.background_color;
}

BMPColor bmpColor(const Color& color) {
    return {
        .red = static_cast<uint8_t>(round(clamp<float>(color.red * 255, 0, 255))),
        .green = static_cast<uint8_t>(round(clamp<float>(color.green * 255, 0, 255))),
        .blue = static_cast<uint8_t>(round(clamp<float>(color.blue * 255, 0, 255)))
    };
}

std::vector<const Object*> objectsPointerView(const vector<unique_ptr<Object>>& objects) {
    vector<const Object*> view;
    view.reserve(objects.size());
    for (const auto& object : objects) {
        view.push_back(object.get());
    }
    return view;
}

int main() {
    Image image = {600, 600};

    Scene scene = {
        .viewport_size = 1,
        .projection_plane_z = 1,
        .camera_position = {0, 0, 0},
        .background_color = {0, 0, 0},
        .lights = {
            Light{.type = AMBIENT, .intensity = 0.2},
            Light{.type = POINT, .intensity = 0.6, .position = {2, 1, 0}},
            Light{.type = DIRECTIONAL, .intensity = 0.2, .position = {1, 4, 4}}
        },
    };
    scene.objects.reserve(4);
    scene.objects.emplace_back(make_unique<Sphere>(Vec3{0, -1, 3}, 1, 500, 0.2, Color{1, 0, 0}));
    scene.objects.emplace_back(make_unique<Sphere>(Vec3{-2, 0, 4}, 1, 10, 0.4, Color{0, 1, 0}));
    scene.objects.emplace_back(make_unique<Sphere>(Vec3{2, 0, 4}, 1, 500, 0.3, Color{0, 0, 1}));
    scene.objects.emplace_back(make_unique<Sphere>(Vec3{0, -5001, 0}, 5000, 1000, 0.5, Color{1, 1, 0}));
    // scene is owning objects and a BVH tree, while BVH is only keeping pointers to the objects
    scene.bvh = BVHNode::build(objectsPointerView(scene.objects));

    Camera camera = {
        .position = {3, 0, 1},
        .rotation = {
            {0.7071, 0, -0.7071},
            {     0, 1,       0},
            {0.7071, 0,  0.7071}
        }
    };

    for (int32_t x = -image.size.width / 2; x < image.size.width / 2; x++) {
        for(int32_t y = -image.size.height / 2; y < image.size.height / 2; y++) {
            Vec3 direction = camera.rotation * canvasToViewport(x, y, image.size, scene);
            Ray camera_ray = {camera.position, direction};
            Color color = traceRay(camera_ray, 1, INFINITY, 3, scene);
            image.data[image.offset(x, y)] = bmpColor(color);
        }
    }

    save_image(image.data, image.size.width, image.size.height, "results/07-optimizations.bmp");

    return 0;
}
