// Shadows
//
// Book chapter: https://gabrielgambetta.com/computer-graphics-from-scratch/05-extending-the-raytracer.html#constructive-solid-geometry
// Book example: -
//
// ```
// clang++ -std=c++17 examples/09-constructive-solid-geometry.cc -o bin/09-constructive-solid-geometry -O3 -fno-fast-math
// bin/09-constructive-solid-geometry
// ```

#include "bmp.h"
#include "bvh-csg.h"
#include <chrono>

const float EPSILON = 0.001;
const bool SUBSAMPLING = false;

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

struct Sphere : Shape {
    Vec3 center;
    float radius;

    Sphere(const Vec3& c, float r)
        : center(c), radius(r) {}

    AABB bounds() const {
        return AABB{
            center - Vec3{radius, radius, radius},
            center + Vec3{radius, radius, radius}
        };
    }

    Vec3 centroid() const {
        return center;
    }

    optional<ShapeHit> intersect(const Ray& ray, float t_min, float t_max) const {
        Vec3 oc = ray.origin - center;

        float a = ray.direction.dot(ray.direction);
        float b = 2 * oc.dot(ray.direction);
        float c = oc.dot(oc) - (radius * radius);

        float discriminant = (b * b) - (4 * a * c);
        if (discriminant < 0) {
            return nullopt;
        }

        float sqrt_d = sqrt(discriminant);
        float closest_t = (-b - sqrt_d) / (2 * a);
        if (closest_t < t_min || closest_t > t_max) {
            closest_t = (-b + sqrt_d) / (2 * a);
            if (closest_t < t_min || closest_t > t_max) {
                return nullopt;
            }
        }

        Vec3 hit_point = ray.at(closest_t);
        return ShapeHit{
            .point = hit_point,
            .normal = (hit_point - center).normalized(),
            .t = closest_t
        };
    }
};

struct Triangle : Shape {
    Vec3 a, b, c;

    Triangle(const Vec3& a, const Vec3& b, const Vec3& c)
        : a(a), b(b), c(c) {}

    AABB bounds() const {
        // adding epsilon to the equation to cover for floating point errors
        Vec3 epsilon = {EPSILON, EPSILON, EPSILON};
        return AABB{
            .min = Vec3{std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::min({a.z, b.z, c.z})} - epsilon,
            .max = Vec3{std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}), std::max({a.z, b.z, c.z})} + epsilon
        };
    }

    Vec3 centroid() const {
        return (a + b + c) * (1/3);
    }

    optional<ShapeHit> intersect(const Ray& ray, float t_min, float t_max) const {
        // Möller-Trumbore algorithm
        Vec3 edge_1 = b - a;
        Vec3 edge_2 = c - a;
        // perpendicular vector
        Vec3 p_vec = ray.direction.cross(edge_2);
        float determinant = edge_1.dot(p_vec);
        // using abs for no culling
        if (fabs(determinant) < EPSILON) {
            return nullopt;
        }
        float inv_determinant = 1.0f / determinant;
        // translation vector
        Vec3 t_vec = ray.origin - a;
        // calculate U
        float u = t_vec.dot(p_vec) * inv_determinant;
        if (u < 0.0f || u > 1.0f) return std::nullopt;
        // quadrilateral|cross helper
        Vec3 q_vec = t_vec.cross(edge_1);
        float v = ray.direction.dot(q_vec) * inv_determinant;
        if (v < 0.0f || (u + v) > 1.0f) {
            return nullopt;
        }
        float t = edge_2.dot(q_vec) * inv_determinant;
        if (t < t_min || t > t_max) {
            return nullopt;
        }
        // plane normal is just a normalized cross product of two edges
        Vec3 normal = edge_1.cross(edge_2).normalized();
        // flip normal so it always opposes ray (because no-culling)
        if (normal.dot(ray.direction) > 0) {
            normal = normal * -1;
        }
        return ShapeHit{
            .point = ray.at(t),
            .normal = normal,
            .t = t
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
    vector<unique_ptr<Primitive>> objects;
    unique_ptr<BVHNode> bvh;
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
        Color local_color = hit->material->color * computeLighting(point, hit->normal, view, hit->material->specular, scene);

        // if we hit the recursion limit or the object is not reflective, we're done
        float reflective = hit->material->reflective;
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

vector<const Primitive*> objectsPointerView(const vector<unique_ptr<Primitive>>& objects) {
    vector<const Primitive*> view;
    view.reserve(objects.size());
    for (const auto& object : objects) {
        view.push_back(object.get());
    }
    return view;
}

unique_ptr<Primitive> makePrimitive(shared_ptr<Shape> shape, shared_ptr<Material> material) {
    // capture the shared_ptr by value to keep material alive
    return make_unique<Primitive>(
        std::move(shape),
        [material](const ShapeHit&) -> const Material& { return *material; }
    );
}

int main() {
    auto start = chrono::high_resolution_clock::now();

    Image image = {600, 600};

    Scene scene = {
        .viewport_size = 1,
        .projection_plane_z = 1,
        .camera_position = {0, 0, 0},
        .background_color = {0, 0, 0},
        .lights = {
            Light{.type = AMBIENT, .intensity = 0.2},
            Light{.type = POINT, .intensity = 0.6, .position = {2, 1, 0}},
            Light{.type = DIRECTIONAL, .intensity = 0.2, .position = {-1, 3, -4}}
        },
    };
    scene.objects.reserve(4);
    auto red_material = make_shared<Material>(Material{ .specular = 500, .reflective = 0.2, .color = Color{1, 0, 0} });
    auto green_material = make_shared<Material>(Material{ .specular = 10, .reflective = 0.4, .color = Color{0, 1, 0} });
    auto blue_material = make_shared<Material>(Material{ .specular = 500, .reflective = 0.3, .color = Color{0, 0, 1} });
    auto yellow_material = make_shared<Material>(Material{ .specular = 1000, .reflective = 0.5, .color = Color{1, 1, 0} });

    scene.objects.emplace_back(makePrimitive(make_shared<Sphere>(Vec3{0.5, -0.5, 0}, 1), red_material));
    scene.objects.emplace_back(makePrimitive(make_shared<Sphere>(Vec3{-7, 0, 4}, 1), green_material));
    scene.objects.emplace_back(makePrimitive(make_shared<Sphere>(Vec3{2, 2, 4}, 1.5), blue_material));
    scene.objects.emplace_back(makePrimitive(make_shared<Sphere>(Vec3{0, -5001, 0}, 5000), yellow_material));

    auto tri_material = make_shared<Material>(Material{ .specular = 420, .reflective = 0.5, .color = Color{1, 0.2, 1} });
    scene.objects.emplace_back(makePrimitive(make_shared<Triangle>(Vec3{-6, -1.1, 4}, Vec3{-3, -1.1, -1}, Vec3{-2.5, 1.5, -1.5}), tri_material));
    scene.objects.emplace_back(makePrimitive(make_shared<Triangle>(Vec3{-3, -1.1, -1}, Vec3{0, -1.1, 4}, Vec3{-2.5, 1.5, -1.5}), tri_material));
    scene.objects.emplace_back(makePrimitive(make_shared<Triangle>(Vec3{0, -1.1, 4}, Vec3{-6, -1.1, 4}, Vec3{-2.5, 1.5, -1.5}), tri_material));

    // scene is owning objects and a BVH tree, while BVH is only keeping pointers to the objects
    scene.bvh = BVHNode::build(objectsPointerView(scene.objects));

    Camera camera = {
        .position = {-2, 0, -9},
        .rotation = {
            {1,0,0},
            {0,1,0},
            {0,0,1}
        }
    };

    if (SUBSAMPLING) {
        int32_t start_x = -image.size.width / 2;
        int32_t start_y = -image.size.height / 2;

        int32_t end_x = image.size.width / 2;
        int32_t end_y = image.size.height / 2;

        // skipping every other pixel to reduce ray tracing computations
        for (int32_t x = start_x; x < end_x; x += 2) {
            for(int32_t y = start_y; y < end_y; y += 2) {
                Vec3 direction = camera.rotation * canvasToViewport(x, y, image.size, scene);
                Ray camera_ray = {camera.position, direction};

                image.data[image.offset(x, y)] = traceRay(camera_ray, 1, INFINITY, 3, scene);
            }
        }
        // now we need to fill in missing colors by moving through every other pixel,
        // the way we iterate through the image, you can assume that the result of the first pass is four corner colors,
        // now we're doing a second pass through every other pixel to fill in 5 missing colors;
        //
        // however since we're moving from bottom-left to top-right corner,
        // it means that we can skip calculating top and right pixel colors,
        // since they'll be calculated as left and bottom colors on the next iteration around another center pixel
        // ┌─────┐
        // │■ △ ■│
        // │◁ ◇ ▷│
        // │■ ▽ ■│
        // └─────┘
        for (int32_t x = start_x + 1; x < end_x; x += 2) {
            for(int32_t y = start_y + 1; y < end_y; y += 2) {
                int32_t left_x = x - 1;
                int32_t bottom_y = y - 1;
                // since we're moving from bottom-left corner to the upper-right one,
                // we need to make sure we're not getting outside of top and right borders
                bool at_top_border = y == end_y - 1;
                bool at_right_border = x == end_x - 1;
                // reflecting x and y coordinates beyond image borders
                int32_t right_x = at_right_border ? left_x : x + 1;
                int32_t top_y = at_top_border ? bottom_y : y + 1;

                Color top_left_color = image.data[image.offset(left_x, top_y)];
                Color top_right_color = image.data[image.offset(right_x, top_y)];
                Color bottom_left_color = image.data[image.offset(left_x, bottom_y)];
                Color bottom_right_color = image.data[image.offset(right_x, bottom_y)];
                // only calculating top and right colors when we're at the corresponding image borders
                if (at_top_border) {
                    image.data[image.offset(x, top_y)] = (top_left_color + top_right_color) * 0.5;
                }
                if (at_right_border) {
                    image.data[image.offset(right_x, y)] = (bottom_right_color + top_right_color) * 0.5;
                }
                image.data[image.offset(x, bottom_y)] = (bottom_left_color + bottom_right_color) * 0.5;
                image.data[image.offset(left_x, y)] = (top_left_color + bottom_left_color) * 0.5;
                image.data[image.offset(x, y)] = (top_left_color + top_right_color + bottom_left_color + bottom_right_color) * 0.25;
            }
        }
        image.data[image.offset(start_x, start_y)] = {1,1,1};
    } else {
        for (int32_t x = -image.size.width / 2; x < image.size.width / 2; x++) {
            for(int32_t y = -image.size.height / 2; y < image.size.height / 2; y++) {
                Vec3 direction = camera.rotation * canvasToViewport(x, y, image.size, scene);
                Ray camera_ray = {camera.position, direction};
                Color color = traceRay(camera_ray, 1, INFINITY, 3, scene);
                image.data[image.offset(x, y)] = color;
            }
        }
    }

    vector<BMPColor> bmp_image = vector<BMPColor>(image.data.size());
    for (int i = 0; i < image.data.size(); i++) {
        bmp_image[i] = bmpColor(image.data[i]);
    }

    if (SUBSAMPLING) {
        save_image(bmp_image, image.size.width, image.size.height, "results/09-constructive-solid-geometry-subsampling.bmp");
    } else {
        save_image(bmp_image, image.size.width, image.size.height, "results/09-constructive-solid-geometry.bmp");
    }

    auto end = chrono::high_resolution_clock::now();
    cout << "Time spent: " << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " ms\n";

    return 0;
}
