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

bool tInRange(float t, float t_min, float t_max) {
    return t > t_min && t <= t_max;
}

// MARK: - Sphere

struct Sphere : SolidShape {
    Vec3 center;
    float radius;

    Sphere(const Vec3& c, float r)
        : center(c), radius(r) {}

    AABB bounds() const override {
        return AABB{
            center - Vec3{radius, radius, radius},
            center + Vec3{radius, radius, radius}
        };
    }

    Vec3 centroid() const override {
        return center;
    }

    optional<ShapeHit> intersect(const Ray& ray, float t_min, float t_max) const override {
        Vec3 oc = ray.origin - center;
        float a = ray.direction.dot(ray.direction);
        float b = 2 * oc.dot(ray.direction);
        float c = oc.dot(oc) - (radius * radius);

        float discriminant = (b * b) - (4 * a * c);
        if (discriminant < 0) {
            return nullopt;
        }

        float a2 = a * 2;
        float sqrt_d = sqrt(discriminant);
        float t0 = (-b - sqrt_d) / a2;
        float t1 = (-b + sqrt_d) / a2;
        if (t0 > t1) {
            std::swap(t0, t1);
        }

        if (tInRange(t0, t_min, t_max)) {
            Vec3 point_enter = ray.at(t0);
            return ShapeHit{
                .point = point_enter,
                .normal = (point_enter - center).normalized(),
                .t = t0
            };
        }
        if (tInRange(t1, t_min, t_max)) {
            Vec3 point_exit = ray.at(t1);
            return ShapeHit{
                .point = point_exit,
                .normal = (point_exit - center).normalized(),
                .t = t1
            };
        }
        return nullopt;
    }
};

// MARK: - Triangle

struct Triangle : Shape {
    Vec3 a, b, c;

    Triangle(const Vec3& a, const Vec3& b, const Vec3& c)
        : a(a), b(b), c(c) {}

    AABB bounds() const override {
        // adding epsilon to the equation to cover for floating point errors
        Vec3 epsilon = Vec3::constant(EPSILON);
        return AABB{
            .min = Vec3{std::min({a.x, b.x, c.x}), std::min({a.y, b.y, c.y}), std::min({a.z, b.z, c.z})} - epsilon,
            .max = Vec3{std::max({a.x, b.x, c.x}), std::max({a.y, b.y, c.y}), std::max({a.z, b.z, c.z})} + epsilon
        };
    }

    Vec3 centroid() const override {
        return (a + b + c) * (1/3);
    }

    optional<ShapeHit> intersect(const Ray& ray, float t_min, float t_max) const override {
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
        if (u < 0.0f || u > 1.0f) return nullopt;
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

// MARK: - Constructive Solid Geometry

enum class CSGOperation {
    UNION, INTERSECTION, DIFFERENCE
};

struct CSGShape : SolidShape {
    shared_ptr<SolidShape> left, right;
    CSGOperation operation;

public:
    static shared_ptr<CSGShape> make(CSGOperation op, shared_ptr<SolidShape> lhs, shared_ptr<SolidShape> rhs) {
        if (!lhs && !rhs) {
            return nullptr;
        }
        auto shape = shared_ptr<CSGShape>(
            new CSGShape(op, std::move(lhs), std::move(rhs))
        );
        if (shape->bounds().isValid()) {
            return shape;
        }
        return nullptr;
    }

    AABB bounds() const override {
        switch (operation) {
            case CSGOperation::UNION:
                return paddingAABB(unionAABB(left->bounds(), right->bounds()));
            case CSGOperation::INTERSECTION:
                return paddingAABB(intersectionAABB(left->bounds(), right->bounds()));
            case CSGOperation::DIFFERENCE:
                return paddingAABB(left->bounds());
        }
    }

    Vec3 centroid() const override {
        return bounds().centroid();
    }

    //         t0          t1   t2          t3
    //     L ───┼████████████████┼───────────┼───
    //
    //     R ───┼───────────┼████████████████┼───
    //
    // L ∪ R ───┼████████████████████████████┼───
    //
    // L ∩ R ───┼───────────┼████┼───────────┼───
    //
    // L - R ───┼███████████┼────┼───────────┼───
    //
    optional<ShapeHit> intersect(const Ray& ray, float t_min, float t_max) const override {
        // implementing csg intersection using event-walking (events = hits)
        // by looping through hits and moving current min_t there everytime we have a hit,
        // this way we don't overwhelm `intersect` function with various number of returned hits
        // (i.e. sphere has two hits, while complex csg might have more than two)

        // current_t will serve as a cursor to aid in our event walking, and we start at t_min
        float current_t = t_min;
        // get the first events from each child
        auto l_next = left->intersect(ray, current_t, t_max);
        auto r_next = right->intersect(ray, current_t, t_max);
        // if the first event we can see is an EXIT, we probably start inside that child
        bool in_l = l_next && !l_next->isEnter(ray);
        bool in_r = r_next && !r_next->isEnter(ray);
        bool in_set = CSGShape::truth(operation, in_l, in_r);

        while (true) {
            // no hits for neither shape
            if (!l_next && !r_next) {
                return nullopt;
            }
            // choose nearest event
            bool from_left;
            ShapeHit hit;
            // if ray is either going through only L shape, or L is in front of R -> we pick L, otherwise -> R
            if (l_next && (!r_next || l_next->t <= r_next->t - EPSILON)) {
                hit = *l_next;
                from_left = true;
            } else {
                hit = *r_next;
                from_left = false;
            }
            // now after crossing the hit.t boundary, we update the child's state whether we're inside the shape or outside
            bool is_enter = hit.isEnter(ray);
            if (from_left) {
                in_l = is_enter;
            } else {
                in_r = is_enter;
            }
            bool new_in_set = CSGShape::truth(operation, in_l, in_r);
            // here we return a matching hit if we're entering a set (toggling from false -> true)
            if (!in_set && new_in_set) {
                // for L - R, if visibility came from the RIGHT child's EXIT, flip the normal
                // (imaging an apple bite where the inside of the bite was formed by another sphere)
                if (operation == CSGOperation::DIFFERENCE && !from_left) {
                    hit.flipNormal();
                }
                // making sure hit is within [t_min, t_max] range
                if (tInRange(hit.t, t_min, t_max)) {
                    return hit;
                }
                return nullopt;
            }
            // advance the cursor and our in_set state
            current_t = hit.t;
            in_set = new_in_set;
            // only refill the side we consumed, keep the other side cached for the next loop iteration,
            // using epsilon to avoid floating point issues
            float next_t_min = std::min(current_t + EPSILON, t_max);
            if (from_left) {
                l_next.reset();
                l_next = left->intersect(ray, next_t_min, t_max);
            } else {
                r_next.reset();
                r_next = right->intersect(ray, next_t_min, t_max);
            }
        }
    }

private:
    CSGShape(CSGOperation op, shared_ptr<SolidShape> lhs, shared_ptr<SolidShape> rhs)
        : operation(op), left(std::move(lhs)), right(std::move(rhs)) {}

    static AABB unionAABB(const AABB& left, const AABB& right) {
        if (!left.isValid()) {
            return right;
        } else if (!right.isValid()) {
            return left;
        }
        return AABB{
            Vec3::min(left.min, right.min),
            Vec3::max(left.max, right.max)
        };
    }

    static AABB intersectionAABB(const AABB& left, const AABB& right) {
        if (!left.isValid() || !right.isValid()) {
            return AABB::invalid();
        }
        Vec3 min = Vec3::max(left.min, right.min);
        Vec3 max = Vec3::min(left.max, right.max);
        if (max.x < min.x || max.y < min.y || max.z < min.z) {
            return AABB::invalid();
        }
        return AABB{min, max};
    }

    static AABB paddingAABB(const AABB& aabb) {
        if (!aabb.isValid()) {
            return aabb;
        }
        Vec3 epsilon = Vec3::constant(EPSILON);
        return AABB{ aabb.min - epsilon, aabb.max + epsilon};
    }

    static bool truth(CSGOperation op, bool lhs, bool rhs) {
        switch (op) {
            case CSGOperation::UNION: return lhs || rhs;
            case CSGOperation::INTERSECTION: return lhs && rhs;
            case CSGOperation::DIFFERENCE: return lhs && !rhs;
        }
    }
};

// MARK: - Light, Camera, Scene

enum class LightType {
    AMBIENT, POINT, DIRECTIONAL
};

struct Light {
    Vec3 position;
    float intensity;
    LightType type;
};

struct Mat33 {
    Vec3 a, b, c;

    Vec3 operator*(const Vec3& vector) const {
        return { a.dot(vector), b.dot(vector), c.dot(vector) };
    }
};

struct Camera {
    Vec3 position;
    Mat33 rotation;
};

struct Scene {
    const float viewport_size = 1;
    const float projection_plane_z = 1;
    Color background_color;
    vector<Light> lights;
    vector<unique_ptr<Primitive>> objects;
    unique_ptr<BVHNode> bvh;
};

// MARK: - Image + Scene

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

// converts 2D canvas coordinates to 3D viewport coordinates.
Vec3 canvasToViewport(int32_t x, int32_t y, const ImageSize& size, const Scene& scene) {
    return Vec3 {
        (float)x * scene.viewport_size / size.width,
        (float)y * scene.viewport_size / size.height,
        scene.projection_plane_z
    };
}

// MARK: - Color Extensions

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

// MARK: - Raytracing

Vec3 reflectRay(const Vec3& ray, const Vec3& normal) {
    return normal * 2.0f * normal.dot(ray) - ray;
}

float computeLighting(const Vec3& point, const Vec3& normal, const Vec3& view, float specular, const Scene& scene) {
    float intensity = 0;

    for (const Light& light : scene.lights) {
        if (light.type == LightType::AMBIENT) {
            intensity += light.intensity;
        } else {
            Vec3 vec_l = {0, 0, 0};
            float shadow_t_max;

            if (light.type == LightType::POINT) {
                vec_l = light.position - point;
                shadow_t_max = 1;
            } else if (light.type == LightType::DIRECTIONAL) {
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

// MARK: - Helpers

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

// MARK: - Main

int main() {
    auto start = chrono::high_resolution_clock::now();

    Image image = {600, 600};

    Scene scene = {
        .viewport_size = 1,
        .projection_plane_z = 1,
        .background_color = {0, 0, 0},
        .lights = {
            Light{.type = LightType::AMBIENT, .intensity = 0.2},
            Light{.type = LightType::POINT, .intensity = 0.6, .position = {2, 1, 0}},
            Light{.type = LightType::DIRECTIONAL, .intensity = 0.2, .position = {-1, 3, -4}}
        },
    };
    scene.objects.reserve(9);
    auto red_material = make_shared<Material>(Material{ .specular = 500, .reflective = 0.2, .color = Color{1, 0, 0} });
    auto green_material = make_shared<Material>(Material{ .specular = 10, .reflective = 0.4, .color = Color{0, 1, 0} });
    auto blue_material = make_shared<Material>(Material{ .specular = 500, .reflective = 0.3, .color = Color{0, 0, 1} });
    auto yellow_material = make_shared<Material>(Material{ .specular = 1000, .reflective = 0.5, .color = Color{1, 1, 0} });

    // few CSGs
    auto cheese_csg1 = CSGShape::make(CSGOperation::DIFFERENCE, make_shared<Sphere>(Vec3{0, -501, 0}, 500), make_shared<Sphere>(Vec3{-1, -1.25, -1}, 1.5));
    auto cheese_csg2 = CSGShape::make(CSGOperation::DIFFERENCE, cheese_csg1, make_shared<Sphere>(Vec3{-3, -1, -6}, 0.4));
    scene.objects.emplace_back(makePrimitive(cheese_csg2, yellow_material));

    scene.objects.emplace_back(makePrimitive(
        CSGShape::make(CSGOperation::DIFFERENCE, make_shared<Sphere>(Vec3{0, 3, 4}, 2), make_shared<Sphere>(Vec3{0, 3, 2.5}, 1)),
        blue_material
    ));
    scene.objects.emplace_back(makePrimitive(
        CSGShape::make(CSGOperation::UNION, make_shared<Sphere>(Vec3{-7, 0, 4}, 1), make_shared<Sphere>(Vec3{-6, 1, 3.7}, 0.5)),
        green_material
    ));
    scene.objects.emplace_back(makePrimitive(
        CSGShape::make(CSGOperation::INTERSECTION, make_shared<Sphere>(Vec3{-5.5, 1.1, 3.7}, 0.25), make_shared<Sphere>(Vec3{-5.5, 0.9, 3.7}, 0.25)),
        yellow_material
    ));

    // regular spheres
    scene.objects.emplace_back(makePrimitive(make_shared<Sphere>(Vec3{0.5, -0.5, 0}, 1), red_material));
    scene.objects.emplace_back(makePrimitive(make_shared<Sphere>(Vec3{-7, 0, 4}, 1), green_material));

    // triangle
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
