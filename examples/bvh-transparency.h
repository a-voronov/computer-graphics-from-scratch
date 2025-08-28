#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>

using namespace std;

// MARK: - XYZ Vector

struct Vec3 {
    float x, y, z;

    enum class Axis { x, y, z };

    Vec3 operator+(const Vec3& other) const {
        return { x + other.x, y + other.y, z + other.z };
    }

    Vec3 operator-(const Vec3& other) const {
        return { x - other.x, y - other.y, z - other.z };
    }

    Vec3 operator*(float n) const {
        return { x * n, y * n, z * n };
    }

    const float operator[] (const Axis& axis) const {
        switch(axis) {
            case Axis::x: return x;
            case Axis::y: return y;
            case Axis::z: return z;
        }
    }

    float dot(const Vec3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vec3 cross(const Vec3& other) const {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    float length() const {
        return sqrt(this->dot(*this));
    }

    Vec3 normalized() const {
        return *this * (1.0 / this->length());
    }

    static Vec3 min(const Vec3& lhs, const Vec3& rhs) {
        return Vec3{std::min(lhs.x, rhs.x), std::min(lhs.y, rhs.y), std::min(lhs.z, rhs.z)};
    }

    static Vec3 max(const Vec3& lhs, const Vec3& rhs) {
        return Vec3{std::max(lhs.x, rhs.x), std::max(lhs.y, rhs.y), std::max(lhs.z, rhs.z)};
    }

    static Vec3 constant(float value) {
        return Vec3{value, value, value};
    }
};

const Vec3::Axis VEC3AXES[3] = {Vec3::Axis::x, Vec3::Axis::y, Vec3::Axis::z};

struct Ray {
    Vec3 origin;
    // normalized
    Vec3 direction;

    // Computes a point along the ray at distance `t`
    Vec3 at(float t) const {
        return origin + direction * t;
    }
};

// MARK: - Axis-Aligned Bounding Box

struct AABB {
    Vec3 min, max;

    Vec3 centroid() const {
        return (min + max) * 0.5f;
    }

    void expand(const AABB& other) {
        min = Vec3::min(min, other.min);
        max = Vec3::max(max, other.max);
    }

    Vec3::Axis longestAxis() const {
        Vec3 size = max - min;
        if (size.x > size.y && size.x > size.z)
            return Vec3::Axis::x;
        if (size.y > size.z)
            return Vec3::Axis::y;
        return Vec3::Axis::z;
    }

    bool intersect(const Ray& ray, float t_min, float t_max) const {
        for (Vec3::Axis axis : VEC3AXES) {
            float origin = ray.origin[axis];
            float direction = ray.direction[axis];
            float inv_direction = 1.0f / direction;

            float t0 = (min[axis] - origin) * inv_direction;
            float t1 = (max[axis] - origin) * inv_direction;
            if (inv_direction < 0.0f) {
                std::swap(t0, t1);
            }

            t_min = std::max(t_min, t0);
            t_max = std::min(t_max, t1);
            if (t_max < t_min) {
                return false;
            }
        }
        return true;
    }

    bool isValid() const {
        return min.x <= max.x
            && min.y <= max.y
            && min.z <= max.z;
    }

    static AABB invalid() {
        return AABB{
            .min = Vec3::constant(INFINITY),
            .max = Vec3::constant(-INFINITY)
        };
    }
};

// MARK: - Shape + Hit

struct ShapeHit {
    Vec3 point;
    Vec3 normal;
    float t;

    bool isEnter(const Ray& ray) const {
        // entering if normal is facing against ray's direction
        return normal.dot(ray.direction) < 0.0f;
    }

    void flipNormal() {
        normal = normal * -1;
    }
};

struct Shape {
    virtual ~Shape() = default;
    virtual AABB bounds() const = 0;
    virtual Vec3 centroid() const = 0;
    virtual optional<ShapeHit> intersect(const Ray& ray, float t_min, float t_max) const = 0;
};

struct SolidShape : Shape {};

// MARK: - Color & Material

struct Color {
    float red, green, blue;
};

struct Material {
    float specular;
    float reflective;
    float transparent = 0;
    float refraction = 1;
    Color color;
};

// MARK: - Primitive + Hit

struct Primitive;

struct PrimitiveHit : ShapeHit {
    const Primitive* primitive = nullptr;
    const Material* material  = nullptr;

    PrimitiveHit(const ShapeHit& hit, const Primitive* primitive, const Material* material)
        : ShapeHit(hit), primitive(primitive), material(material) {}
};

struct Primitive {
    shared_ptr<Shape> shape;
    function<const Material&(const ShapeHit&)> material;

    Primitive(shared_ptr<Shape> shape, function<const Material&(const ShapeHit&)> material)
        : shape(std::move(shape)), material(std::move(material)) {}

    AABB bounds() const {
        return shape->bounds();
    }

    virtual Vec3 centroid() const {
        return shape->centroid();
    }

    optional<PrimitiveHit> intersect(const Ray& ray, float t_min, float t_max) const {
        if (auto s_hit = shape->intersect(ray, t_min, t_max)) {
            return PrimitiveHit(*s_hit, this, &material(*s_hit));
        }
        return nullopt;
    }
};

// MARK: - Bounding Volume Hierarchy Tree

// This implementation is mostly for demo purposes and own exploration,
// a better explanation of how to build a BVH tree can be found in the series of articles here:
// https://jacco.ompf2.com/2022/04/13/how-to-build-a-bvh-part-1-basics/

struct BVHNode {
    AABB bounds;
    unique_ptr<BVHNode> left = nullptr;
    unique_ptr<BVHNode> right = nullptr;
    // only in a leaf
    vector<const Primitive *> objects;

    static unique_ptr<BVHNode> build(vector<const Primitive *> objects, int max_leaf_size = 2, int depth = 0, int max_depth = 8) {
        auto node = make_unique<BVHNode>();

        for (const Primitive *obj : objects) {
            node->bounds.expand(obj->bounds());
        }

        // leaf node if small enough
        if (objects.size() <= max_leaf_size || depth >= max_depth) {
            node->objects = std::move(objects);
            return node;
        }

        Vec3::Axis splitAxis = node->bounds.longestAxis();
        sort(objects.begin(), objects.end(), [splitAxis](const Primitive* lhs, const Primitive* rhs) {
            return lhs->centroid()[splitAxis] < rhs->centroid()[splitAxis];
        });

        // split list in two halves
        int mid = objects.size() / 2;
        vector<const Primitive*> left_objs(objects.begin(), objects.begin() + mid);
        vector<const Primitive*> right_objs(objects.begin() + mid, objects.end());

        node->left = build(std::move(left_objs), max_leaf_size, depth + 1);
        node->right = build(std::move(right_objs), max_leaf_size, depth + 1);

        return node;
    }

    bool isLeaf() const {
        return !left && !right;
    }

    optional<PrimitiveHit> intersect(const Ray& ray, float t_min, float t_max) const {
        // first, cull with bounding box
        if (!bounds.intersect(ray, t_min, t_max)) {
            return nullopt;
        }

        // leaf node - test all objects
        if (isLeaf()) {
            optional<PrimitiveHit> closest_hit;
            float closest_t = t_max;

            for (const Primitive* obj : objects) {
                if (auto hit = obj->intersect(ray, t_min, closest_t)) {
                    // shrink max range
                    closest_t = hit->t;
                    closest_hit = std::move(hit);
                }
            }
            return closest_hit;
        }

        // parent node - test children
        optional<PrimitiveHit> hit_left;
        optional<PrimitiveHit> hit_right;
        float closest_t = t_max;

        if (left) {
            if (auto hit = left->intersect(ray, t_min, closest_t)) {
                // shrink max range
                closest_t = hit->t;
                hit_left = std::move(hit);
            }
        }
        if (right) {
            if (auto hit = right->intersect(ray, t_min, closest_t)) {
                // shrink max range
                closest_t = hit->t;
                hit_right = std::move(hit);
            }
        }

        // return closer of the two hits
        if (hit_left && hit_right) {
            return (hit_left->t < hit_right->t)
                ? hit_left
                : hit_right;
        } else if (hit_left) {
            return hit_left;
        } else {
            return hit_right;
        }
    }

    bool intersectAnything(const Ray& ray, float t_min, float t_max) const {
        if (!bounds.intersect(ray, t_min, t_max)) {
            return false;
        }

        if (isLeaf()) {
            for (const Primitive* obj : objects) {
                if (obj->intersect(ray, t_min, t_max))
                    return true;
            }
            return false;
        }

        return (left && left->intersectAnything(ray, t_min, t_max))
            || (right && right->intersectAnything(ray, t_min, t_max));
    }
};
