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
};

// MARK: - Object and related

struct Color {
    float red, green, blue;
};

struct HitInfo;

struct Object {
    float specular;
    float reflective;
    Color color;

    virtual ~Object() = default;
    virtual AABB bounds() const = 0;
    virtual Vec3 centroid() const = 0;
    virtual optional<HitInfo> intersect(const Ray& ray, float t_min, float t_max) const = 0;
};

struct HitInfo {
    Vec3 point;
    Vec3 normal;
    float t;
    const Object* object;
};

// MARK: - Bounding Volume Hierarchy Tree

struct BVHNode {
    AABB bounds;
    unique_ptr<BVHNode> left = nullptr;
    unique_ptr<BVHNode> right = nullptr;
    // only in a leaf
    vector<const Object *> objects;

    static unique_ptr<BVHNode> build(vector<const Object *> objects, int max_leaf_size = 2, int depth = 0) {
        auto node = make_unique<BVHNode>();

        for (const Object *obj : objects) {
            node->bounds.expand(obj->bounds());
        }

        // leaf node if small enough
        if (objects.size() <= max_leaf_size) {
            node->objects = std::move(objects);
            return node;
        }

        Vec3::Axis splitAxis = node->bounds.longestAxis();
        sort(objects.begin(), objects.end(), [splitAxis](const Object* lhs, const Object* rhs) {
            return lhs->centroid()[splitAxis] < rhs->centroid()[splitAxis];
        });

        // split list in two halves
        int mid = objects.size() / 2;
        vector<const Object*> left_objs(objects.begin(), objects.begin() + mid);
        vector<const Object*> right_objs(objects.begin() + mid, objects.end());

        node->left = build(std::move(left_objs), max_leaf_size, depth + 1);
        node->right = build(std::move(right_objs), max_leaf_size, depth + 1);

        return node;
    }

    bool isLeaf() const {
        return !left && !right;
    }

    optional<HitInfo> intersect(const Ray& ray, float t_min, float t_max) const {
        // first, cull with bounding box
        if (!bounds.intersect(ray, t_min, t_max)) {
            return nullopt;
        }

        // leaf node - test all objects
        if (isLeaf()) {
            optional<HitInfo> closest_hit;
            float closest_t = t_max;

            for (const Object* obj : objects) {
                if (auto hit = obj->intersect(ray, t_min, closest_t)) {
                    // shrink max range
                    closest_t = hit->t;
                    closest_hit = std::move(hit);
                }
            }
            return closest_hit;
        }

        // parent node - test children
        optional<HitInfo> hit_left;
        optional<HitInfo> hit_right;
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
            for (const Object* obj : objects) {
                if (obj->intersect(ray, t_min, t_max))
                    return true;
            }
            return false;
        }

        return (left && left->intersectAnything(ray, t_min, t_max))
            || (right && right->intersectAnything(ray, t_min, t_max));
    }
};
