#pragma once

#include <Eigen/Core>
#include <tuple>
#include <array>

#include "./inside.hpp"

namespace SimulMan {

    struct Sphere;
    struct AABB;
    struct Ray;
    struct Segment;
    struct Plane;
    struct Triangle;

    //==========================================
    // closest
    // 점 -> 구, AABB, 평면, 세그먼트, 레이, 삼각형
    //==========================================
    using vec2 = Eigen::Vector2d;
    using vec3 = Eigen::Vector3d;
    using vec4 = Eigen::Vector4d;

    //✓ 점 -> 구
    vec3 closest(
        const vec3& p, const Sphere& s
    ) {
        return s.c + (p - s.c).normalized() * s.r;
    }
    vec3 closest(
        const Sphere& s, const vec3& p
    ) {
        return closest(p, s);
    }

    //✓ 점 -> AABB          RTCD 130
    vec3 closest(const vec3& p, const AABB& aabb) {
        vec3 result = p;
        result = result.array().max(aabb.min.array()); // result = max(result, b.min)
        result = result.array().min(aabb.max.array()); // result = min(result, b.max)
        return result;
    }
    vec3 closest(
        const AABB& aabb, const vec3& p
    ) {
        return closest(p, aabb);
    }
    //✓ 점 -> 평면       GPC pp171, RTCD 126
    vec3 closest(const vec3& p, const Plane& plane) {
        double t = (p.dot(plane.n) - plane.d) / plane.n.squaredNorm();
        return p - t * plane.n;
    }
    vec3 closest(
        const Plane& plane, const vec3& p
    ) {
        return closest(p, plane);
    }
    //✓ 점 -> 세그먼트           GPC pp173, RTCD 127
    vec3 closest(const vec3& p, const Segment& seg) {
        vec3 ab = seg.b - seg.a;
        double t = (p - seg.a).dot(ab) / ab.squaredNorm();
        t = std::clamp(t, 0.0, 1.0);
        return seg.a + t * ab;
    }
    vec3 closest(
        const Segment& seg, const vec3& p
    ) {
        return closest(p, seg);
    }
    // 점 -> 레이           GPC pp175
    vec3 closest(const vec3& p, const Ray& ray) {
        double t = (p - ray.p).dot(ray.n);
        t = std::max(t, 0.0);
        return ray.p + ray.n * t;
    }
    vec3 closest(
        const Ray& ray, const vec3& p
    ) {
        return closest(p, ray);
    }
    //★ 점 -> 삼각형           GPC pp227, RTCD 137 (3가지 버젼)
    bool is_inside(
        const vec3& p,
        const Triangle& tri
    );
    vec3 closest(
        const vec3& p,
        const Triangle& tri
    ) {
        Plane plane;
        plane.n = (tri.b - tri.a).cross(tri.c - tri.a).normalized();
        vec3 cloPt = closest(p, plane);
        if (
            is_inside(cloPt, tri)) {
            return cloPt;
        }

        vec3 c1 = closest(p, Segment(tri.a, tri.b ));
        vec3 c2 = closest(p, Segment(tri.b, tri.c ));
        vec3 c3 = closest(p, Segment(tri.c, tri.a ));

        double magSq1 = (p - c1).squaredNorm();
        double magSq2 = (p - c2).squaredNorm();
        double magSq3 = (p - c3).squaredNorm();

        if (magSq1 < magSq2 && magSq1 < magSq3) {
            return c1;
        }
        else if (magSq2 < magSq1 && magSq2 < magSq3) {
            return c2;
        }
        return c3;
    }

    //✓ 점 -> 사면체           RTCD 143
    // 간단한 방법은 사면체의 각 면 평면에 대해 closest point triangle
    // 함수를 한번씩 호출해 가장 가까운 점을 계산.
    // 계산된 모든 점들 중에 p에 가장 가까운 점이 반환
    // p가 모든 면 평면 내부에 있으면 p 자체가 가장 가까운 점
    vec3 closest(
        const vec3& p,
        const Tetrahedra& tet
    ) {
        vec3 cloPt = p;
        double bestSqDist = std::numeric_limits<double>::max();
        // ABC
        if ((tet.b - tet.a).cross(tet.c - tet.a).dot(p - tet.a) < 0.0) {
            vec3 q = closest(p, { tet.a, tet.b, tet.c });
            double sqDist = (q - p).squaredNorm();
            if (sqDist < bestSqDist) bestSqDist = sqDist, cloPt = q;
        }
        // ACD
        if ((tet.c - tet.a).cross(tet.d - tet.a).dot(p - tet.a) < 0.0) {
            vec3 q = closest(p, { tet.a, tet.c, tet.d });
            double sqDist = (q - p).squaredNorm();
            if (sqDist < bestSqDist) bestSqDist = sqDist, cloPt = q;
        }
        // ADB
        if ((tet.d - tet.a).cross(tet.b - tet.a).dot(p - tet.a) < 0.0) {
            vec3 q = closest(p, { tet.a, tet.d, tet.b });
            double sqDist = (q - p).squaredNorm();
            if (sqDist < bestSqDist) bestSqDist = sqDist, cloPt = q;
        }
        // BDC
        if ((tet.d - tet.b).cross(tet.c - tet.b).dot(p - tet.b) < 0.0) {
            vec3 q = closest(p, { tet.b, tet.d, tet.c });
            double sqDist = (q - p).squaredNorm();
            if (sqDist < bestSqDist) bestSqDist = sqDist, cloPt = q;
        }

        return cloPt;

    }



}

