#pragma once

#include <Eigen/Core>
#include <tuple>
#include <array>

#include "./closest.hpp"

namespace SimulMan {

    struct Sphere;
    struct AABB;
    struct Ray;
    struct Segment;
    struct Plane;

    //==========================================
    // is_inside
    // 점 in 구, AABB, 평면, 세그먼트, 레이, 삼각형
    //==========================================
    using vec2 = Eigen::Vector2d;
    using vec3 = Eigen::Vector3d;
    using vec4 = Eigen::Vector4d;

    //✓ 점 in 구
    bool is_inside(const vec3& p, const Sphere& s) {
        return (p - s.c).squaredNorm() < s.r * s.r;
    }
    bool is_inside(const Sphere& s, const vec3& p) {
        return is_inside(p, s);
    }
    // 점 in AABB
    bool is_inside(const vec3& p, const AABB& aabb) {
        if (p[0] < aabb.min[0] || p[1] < aabb.min[1] || p[2] < aabb.min[2]) return false;
        if (p[0] > aabb.max[0] || p[1] > aabb.max[1] || p[2] > aabb.max[2]) return false;
        return true;
    }
    bool is_inside(const AABB& aabb, const vec3& p) {
        return is_inside(p, aabb);
    }
    //✓ 점 in 평면       GPC pp171
    bool is_inside(const vec3& p, const Plane& plane) {
        double dot = p.dot(plane.n);
        return std::abs(dot - plane.d) <= 1.e-12;
    }
    bool is_inside(const Plane& plane, const vec3& p) {
        return is_inside(p, plane);
    }
    //✓ 점 in 세그먼트           GPC pp172
    vec3 closest(const vec3& p, const Segment& seg);
    bool is_inside(const vec3& p, const Segment& seg) {
        vec3 cloPt = closest(p, seg);
        return std::abs((p - cloPt).squaredNorm()) <= 1.e-12;
    }
    bool is_inside(const Segment& seg, const vec3& p) {
        return is_inside(p, seg);
    }
    // 점 in 레이           GPC pp174
    bool is_inside(const vec3& p, const Ray& ray) {
        vec3 norm = p - ray.p;
        if (norm.squaredNorm() < 1.e-12) return true;
        norm.normalize();
        double diff = norm.dot(ray.n);
        return std::abs(diff - 1.0) < 1.e-12;
    }
    bool is_inside(const Ray& ray, const vec3& p) {
        return is_inside(p, ray);
    }
    // 점 in 삼각형           GPC pp224, RTCD 203
    bool is_inside(
        const vec3& p,
        const Triangle& t
    ) {
        vec3 pa = t.a - p;
        vec3 pb = t.b - p;
        vec3 pc = t.c - p;

        vec3 normPBC = pb.cross(pc);
        vec3 normPCA = pc.cross(pa);
        vec3 normPAB = pa.cross(pb);

        if (normPBC.dot(normPCA) < 0.0) {
            return false;
        }
        else if (normPBC.dot(normPAB) < 0.0) {
            return false;
        }
        return true;

    }
    //★ 점 in 다각형           RTCD 201
    // 요르단 곡선 정리 사용.
    bool is_inside(
        const vec2& p,
        const std::vector<vec2>& vrts
    ) {
        double tx = p[0];
        double ty = p[1];
        bool inside = false;
        vec2 e0 = vrts.back();
        bool y0 = (e0[1] >= ty);
        for (int i = 0; i < vrts.size(); ++i) {
            vec2 e1 = vrts[i];
            bool y1 = (e1[1] >= ty);
            // 이전 꼭짓점과 현재 꼭짓점이 검사점의 y 좌표를 기준으로 서로 다른 쪽에 있는 경우
            if (y0 != y1) {
                // 검사점이 꼭짓점을 연결하는 선분의 왼쪽에 있는지 확인
                // (이 조건은 선분과 검사점을 지나는 수평선의 교차점을 계산하는 것과 동일)
                if (((e1[1] - ty) * (e0[0] - e1[0]) >= (e1[0] - tx) * (e0[1] - e1[1])) == y1) {
                    inside = !inside;
                }
            }
            y0 = y1;
            e0 = e1;
        }
        return inside;
    }

    // 점 in 다면체           RTCD 206
    // 점이 각 반공간 내부에 있다면 다면체 내부에 있는 것
    bool is_inside(
        const vec3& p,
        const std::vector<Plane>& planes
    ) {
        for (auto& plane : planes) {
            if (p.dot(plane.n) - plane.d > 0.0) return false;
        }
        return true;
    }
    //==========================================



}

