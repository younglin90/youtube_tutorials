#pragma once

#include <Eigen/Core>
#include <tuple>
#include <array>

#include "./predicates.h"

/*

RTCD : real-time collision detection book
GPC : game physics cookbook book
RTR : real-time rendering

*/
class GeometryPrimer {
public:
    using vec2 = Eigen::Vector2d;
    using vec3 = Eigen::Vector3d;
    using vec4 = Eigen::Vector4d;

	struct AABB {
		vec3 min{}, max{};
	};

	struct Sphere {
		vec3 c{};
		double r{};
	};

    struct Plane {
        // xyz = normal, w = constant, plane: x . normal + constant = 0
        vec3 n{};
        double c{};
    };

    struct Segment {
        vec3 a{}, b{};
    };

    struct Ray {
        vec3 p{}, n{};
    };

    struct Interval {
        double min{}, max{};
    };



    //==========================================
    // closest
    // 점 -> 구, AABB, 평면, 세그먼트, 레이, 삼각형
    //==========================================
    //✓ 점 -> 구
    vec3 closest(
        const vec3& p, const Sphere& s
    ) {
        return s.c + (p - s.c).normalized() * s.r;
    }
    //✓ 점 -> AABB          RTCD 130
    vec3 closest(const vec3& p, const AABB& aabb) {
        vec3 result = p;
        result = result.array().max(aabb.min.array()); // result = max(result, b.min)
        result = result.array().min(aabb.max.array()); // result = min(result, b.max)
        return result;
    }
    //✓ 점 -> 평면       GPC pp171, RTCD 126
    vec3 closest(const vec3& p, const Plane& plane) {
        double t = (p.dot(plane.n) - plane.c) / plane.n.squaredNorm();
        return p - t * plane.n;
    }
    //✓ 점 -> 세그먼트           GPC pp173, RTCD 127
    vec3 closest(const vec3& p, const Segment& seg) {
        vec3 ab = seg.b - seg.a;
        double t = (p - seg.a).dot(ab) / ab.squaredNorm();
        t = std::clamp(t, 0.0, 1.0);
        return seg.a + t * ab;
    }
    // 점 -> 레이           GPC pp175
    vec3 closest(const vec3& p, const Ray& ray) {
        double t = (p - ray.p).dot(ray.n);
        t = std::max(t, 0.0);
        return ray.p + ray.n * t;
    }
    //★ 점 -> 삼각형           GPC pp227, RTCD 137 (3가지 버젼)
    vec3 closest(
        const vec3& p,
        const vec3& a, const vec3& b, const vec3& c
    ) {
        Plane plane;
        plane.n = (b - a).cross(c - a).normalized();
        vec3 cloPt = closest(p, plane);
        if (
            is_inside(cloPt, a, b, c)) {
            return cloPt;
        }

        vec3 c1 = closest(p, Segment{ a, b });
        vec3 c2 = closest(p, Segment{ b, c });
        vec3 c3 = closest(p, Segment{ c, a });

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
        const vec3& a, const vec3& b, const vec3& c, const vec3& d
    ) {
        vec3 cloPt = p;
        double bestSqDist = std::numeric_limits<double>::max();
        // ABC
        if ((b - a).cross(c - a).dot(p - a) < 0.0) {
            vec3 q = closest(p, a, b, c);
            double sqDist = (q - p).squaredNorm();
            if (sqDist < bestSqDist) bestSqDist = sqDist, cloPt = q;
        }
        // ACD
        if ((c - a).cross(d - a).dot(p - a) < 0.0) {
            vec3 q = closest(p, a, c, d);
            double sqDist = (q - p).squaredNorm();
            if (sqDist < bestSqDist) bestSqDist = sqDist, cloPt = q;
        }
        // ADB
        if ((d - a).cross(b - a).dot(p - a) < 0.0) {
            vec3 q = closest(p, a, d, b);
            double sqDist = (q - p).squaredNorm();
            if (sqDist < bestSqDist) bestSqDist = sqDist, cloPt = q;
        }
        // BDC
        if ((d - b).cross(c - b).dot(p - b) < 0.0) {
            vec3 q = closest(p, b, d, c);
            double sqDist = (q - p).squaredNorm();
            if (sqDist < bestSqDist) bestSqDist = sqDist, cloPt = q;
        }

        return cloPt;

    }




    //==========================================
    // is_inside
    // 점 in 구, AABB, 평면, 세그먼트, 레이, 삼각형
    //==========================================
    //✓ 점 in 구
    bool is_inside(const vec3& p, const Sphere& s) {
        return (p - s.c).norm() < s.r * s.r;
    }
    // 점 in AABB
    bool is_inside(const vec3& p, const AABB& aabb) {
        if (p[0] < aabb.min[0] || p[1] < aabb.min[1] || p[2] < aabb.min[2]) return false;
        if (p[0] > aabb.max[0] || p[1] > aabb.max[1] || p[2] > aabb.max[2]) return false;
        return true;
    }
    //✓ 점 in 평면       GPC pp171
    bool is_inside(const vec3& p, const Plane& plane) {
        double dot = p.dot(plane.n);
        return std::abs(dot - plane.c) <= 1.e-12;
    }
    //✓ 점 in 세그먼트           GPC pp172
    bool is_inside(const vec3& p, const Segment& seg) {
        vec3 cloPt = closest(p, seg);
        return std::abs((p - cloPt).squaredNorm()) <= 1.e-12;
    }
    // 점 in 레이           GPC pp174
    bool is_inside(const vec3& p, const Ray& ray) {
        vec3 norm = p - ray.p;
        if (norm.squaredNorm() < 1.e-12) return true;
        norm.normalize();
        double diff = norm.dot(ray.n);
        return std::abs(diff - 1.0) < 1.e-12;
    }
    // 점 in 삼각형           GPC pp224, RTCD 203
    bool is_inside(
        const vec3& p, 
        const vec3& a, const vec3& b, const vec3& c
    ) {
        vec3 pa = a - p;
        vec3 pb = b - p;
        vec3 pc = c - p;

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
            if (p.dot(plane.n) - plane.c > 0.0) return false;
        }
        return true;
    }
    //==========================================



    //==========================================
    // 특수한 경우
    //==========================================
    // 2평면          RTCD 207
    std::pair<bool, Ray> intersect_two_planes(
        const Plane& plane0,
        const Plane& plane1 
    ) {
        Ray ray;
        ray.n = plane0.n.cross(plane1.n);

        if(ray.n.squaredNorm() < 1.e-12)
            return std::make_pair(false, ray);

        double d11 = plane0.n.squaredNorm();
        double d12 = plane0.n.dot(plane1.n);
        double d22 = plane1.n.squaredNorm();

        double denom = d11 * d22 - d12 * d12;
        double k1 = (plane0.c * d22 - plane1.c * d12) / denom;
        double k2 = (plane1.c * d11 - plane0.c * d12) / denom;
        ray.p = k1 * plane0.n + k2 * plane1.n;
        return std::make_pair(true, ray);
    }
    // 3평면          RTCD 212
    std::pair<bool, vec3> intersect_three_planes(
        const Plane& plane0,
        const Plane& plane1,
        const Plane& plane2
    ) {
        // 평행인지 확인
        if(std::abs(plane0.n.dot(plane1.n)) < 1.e-12) 
            return std::make_pair(false, vec3(0.0, 0.0, 0.0));
        if (std::abs(plane0.n.dot(plane2.n)) < 1.e-12)
            return std::make_pair(false, vec3(0.0, 0.0, 0.0));
        if (std::abs(plane1.n.dot(plane2.n)) < 1.e-12)
            return std::make_pair(false, vec3(0.0, 0.0, 0.0));

        // 평면위에 임의의 점
        vec3 p0 = -plane0.c * plane0.n;
        vec3 p1 = -plane1.c * plane1.n;
        vec3 p2 = -plane2.c * plane2.n;

        vec3 result;
        result =
            p0.dot(plane0.n) * plane1.n.cross(plane2.n) +
            p1.dot(plane1.n) * plane2.n.cross(plane0.n) +
            p2.dot(plane2.n) * plane0.n.cross(plane1.n);
        result /= plane0.n.dot(plane1.n.cross(plane2.n));
        return std::make_pair(true, result);
    }
    //==========================================



    //==========================================
    // overlap
    // 구 -> 구, AABB, 평면
    // AABB -> AABB, 평면
    // 세그먼트 -> AABB, 평면
    // 평면 -> 평면
    // 구, 평면, AABB, 삼각형, 세그먼트 -> 삼각형
    //==========================================
    //✓ 구 -> 구           GPC pp178, RTCD 88
    bool is_overlap(const Sphere& s1, const Sphere& s2) {
        double radiiSum = s1.r + s2.r;
        double sqDistance = (s1.c - s2.c).squaredNorm();
        return sqDistance < radiiSum * radiiSum;
    }
    //✓ 구 -> AABB           GPC pp179, RTCD 165
    bool is_overlap(const Sphere& sphere, const AABB& aabb) {
        vec3 closestPoint = closest(sphere.c, aabb);
        double distSq = (sphere.c - closestPoint).squaredNorm();
        return distSq < sphere.r * sphere.r;
    }
    //✓ 구 -> 평면           GPC pp182, RTCD 160
    bool is_overlap(const Sphere& sphere, const Plane& plane) {
        vec3 closestPoint = closest(sphere.c, plane);
        double distSq = (sphere.c - closestPoint).squaredNorm();
        return distSq < sphere.r * sphere.r;
    }
    //✓ AABB -> AABB           GPC pp184, RTCD 79
    bool is_overlap(const AABB& aabb1, const AABB& aabb2) {
        return (aabb1.min[0] <= aabb2.max[0] && aabb1.max[0] >= aabb2.min[0]) &&
            (aabb1.min[1] <= aabb2.max[1] && aabb1.max[1] >= aabb2.min[1]) &&
            (aabb1.min[2] <= aabb2.max[2] && aabb1.max[2] >= aabb2.min[2]);
    }
    //✓ AABB -> 평면           GPC pp191, RTCD 161, RTR 1275
    bool is_overlap(const AABB& aabb, const Plane& plane) {
        vec3 c = (aabb.max + aabb.min) * 0.5;
        vec3 len = aabb.max - c; // 항상 positive
        double pLen = len.dot(plane.n.cwiseAbs());
        double dist = plane.n.dot(c) - plane.c;
        return std::abs(dist) <= pLen;
    }
    // 평면 -> 평면           GPC pp197
    bool is_overlap(const Plane& plane1, const Plane& plane2) {
        vec3 d = plane1.n.cross(plane2.n);
        return d.squaredNorm() > 1.e-12;
    }
    //✓ 구 -> 삼각형           GPC pp229, RTCD 167
    bool is_overlap(
        const Sphere& s,
        const vec3& a, const vec3& b, const vec3& c
    ) {
        vec3 clo_point = closest(s.c, a, b, c);
        double magSq = (clo_point - s.c).squaredNorm();
        return magSq <= s.r * s.r;
    }
    //✓ AABB -> 삼각형           GPC pp230, RTCD 169, RTR pp1280
    bool is_overlap(
        const AABB& aabb,
        const vec3& a, const vec3& b, const vec3& c
    ) {

        //double eps = std::numeric_limits<double>::epsilon();

        // 박스 중심과 반지름 구하기
        vec3 boxCenter = (aabb.max + aabb.min) * 0.5;
        vec3 boxRadius = aabb.max - boxCenter;

        // 박스 공간으로 삼각형 정점 변환
        vec3 v0 = a - boxCenter;
        vec3 v1 = b - boxCenter;
        vec3 v2 = c - boxCenter;

        // 삼각형 엣지 벡터
        vec3 f0 = v1 - v0;
        vec3 f1 = v2 - v1;
        vec3 f2 = v0 - v2;


        // 1. AABB 축 검사 (3번)
        for (int i = 0; i < 3; ++i) {
            double min_v = std::min({ v0[i], v1[i], v2[i] });
            double max_v = std::max({ v0[i], v1[i], v2[i] });
            if (min_v > boxRadius[i] || max_v < -boxRadius[i]) {
                return false;
            }
        }

        // 2. 삼각형 법선 축 검사 (1번)
        {
            // 삼각형 법선
            vec3 axis = f0.cross(f1);
            double r = boxRadius.dot(axis.cwiseAbs());
            double p0 = v0.dot(axis);
            double p1 = v1.dot(axis);
            double p2 = v2.dot(axis);
            double min_v = std::min({ p0, p1, p2 });
            double max_v = std::max({ p0, p1, p2 });
            if (min_v > r || max_v < -r) {
                return false;
            }
        }

        // 3. 교차 축 검사 (9번)
        const vec3 boxNormals[3] = {
            vec3(1, 0, 0), vec3(0, 1, 0), vec3(0, 0, 1)
        };
        const vec3 edges[3] = { f0, f1, f2 };

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                vec3 axis = boxNormals[i].cross(edges[j]);
                double r = boxRadius.dot(axis.cwiseAbs());
                double p0 = v0.dot(axis);
                double p1 = v1.dot(axis);
                double p2 = v2.dot(axis);
                double min_v = std::min({ p0, p1, p2 });
                double max_v = std::max({ p0, p1, p2 });
                if (min_v > r || max_v < -r) {
                    return false;
                }
            }
        }

        return true;

        //// Nine axes given by the cross products of combination of edges from both
        //// separating axes -> aij
        //double p0{}, p1{}, r{}, s{}, mmm{};

        //// test a00
        //// a00 = u0 × f0 = (1, 0, 0) × f0 = (0, −f0z, f0y)
        //// r -> the closestion radius of a box with respect to an axis n
        //// r = e0 |u0 · a00| + e1 |u1 · a00| + e2 |u2 · a00| 
        ////   = e0 |0| + e1 |−a00z| + e2 | a00y |
        ////   = e1 | f0z | + e2 | f0y |
        //p0 = v0[2] * v1[1] - v0[1] * v1[2];
        //p1 = -v2[1] * f0[2] + v2[2] * f0[1];
        //r = boxRadius[1] * std::abs(f0[2]) + boxRadius[2] * std::abs(f0[1]);
        //mmm = std::max(-std::max(p0, p1), std::min(p0, p1));
        //if (mmm > r + eps) return false;

        //// test a01
        //// a01 = u0 × f1 = (1, 0, 0) × f1 = (0, −f1z, f1y)
        //p0 = v0[1] * v1[2] - v0[1] * v2[2] - v0[2] * v1[1] + v0[2] * v2[1];
        //p1 = -v1[1] * v2[2] + v1[2] * v2[1];
        //r = boxRadius[1] * std::abs(f1[2]) + boxRadius[2] * std::abs(f1[1]);
        //mmm = std::max(-std::max(p0, p1), std::min(p0, p1));
        //if (mmm > r + eps) return false;

        //// test a02
        //// a02 = u0 × f2 = (1, 0, 0) × f2 = (0, −f2z, f2y)
        //p0 = v0[1] * v2[2] - v0[2] * v2[1];
        //p1 = v0[1] * v1[2] - v0[2] * v1[1] + v1[1] * v2[2] - v1[2] * v2[1];
        //r = boxRadius[1] * std::abs(f2[2]) + boxRadius[2] * std::abs(f2[1]);
        //mmm = std::max(-std::max(p0, p1), std::min(p0, p1));
        //if (mmm > r + eps) return false;

        //// test a10
        //// a10 = u1 × f0 = (0, 1, 0) × f0 = ( f0z, 0, −f0x )
        //p0 = v0[0] * v1[2] - v0[2] * v1[0];
        //p1 = v0[0] * v2[2] - v0[2] * v2[0] - v1[0] * v2[2] + v1[2] * v2[0];
        //r = boxRadius[0] * std::abs(f0[2]) + boxRadius[2] * std::abs(f0[0]);
        //mmm = std::max(-std::max(p0, p1), std::min(p0, p1));
        //if (mmm > r + eps) return false;

        //// test a11
        //// a11 = u1 × f1 = (0, 1, 0) × f1 = ( f1z, 0, −f1x )
        //p0 = -v0[0] * v1[2] + v0[0] * v2[2] + v0[2] * v1[0] - v0[2] * v2[0];
        //p1 = v1[0] * v2[2] - v1[2] * v2[0];
        //r = boxRadius[0] * std::abs(f1[2]) + boxRadius[2] * std::abs(f1[0]);
        //mmm = std::max(-std::max(p0, p1), std::min(p0, p1));
        //if (mmm > r + eps) {
        //    return false;
        //}

        //// test a12
        //// a12 = u1 × f2 = (0, 1, 0) × f2 = ( f2z, 0, −f2x )
        //p0 = -v0[0] * v2[2] + v0[2] * v2[0];
        //p1 = -v0[0] * v1[2] + v0[2] * v1[0] - v1[0] * v2[2] + v1[2] * v2[0];
        //r = boxRadius[0] * std::abs(f2[2]) + boxRadius[2] * std::abs(f2[0]);
        //mmm = std::max(-std::max(p0, p1), std::min(p0, p1));
        //if (mmm > r + eps) return false;

        //// test a20
        //// a20 = u2 × f0 = (0, 0, 1) × f0 = (−f0y, f0x , 0)
        //p0 = -v0[0] * v1[1] + v0[1] * v1[0];
        //p1 = -v0[0] * v2[1] + v0[1] * v2[0] + v1[0] * v2[1] - v1[1] * v2[0];
        //r = boxRadius[0] * std::abs(f0[1]) + boxRadius[1] * std::abs(f0[0]);
        //mmm = std::max(-std::max(p0, p1), std::min(p0, p1));
        //if (mmm > r + eps) return false;

        //// test a21
        //// a21 = u2 × f1 = (0, 0, 1) × f1 = (−f1y, f1x , 0)
        //p0 = v0[0] * v1[1] - v0[0] * v2[1] - v0[1] * v1[0] + v0[1] * v2[0];
        //p1 = -v1[0] * v2[1] + v1[1] * v2[0];
        //r = boxRadius[0] * std::abs(f1[1]) + boxRadius[1] * std::abs(f1[0]);
        //mmm = std::max(-std::max(p0, p1), std::min(p0, p1));
        //if (mmm > r + eps) return false;

        //// test a22
        //// a22 = u2 × f2 = (0, 0, 1) × f2 = (−f2y, f2x , 0)
        //p0 = v0[0] * v2[1] - v0[1] * v2[0];
        //p1 = v0[0] * v1[1] - v0[1] * v1[0] + v1[0] * v2[1] - v1[1] * v2[0];
        //r = boxRadius[0] * std::abs(f2[1]) + boxRadius[1] * std::abs(f2[0]);
        //mmm = std::max(-std::max(p0, p1), std::min(p0, p1));
        //if (mmm > r + eps) return false;

        //// Three face normals from the AABB
        //if (std::max(std::max(v0[0], v1[0]), v2[0]) < -boxRadius[0] ||
        //    std::min(std::min(v0[0], v1[0]), v2[0]) > boxRadius[0]) return false;
        //if (std::max(std::max(v0[1], v1[1]), v2[1]) < -boxRadius[1] ||
        //    std::min(std::min(v0[1], v1[1]), v2[1]) > boxRadius[1]) return false;
        //if (std::max(std::max(v0[2], v1[2]), v2[2]) < -boxRadius[2] ||
        //    std::min(std::min(v0[2], v1[2]), v2[2]) > boxRadius[2]) return false;

        //// One face normal from the triangle
        //vec3 n = f0.cross(f1).normalized();
        //double d = n.dot(a);
        //Plane p = Plane(n, d);
        //return is_overlap(aabb, p);


    }
    //✓ 평면 -> 삼각형           GPC pp235, TRT pp1277
    bool is_overlap(
        const Plane& plane,
        const vec3& A, const vec3& B, const vec3& C
    ) {

        // 삼각형 ABC의 평면 방정식 계산
        vec3 nABC = plane.n;
        double dABC = plane.c;

        // 삼각형 PQR의 꼭짓점들이 ABC 평면의 같은 쪽에 있는지 확인
        double sP = nABC.dot(A) + dABC;
        double sQ = nABC.dot(B) + dABC;
        double sR = nABC.dot(C) + dABC;

        if ((sP > 0 && sQ > 0 && sR > 0) || (sP < 0 && sQ < 0 && sR < 0)) {
            return false;  // 삼각형이 서로 다른 면에 있음
        }
        return true;

    }
    //✓ 삼각형 -> 삼각형,     RTR pp1277 -> Guigue와 Devillers 방법
    bool is_overlap(
        const vec3& A, const vec3& B, const vec3& C,
        const vec3& P, const vec3& Q, const vec3& R
    ) {
        // 삼각형 ABC의 평면 방정식 계산
        vec3 nABC = (B - A).cross(C - A);
        double dABC = -nABC.dot(A);

        // 삼각형 PQR의 꼭짓점들이 ABC 평면의 같은 쪽에 있는지 확인
        double sP = nABC.dot(P) + dABC;
        double sQ = nABC.dot(Q) + dABC;
        double sR = nABC.dot(R) + dABC;

        if ((sP > 0 && sQ > 0 && sR > 0) || (sP < 0 && sQ < 0 && sR < 0)) {
            return false;  // 삼각형이 서로 다른 면에 있음
        }

        // 삼각형 PQR의 평면 방정식 계산
        vec3 nPQR = (Q - P).cross(R - P);
        double dPQR = -nPQR.dot(P);

        // 삼각형 ABC의 꼭짓점들이 PQR 평면의 같은 쪽에 있는지 확인
        double sA = nPQR.dot(A) + dPQR;
        double sB = nPQR.dot(B) + dPQR;
        double sC = nPQR.dot(C) + dPQR;

        if ((sA > 0 && sB > 0 && sC > 0) || (sA < 0 && sB < 0 && sC < 0)) {
            return false;  // 삼각형이 서로 다른 면에 있음
        }

        // 정준 형식으로 변환
        vec3 vABC[3] = { A, B, C };
        vec3 vPQR[3] = { P, Q, R };

        // ABC 삼각형 정준화
        if (sA * sB > 0) {
            std::swap(vABC[0], vABC[2]);
        }
        else if (sA * sC > 0) {
            std::swap(vABC[0], vABC[1]);
        }

        // PQR 삼각형 정준화
        if (sP * sQ > 0) {
            std::swap(vPQR[0], vPQR[2]);
        }
        else if (sP * sR > 0) {
            std::swap(vPQR[0], vPQR[1]);
        }

        // nABC 위에 Q가 있어야함.
        if ((vPQR[0] - vABC[0]).dot((vABC[1] - vABC[0]).cross(vABC[2] - vABC[0])) < 0) {
            std::swap(vABC[1], vABC[2]);
        }
        // nPQR 위에 A가 있어야함.
        if ((vABC[0] - vPQR[0]).dot((vPQR[1] - vPQR[0]).cross(vPQR[2] - vPQR[0])) < 0) {
            std::swap(vPQR[1], vPQR[2]);
        }

        // 교차 간격 계산 및 겹침 확인
        double p1p2q1q2 = (vPQR[1] - vABC[0]).dot((vABC[1] - vABC[0]).cross(vPQR[0] - vABC[0]));
        double p1p3q3q1 = (vPQR[0] - vABC[0]).dot((vABC[2] - vABC[0]).cross(vPQR[2] - vABC[0]));

        return (p1p2q1q2 <= 0.0 && p1p3q3q1 <= 0.0);

    }
    //✓ 구 -> 다각형           RTCD 168
    // 1. 구가 다각형 평면과 교차하는지 테스트. 교차하지 않으면 false
    // 2. 다각형의 각 모서리가 구를 관통하는지 테스트. 관통하면 true
    // 3. 구의 중심을 다각형의 평면에 투영. 점-다각형 내부 테스트를 수행하여
    //    다각형 내부에 있는지 확인.



    //==========================================



    //==========================================
    // raycast
    // 광선 -> 구, AABB, 평면, 삼각형
    // is_overlap
    // 광선 대신 선분
    //==========================================
    //✓ 광선 -> 광선           RTR
    std::tuple<bool, double, double> raycast(const Ray& ray0, const Ray& ray1) {
        Eigen::Matrix3d mat;
        mat.col(0) = ray1.p - ray0.p;
        mat.col(1) = ray1.n;
        mat.col(2) = ray0.n.cross(ray1.n);
        double d1d2 = mat.col(2).squaredNorm();
        if (d1d2 == 0.0) return std::make_tuple(false, -1.0, -1.0);
        double s = mat.determinant() / d1d2;
        if (s < 0.0) return std::make_tuple(false, -1.0, -1.0);
        mat.col(1) = ray0.n;
        double t = mat.determinant() / d1d2;
        if (t < 0.0) return std::make_tuple(false, -1.0, -1.0);
        return std::make_tuple(true, s, t);
    }
    //✓ 선분 -> 선분
    bool is_overlap(const Segment& seg0, const Segment& seg1) {
        Ray ray0, ray1;
        ray0.p = seg0.a;
        ray0.n = (seg0.b - seg0.a).normalized();
        ray1.p = seg1.a;
        ray1.n = (seg1.b - seg1.a).normalized();
        auto [is_collide, s, t] = raycast(ray0, ray1);
        return is_collide && 
            s * s <= (seg0.b - seg0.a).squaredNorm() &&
            t * t <= (seg1.b - seg1.a).squaredNorm();
    }
    //✓ 광선 -> 구           GPC pp200, RTCD 177
    std::pair<bool, double> raycast(const Sphere& sphere, const Ray& ray) {
        vec3 l = sphere.c - ray.p;
        double s = l.dot(ray.n);
        double l2 = l.dot(l);
        double r2 = sphere.r * sphere.r;
        if (s < 0.0 && l2 > r2) return std::make_pair(false, -1.0);
        double m2 = l2 - s * s;
        if (m2 > r2) return std::make_pair(false, -1.0);
        double q = std::sqrt(r2 - m2);
        double t = s + q;
        if (l2 > r2) t = s - q;
        return std::make_pair(true, t);
    }
    //✓ 선분 -> 구           RTCD 177
    bool is_overlap(const Segment& seg, const Sphere& sphere) {
        Ray ray;
        ray.p = seg.a;
        ray.n = (seg.b - seg.a).normalized();
        auto [is_collide, t] = raycast(sphere, ray);
        return is_collide && t * t <= (seg.b - seg.a).squaredNorm();
    }
    //✓ 광선 -> AABB           GPC pp206, RTCD 179, RTR 1262
    std::pair<bool, double> raycast(const AABB& aabb, const Ray& ray) {
        double tmin = std::numeric_limits<double>::min();
        double tmax = std::numeric_limits<double>::max();

        // 박스 중심과 반지름 구하기
        vec3 boxCenter = (aabb.max + aabb.min) * 0.5;
        vec3 boxRadius = aabb.max - boxCenter;
        vec3 p = boxCenter - ray.p;
        vec3 a[3] = { vec3(1.0,0.0,0.0), vec3(0.0,1.0,0.0), vec3(0.0,0.0,1.0) };
        for (int i = 0; i < 3; ++i) {
            double hi = boxRadius[i];
            double e = a[i].dot(p);
            double f = a[i].dot(ray.n);
            // 광선 방향이 현재 검사 중인 슬래브의 법선 방향과 수직인지 아닌지 확인
            // 광선이 슬래브 평면과 평행하지 않아 교차할 수 있는지 여부 검사
            if (std::abs(f) > 1.e-12) {
                double t1 = (e + hi) / f;
                double t2 = (e - hi) / f;
                if (t1 > t2) std::swap(t1, t2);
                if (t1 > tmin) tmin = t1;
                if (t2 < tmax) tmax = t2;
                if (tmin > tmax) return std::make_pair(false, -1.0);
                if (tmax < 0.0) return std::make_pair(false, -1.0);
            }
            // 광선이 슬래브 외부에 있는지 검사
            else if (-e - hi > 0.0 || -e + hi < 0.0) return std::make_pair(false, -1.0);
        }
        if (tmin > 0.0) return std::make_pair(true, tmin);
        return std::make_pair(true, tmax);

    }
    //✓ 선분 -> AABB           GPC pp217, RTCD 179
    bool is_overlap(const Segment& seg, const AABB& aabb) {
        Ray ray;
        ray.p = seg.a;
        ray.n = (seg.b - seg.a).normalized();
        auto [is_collide, t] = raycast(aabb, ray);
        return is_collide && t * t <= (seg.b - seg.a).squaredNorm();
    }
    //✓ 광선 -> 평면           GPC pp214
    std::pair<bool, double> raycast(const Plane& plane, const Ray& ray) {
        double nd = ray.n.dot(plane.n);

        // ray는 plane이랑 거의 평행
        if (std::abs(nd) < 1.e-12) return std::make_pair(false, -1.0);

        double pn = ray.p.dot(plane.n);
        double t = (plane.c - pn) / nd;
        if (t < 0.0) return std::make_pair(false, -1.0);
        return std::make_pair(true, t);
    }
    //✓ 선분 -> 평면           GPC pp220, RTCD 175
    bool is_overlap(const Segment& seg, const Plane& plane) {
        Ray ray;
        ray.p = seg.a;
        ray.n = (seg.b - seg.a).normalized();
        auto [is_collide, t] = raycast(plane, ray);
        return is_collide && t * t <= (seg.b - seg.a).squaredNorm();
    }
    //✓ 광선 -> 삼각형           GPC pp244, RTCD 190, RTR 1264
    std::pair<bool, double> raycast(
        const vec3& ta, const vec3& tb, const vec3& tc,
        const Ray& ray
    ) {
        vec3 e1 = tb - ta;
        vec3 e2 = tc - ta;
        vec3 q = ray.n.cross(e2);
        double a = e1.dot(q);
        if (a > -1.e-12 && a < 1.e-12) return std::make_pair(false, -1.0);
        double f = 1.0 / a;
        vec3 s = ray.p - ta;
        double u = f * s.dot(q);
        if (u < 0.0) return std::make_pair(false, -1.0);
        vec3 r = s.cross(e1);
        double v = f * ray.n.dot(r);
        if (v < 0.0 || u + v > 1.0) return std::make_pair(false, -1.0);
        double t = f * e2.dot(r);
        return std::make_pair(true, t);
    }
    //✓ 선분 -> 삼각형           GPC pp244, RTCD 190, RTR
    bool is_overlap(
        const Segment& seg,
        const vec3& ta, const vec3& tb, const vec3& tc
    ) {
        Ray ray;
        ray.p = seg.a;
        ray.n = (seg.b - seg.a).normalized();
        auto [is_collide, t] = raycast(ta, tb, tc, ray);
        return is_collide && t * t <= (seg.b - seg.a).squaredNorm();
    }
    //==========================================






 //   
 //   //점 -> 평면          RTCD 126
 //   std::pair<vec3, double> closest(const vec3& q, const Plane& p) {
 //       double t = (p.n.dot(q) - p.c) / p.n.squaredNorm();
 //       return std::make_pair(q - t * p.n, t);
 //   }

 //   //점 -> 세그먼트          RTCD 127
 //   std::pair<vec3, double> closest(const vec3& q, const Segment& s) {
 //       vec3 ab = s.b - s.a;
 //       vec3 aq = q - s.a;
 //       vec3 bq = q - s.b;
 //       double t = aq.dot(ab);
 //       if (t <= 0.0) {
 //           t = 0.0;
 //           return std::make_pair(s.a, aq.norm());
 //       }
 //       else {
 //           double denom = ab.squaredNorm();
 //           if (t >= denom) {
 //               t = 1.0;
 //               return std::make_pair(s.b, bq.norm());
 //           }
 //           else {
 //               t = t / denom;
 //               return std::make_pair(s.a + t * ab, 
 //                   std::sqrt(aq.squaredNorm() - t * t * denom));
 //           }
 //       }
 //   }



 //   //점 -> 삼각형          RTCD 136
 //   std::pair<vec3, double> closest(
 //       const vec3& p, 
 //       const vec3& a, const vec3& b, const vec3& c
 //   ) {
 //       vec3 ab = b - a;
 //       vec3 ac = c - a;
 //       vec3 bc = c - b;

 //       double snom = (p - a).dot(ab);
 //       double sdenom = (p - b).dot(a - b);
 //       double tnom = (p - a).dot(ac);
 //       double tdenom = (p - c).dot(a - c);

 //       if (snom <= 0.0 && tnom < 0.0) 
 //           return std::make_pair(a, (p - a).norm());

 //       double unom = (p - b).dot(bc);
 //       double udenom = (p - c).dot(b - c);

 //       if (sdenom <= 0.0 && unom <= 0.0) 
 //           return std::make_pair(b, (p - b).norm());
 //       if (tdenom <= 0.0 && udenom <= 0.0) 
 //           return std::make_pair(c, (p - c).norm());

 //       vec3 result;
 //       vec3 n = (b - a).cross(c - a);
 //       double vc = n.dot((a - p).cross(b - p));
 //       if (vc <= 0.0 && snom >= 0.0 && sdenom >= 0.0) {
 //           result = a + snom / (snom + sdenom) * ab;
 //           return std::make_pair(result, (p - result).norm());
 //       }

 //       double va = n.dot((b - p).cross(c - p));
 //       if (va <= 0.0 && unom >= 0.0 && udenom >= 0.0) {
 //           result = b + unom / (unom + udenom) * bc;
 //           return std::make_pair(result, (p - result).norm());
 //       }

 //       double vb = n.dot((c - p).cross(a - p));
 //       if (vb <= 0.0 && tnom >= 0.0 && tdenom >= 0.0) {
 //           result = a + tnom / (tnom + tdenom) * ac;
 //           return std::make_pair(result, (p - result).norm());
 //       }

 //       double u = va / (va + vb + vc);
 //       double v = vb / (va + vb + vc);
 //       double w = 1.0 - u - v;
 //       result = u * a + v * b + w * c;
 //       return std::make_pair(result, (p - result).norm());
 //   }

 //   //점 -> 사면체          RTCD 142
 //   std::pair<vec3, double> closest(
 //       const vec3& p,
 //       const vec3& a, const vec3& b, const vec3& c, const vec3& d
 //   ) {
 //       vec3 closestPt = p;
 //       double bestDist = std::numeric_limits<double>::max();

 //       if (point_outside_of_plane(p, a, b, c)) {
 //           auto [q, dist] = closest(p, a, b, c);
 //           if (dist < bestDist) bestDist = dist, closestPt = q;
 //       }
 //       if (point_outside_of_plane(p, a, c, d)) {
 //           auto [q, dist] = closest(p, a, c, d);
 //           if (dist < bestDist) bestDist = dist, closestPt = q;
 //       }
 //       if (point_outside_of_plane(p, a, d, b)) {
 //           auto [q, dist] = closest(p, a, d, b);
 //           if (dist < bestDist) bestDist = dist, closestPt = q;
 //       }
 //       if (point_outside_of_plane(p, b, d, c)) {
 //           auto [q, dist] = closest(p, b, d, c);
 //           if (dist < bestDist) bestDist = dist, closestPt = q;
 //       }
 //       return std::make_pair(closestPt, bestDist);

 //   }
 //   int point_outside_of_plane(
 //       const vec3& p,
 //       const vec3& a, const vec3& b, const vec3& c
 //   ) {
 //       return (p - a).dot((b - a).cross(c - a)) >= 0.0;
 //   }

 //   //점 -> 다면체          RTCD 145


 //   //세그먼트->세그먼트          RTCD 148
 //   std::tuple<vec3, vec3, double> closest(
 //       const Segment& s1, const Segment& s2
 //   ) {
 //       double epsilon = 1.e-12;
 //       vec3 d1 = s1.b - s1.a;
 //       vec3 d2 = s2.b - s2.a;
 //       vec3 r = s1.a - s2.a;
 //       double a = d1.squaredNorm();
 //       double e = d2.squaredNorm();
 //       double f = d2.dot(r);

 //       vec3 c1, c2;
 //       double s = 0.0;
 //       double t = 0.0;
 //       if (a <= epsilon && e <= epsilon) {
 //           c1 = s1.a;
 //           c2 = s2.a;
 //           return std::make_tuple(c1, c2, (c1 - c2).norm());
 //       }

 //       if (a <= epsilon) {
 //           s = 0.0;
 //           t = f / e;
 //           t = std::clamp(t, 0.0, 1.0);
 //       }
 //       else {
 //           double c = d1.dot(r);
 //           if (e <= epsilon) {
 //               t = 0.0;
 //               s = std::clamp(-c / a, 0.0, 1.0);
 //           }
 //           else {
 //               double b = d1.dot(d2);
 //               double denom = a * e - b * b;

 //               s = 0.0;
 //               if (denom != 0.0) {
 //                   s = std::clamp((b * f - c * e) / denom, 0.0, 1.0);
 //               }
 //               t = (b * s + f) / e;
 //               if (t < 0.0) {
 //                   t = 0.0;
 //                   s = std::clamp(-c / a, 0.0, 1.0);
 //               }
 //               else if (t > 1.0) {
 //                   t = 1.0;
 //                   s = std::clamp((b - c) / a, 0.0, 1.0);
 //               }
 //           }
 //       }
 //       c1 = s1.a + d1 * s;
 //       c2 = s2.a + d2 * t;
 //       return std::make_tuple(c1, c2, (c1 - c2).norm());

 //   }
 //   

	//
	//// AABB vs AABB
	//bool test(const AABB& a, const AABB& b) {
	//	if (a.max[0]<b.min[0] || a.min[0]>b.max[0]) return false;
	//	if (a.max[1]<b.min[1] || a.min[1]>b.max[1]) return false;
	//	if (a.max[2]<b.min[2] || a.min[2]>b.max[2]) return false;
	//	return true;
	//}

	//// 구 vs 구
	//bool test(const Sphere& a, const Sphere& b) {
	//	vec3 d = a.c - b.c;
	//	double dist2 = d.squaredNorm();
	//	double radiusSum = a.r + b.r;
	//	return dist2 <= radiusSum * radiusSum;
	//}

 //   
 //   //구 vs 평면      RTCD p160, GPC p182
 //   bool test(const Sphere& s, const Plane& p) {
 //       double dist = s.c.dot(p.n) - p.c;
 //       return std::abs(dist) <= s.r;
 //   }

 //   //AABB vs 평면       RTCD p161, GPC p190
 //   bool test(const AABB& b, const Plane& p) {
 //       vec3 c = (b.max + b.min) * 0.5;
 //       vec3 n_abs = p.n.cwiseAbs();
 //       vec3 e = b.max - c;
 //       double r = e.dot(n_abs);
 //       double s = p.n.dot(c) - p.c;
 //       return std::abs(s) <= r;
 //   }

 //   //구 vs AABB      RTCD p165
 //   bool test(const AABB& b, const Sphere& s) {
 //       auto [p, dist] = closest(s.c, b);
 //       return dist * dist <= s.r * s.r;
 //   }

 //   //구 vs 삼각형      RTCD p167, GPC p229
 //   bool test(
 //       const Sphere& s, 
 //       const vec3& a, const vec3& b, const vec3& c
 //   ) {
 //       auto [p, dist] = closest(s.c, a, b, c);
 //       return dist * dist <= s.r * s.r;
 //   }

 //   //구 vs 다각형      RTCD p168
 //   bool test(
 //       const Sphere& s,
 //       const std::vector<vec3>& p
 //   ) {
 //       vec3 n = (p[1] - p[0]).cross(p[2] - p[0]).normalized();
 //       Plane m;
 //       m.n = n; m.c = -n.dot(p[0]);
 //       if (!test(s, m)) return false;

 //       for (int k = p.size(), i = 0, j = k - 1; i < k; j = i, ++i) {
 //           double t;
 //           vec3 q;
 //           Ray ray;
 //           ray.p = p[j];
 //           ray.n = p[i] - p[j];
 //           if (test(ray, s, t, q) && t <= 1.0) {
 //               return true;
 //           }
 //       }

 //       auto [q, dist] = closest(s.c, m);
 //       return;
 //   }


 //   //AABB vs 삼각형     RTCD p169, GPC p230
 //   bool test(
 //       const AABB& b,
 //       const vec3& v0, const vec3& v1, const vec3& v2
 //   ) {
 //       double p0, p1, p2, r;
 //       vec3 c = (b.min + b.max) * 0.5;
 //       vec3 e = (b.max - b.min) * 0.5;
 //       vec3 cv0 = v0 - c;
 //       vec3 cv1 = v1 - c;
 //       vec3 cv2 = v2 - c;
 //       vec3 f0 = v1 - v0;
 //       vec3 f1 = v2 - v1;
 //       vec3 f2 = v0 - v2;

 //       p0 = v0[2] * v1[1] - v0[1] * v1[2];
 //       p2 = v2[2] * (v1[1] - v0[1]) - v2[2] * (v1[2] - v0[2]);
 //       r = e[1] * std::abs(f0[2]) + e[2] * std::abs(f0[1]);
 //       if (std::max(-std::max(p0, p2), std::min(p0, p2)) > r)
 //           return false;

 //   }

 //   //삼각형 vs 삼각형     RTCD p172, GPC p237
 //   bool test(
 //       const AABB& b,
 //       const vec3& v0, const vec3& v1, const vec3& v2
 //   ) {
 //   }

 //   //세그먼트 vs 평면      RTCD p175, GPC p220
 //   bool test(
 //       const Segment& s,
 //       const Plane& p
 //   ) {
 //       vec3 ab = s.b - s.a;
 //       double t = (p.c - p.n.dot(s.a)) / p.n.dot(ab);

 //       if (t >= 0.0 && t <= 1.0) {
 //           return true;
 //       }
 //       return false;
 //   }

 //   //세그먼트 vs 구      RTCD p177, GPC p216
 //   bool test(
 //       const Ray& r,
 //       const Sphere& s
 //   ) {
 //       vec3 m = r.p - s.c;
 //       double b = m.dot(r.n);
 //       double c = m.dot(m) - s.r * s.r;
 //       if (c > 0.0 && b > 0.0) return false;
 //       double discr = b * b - c;
 //       if (discr < 0.0) return false;
 //       double t = -b - std::sqrt(discr);
 //       if (t < 0.0) t = 0.0;
 //       //q = p + t * d;
 //       return true;

 //   }


 //   //레이 vs AABB      RTCD p179, GPC p204
 //   bool test(
 //       const Ray& r,
 //       const AABB& b
 //   ) {
 //       double epsilon = 1.e-12;
 //       double tmin = 0.0;
 //       double tmax = std::numeric_limits<double>::max();

 //       for (int i = 0; i < 3; ++i) {
 //           if (std::abs(r.n[i]) < epsilon) {
 //               if (r.p[i]<b.min[i] || r.p[i]>b.max[i]) return false;
 //           }
 //           else {
 //               double ood = 1.0 / r.n[i];
 //               double t1 = (b.min[i] - r.p[i]) * ood;
 //               double t2 = (b.max[i] - r.p[i]) * ood;
 //               if (t1 > t2) std::swap(t1, t2);
 //               if (t1 > tmin) tmin = t1;
 //               if (t2 > tmax) tmax = t2;
 //               if (tmin > tmax) return false;
 //           }
 //       }
 //       //q=p+d*tmin;
 //       return true;
 //   }


 //   //레이 vs 삼각형      RTCD p190, GPC p244
 //   bool test(
 //       const Segment& r, 
 //       const vec3& a, const vec3& b, const vec3& c
 //   ) {
 //       vec3 pq = r.b - r.a;
 //       vec3 pa = a - r.a;
 //       vec3 pb = b - r.a;
 //       vec3 pc = c - r.a;

 //       double u = pb.dot(pq.cross(pc));
 //       if (u < 0.0) return false;
 //       double v = pc.dot(pq.cross(pa));
 //       if (v < 0.0) return false;
 //       double w = pa.dot(pq.cross(pb));
 //       if (w < 0.0) return false;

 //       double denom = 1.0 / (u + v + w);
 //       u *= denom;
 //       v *= denom;
 //       w *= denom;
 //       return true;
 //   }

 //   //세그먼트 vs 다면체      RTCD p198
 //   bool test(
 //       const Segment& s,
 //       const std::vector<Plane>& p 
 //   ) {
 //       vec3 d = s.b - s.a;
 //       double tfirst = 0.0;
 //       double tlast = 1.0;

 //       for (int i = 0; i < p.size(); ++i) {
 //           double denom = p[i].n.dot(d);
 //           double dist = p[i].c - p[i].n.dot(s.a);

 //           if (denom == 0.0) {
 //               if (dist > 0.0) return false;
 //           }
 //           else {
 //               double t = dist / denom;
 //               if (denom < 0.0) {
 //                   if (t > tfirst) tfirst = t;
 //               }
 //               else {
 //                   if (t < tlast) tlast = t;
 //               }

 //               if (tfirst > tlast) return false;
 //           }
 //       }

 //       return true;


 //   }


 //   //점 -> 다각형          RTCD 201
 //   bool is_inside(
 //       const vec3& p,
 //       const std::vector<vec3>& v
 //   ) {
 //       int low = 0, high = v.size();
 //       do {
 //           int mid = (low + high) / 2;
 //           if (triangle_is_ccw(v[0], v[mid], p)) {
 //               low = mid;
 //           }
 //           else {
 //               high = mid;
 //           }
 //       } while (low + 1 < high);

 //       if (low == 0 || high == v.size()) return 0;

 //       return triangle_is_ccw(v[low], v[high], p);

 //   }

 //   //점 -> 삼각형          RTCD 203
 //   bool is_inside(
 //       const vec3& p,
 //       const vec3& a, const vec3& b, const vec3& c
 //   ) {
 //       vec3 pa = a - p;
 //       vec3 pb = b - p;
 //       vec3 pc = c - p;

 //       vec3 u = pb.cross(c);
 //       vec3 v = pc.cross(a);
 //       if (u.dot(v) < 0.0) return false;
 //       vec3 w = pa.cross(pb);
 //       if (u.dot(w) < 0.0) return false;
 //       return true;

 //   }
 //   //점->다면체          RTCD 206
 //   bool is_inside(
 //   
 //   ) {

 //   }














    //// 평면 vs 평면
    //bool test(const Plane& a, const Plane& b) {
    //    double epsilon = 1.e-8;
    //    vec3 d = a.n.cross(b.n);
    //    if (d.squaredNorm() < epsilon) return false;
    //    double d11 = a.n.dot(a.n);
    //    double d12 = a.n.dot(b.n);
    //    double d22 = b.n.dot(b.n);

    //    double denom = d11 * d22 - d12 * d12;
    //    double k1 = (a.c * d22 - b.c * d12) / denom;
    //    double k2 = (b.c * d11 - a.c * d12) / denom;
    //    vec3 p = k1 * a.n + k2 * b.n;
    //    return true;
    //}

	// 점 vs AABB


	//// 구 vs AABB
	//bool test(const Sphere& sphere, const AABB& aabb) {
	//	vec3 closestPoint = closest_point(aabb, sphere.c);
	//	double distSq = (sphere.c - closestPoint).squaredNorm();
	//	double radiusSq = sphere.r * sphere.r;
	//	return distSq < radiusSq;
	//}
	//bool test(const AABB& aabb, const Sphere& sphere) {
	//	return test(sphere, aabb);
	//}

	//// closest point of AABB
	//vec3 closest_point(const AABB& aabb, const vec3& point) {
	//	vec3 result = point;
	//	for (int i = 0; i < 3; ++i) {
	//		result[i] = (result[i] < aabb.min[i]) ? aabb.min[i] : result[i];
	//	}
	//	for (int i = 0; i < 3; ++i) {
	//		result[i] = (result[i] > aabb.max[i]) ? aabb.max[i] : result[i];
	//	}
	//	return result;
	//}



    //// 평면 vs AABB
    //bool test_AABB_plane(AABB& b, Plane& p) {
    //    vec3 c = (b.max + b.min) * 0.5;
    //    vec3 e = b.max - c;

    //    double r = e[0] * std::abs(p.n[0]) + e[1] * std::abs(p.n[1]) + e[2] * std::abs(p.n[2]);
    //    double s = p.n.dot(c) - p.d;
    //    return std::abs(s) <= r;
    //}



    //bool test_AABB_polygon(AABB aabb, Polyhed poly) {

    //    double eps = std::numeric_limits<double>::epsilon();

    //    vec3 boxCenter = (aabb.max + aabb.min) * 0.5;
    //    vec3 boxRadius = (aabb.max - aabb.min) * 0.5;

    //    std::vector<vec3> vs;
    //    vs.reserve(poly.poses.size());
    //    for (int i = 0, size = poly.poses.size(); i < size; ++i) {
    //        vs.push_back(poly.poses[i] - boxCenter);
    //    }
    //    std::vector<vec3> fs;
    //    fs.reserve(poly.poses.size());
    //    for (int i = 0, size = poly.poses.size(); i < size; ++i) {
    //        auto& pos0 = poly.poses[i];
    //        auto& pos1 = poly.poses[(i + 1) % size];
    //        fs.push_back(pos1 - pos0);
    //    }

    //    std::vector<vec3> us(3);
    //    us[0] = vec3(1.0, 0.0, 0.0);
    //    us[1] = vec3(0.0, 1.0, 0.0);
    //    us[2] = vec3(0.0, 0.0, 1.0);


    //    // Nine axes given by the cross products of combination of edges from both
    //    // separating axes -> aij
    //    for (int i = 0, size = fs.size(); i < size; ++i) {
    //        auto& fi = fs[i];
    //        for (int j = 0; j < 3; ++j) {
    //            auto& ui = us[j];

    //            vec3 aij = ui.cross(fi);

    //            double m_max = -std::numeric_limits<double>::max();
    //            double m_min = std::numeric_limits<double>::max();
    //            for (int k = 0; k < size; ++k) {
    //                auto& vk = vs[k];
    //                double p = vk.dot(aij);
    //                m_max = std::max(p, m_max);
    //                m_min = std::min(p, m_min);
    //            }
    //            double mmm = std::max(-m_max, m_min);

    //            double r =
    //                boxRadius[0] * std::abs(us[0].dot(aij)) +
    //                boxRadius[1] * std::abs(us[1].dot(aij)) +
    //                boxRadius[2] * std::abs(us[2].dot(aij));

    //            if (mmm > r + eps) return false;
    //        }
    //    }


    //    // Three face normals from the AABB
    //    for (int idm = 0; idm < 3; ++idm) {
    //        double max_vs = -std::numeric_limits<double>::max();
    //        double min_vs = std::numeric_limits<double>::max();
    //        for (int i = 0, size = vs.size(); i < size; ++i) {
    //            max_vs = std::max(vs[i][idm], max_vs);
    //            min_vs = std::min(vs[i][idm], min_vs);
    //        }
    //        if (max_vs < -boxRadius[idm] || min_vs > boxRadius[idm]) return false;
    //    }

    //    // One face normal from the triangle
    //    vec3 n = fs[0].cross(fs[1]);
    //    double d = n.dot(poly.poses[0]);
    //    Plane p = Plane(n, d);
    //    return test_AABB_plane(aabb, p);

    //}







    //bool test_AABB_polyhed(AABB aabb, Polyhed poly) {

    //    double eps = std::numeric_limits<double>::epsilon();

    //    vec3 boxCenter = (aabb.max + aabb.min) * 0.5;
    //    vec3 boxRadius = (aabb.max - aabb.min) * 0.5;

    //    std::vector<vec3> vs;
    //    vs.reserve(poly.poses.size());
    //    for (int i = 0, size = poly.poses.size(); i < size; ++i) {
    //        vs.push_back(poly.poses[i] - boxCenter);
    //    }
    //    std::vector<vec3> fs;
    //    fs.reserve(poly.poses.size());
    //    for (int i = 0, size = poly.poses.size(); i < size; ++i) {
    //        auto& pos0 = poly.poses[i];
    //        auto& pos1 = poly.poses[(i + 1) % size];
    //        fs.push_back(pos1 - pos0);
    //    }

    //    std::vector<vec3> us(3);
    //    us[0] = vec3(1.0, 0.0, 0.0);
    //    us[1] = vec3(0.0, 1.0, 0.0);
    //    us[2] = vec3(0.0, 0.0, 1.0);


    //    // Nine axes given by the cross products of combination of edges from both
    //    // separating axes -> aij
    //    for (int i = 0, size = fs.size(); i < size; ++i) {
    //        auto& fi = fs[i];
    //        for (int j = 0; j < 3; ++j) {
    //            auto& ui = us[j];

    //            vec3 aij = ui.cross(fi);

    //            double m_max = -std::numeric_limits<double>::max();
    //            double m_min = std::numeric_limits<double>::max();
    //            for (int k = 0; k < size; ++k) {
    //                auto& vk = vs[k];
    //                double p = vk.dot(aij);
    //                m_max = std::max(p, m_max);
    //                m_min = std::min(p, m_min);
    //            }
    //            double mmm = std::max(-m_max, m_min);

    //            double r =
    //                boxRadius[0] * std::abs(us[0].dot(aij)) +
    //                boxRadius[1] * std::abs(us[1].dot(aij)) +
    //                boxRadius[2] * std::abs(us[2].dot(aij));

    //            if (mmm > r + eps) return false;
    //        }
    //    }


    //    // Three face normals from the AABB
    //    for (int idm = 0; idm < 3; ++idm) {
    //        double max_vs = -std::numeric_limits<double>::max();
    //        double min_vs = std::numeric_limits<double>::max();
    //        for (int i = 0, size = vs.size(); i < size; ++i) {
    //            max_vs = std::max(vs[i][idm], max_vs);
    //            min_vs = std::min(vs[i][idm], min_vs);
    //        }
    //        if (max_vs < -boxRadius[idm] || min_vs > boxRadius[idm]) return false;
    //    }

    //    // One face normal from the triangle
    //    for (int i = 0; i < poly.f2v.size(); ++i) {
    //        auto ivs = poly.f2v[i];
    //        vec3 n = fs[ivs[0]].cross(fs[ivs[1]]);
    //        double d = n.dot(poly.poses[ivs[0]]);
    //        Plane p = Plane(n, d);
    //        if (test_AABB_plane(aabb, p) == false) return false;
    //    }
    //    return true;

    //}





    //// Moller-Trumbore 알고리즘 : 삼각형과 광선(무한선)의 교차점을 찾는 알고리즘
    //// input : 시작 지점, 방향, 삼각형의 3개의 점, 교차 점, 교차점까지의 거리, 약간의 오차
    //// Fast, Minimum Storage Ray/Triangle Intersection, 1997
    //// https://wraithkim.wordpress.com/2021/06/08/%EB%A0%88%EC%9D%B4-%ED%8A%B8%EB%A0%88%EC%9D%B4%EC%8B%B1%EC%9D%98-%EC%9B%90%EB%A6%AC/
    //// 표면 방정식 : surfaceNormal*x = d
    //// output : 교차점인가?, 
    //// intersectionPoint : 교차포인트, 
    //// alpha : 포인트에서 교차점까지의 거리
    //template<typename Container, typename T = double>
    //constexpr bool is_intersect_ray_triangle(
    //    Container& point, Container& direction,
    //    Container& A, Container& B, Container& C,
    //    Container& intersectionPoint, T& alpha, T radius = 0.0)
    //{
    //    // 재료 준비
    //    std::array<double, 3> b{}, c{};
    //    std::array<double, 3> uBeta{}, uGamma{};
    //    double bb = 0., bc = 0., cc = 0.;
    //    T eps = std::numeric_limits<T>::epsilon();

    //    for (int i = 0; i < 3; i++) {
    //        b[i] = B[i] - A[i];
    //        c[i] = C[i] - A[i];
    //        bb += b[i] * b[i];
    //        bc += b[i] * c[i];
    //        cc += c[i] * c[i];
    //    }
    //    auto surfaceNormal = semo::math::cross_product(b, c);
    //    double norm = semo::math::norm(surfaceNormal);
    //    norm = std::sqrt(norm);
    //    surfaceNormal[0] /= norm;
    //    surfaceNormal[1] /= norm;
    //    surfaceNormal[2] /= norm;

    //    double D = 1.0 / (cc * bb - bc * bc);
    //    double bbD = bb * D;
    //    double bcD = bc * D;
    //    double ccD = cc * D;

    //    double kBeta = 0.0, kGamma = 0.0, d = 0.0;

    //    for (int i = 0; i < 3; i++) {
    //        uBeta[i] = b[i] * ccD - c[i] * bcD;
    //        uGamma[i] = c[i] * bbD - b[i] * bcD;
    //        kBeta -= A[i] * uBeta[i];
    //        kGamma -= A[i] * uGamma[i];
    //        d += A[i] * surfaceNormal[i];
    //    }

    //    // 본격시작
    //    T testPoint[3] = {};
    //    for (int j = 0; j < 3; ++j) testPoint[j] = point[j] + radius * surfaceNormal[j];

    //    /* Vector<T,3> help; */
    //    auto dotProduct = semo::math::dot_product(direction, surfaceNormal);

    //    // Ray-Plane Intersection Test
    //    if (std::abs(dotProduct) < eps) return false;

    //    // Distance Particle-Plane, 
    //    // 원점에서 교차점까지 distance_vec=point+alpha*dir
    //    alpha = d -
    //        testPoint[0] * surfaceNormal[0] -
    //        testPoint[1] * surfaceNormal[1] -
    //        testPoint[2] * surfaceNormal[2];
    //    alpha /= dotProduct;

    //    // Distance Particle-Plane
    //    if (alpha < -eps) return false;
    //    for (int i = 0; i < 3; i++) intersectionPoint[i] = testPoint[i] + alpha * direction[i];
    //    T beta = kBeta;
    //    for (int i = 0; i < 3; i++) beta += uBeta[i] * intersectionPoint[i];

    //    // Intersection point q in the plane?
    //    if (beta < -eps) return false;
    //    T gamma = kGamma;
    //    for (int i = 0; i < 3; i++) gamma += uGamma[i] * intersectionPoint[i];
    //    if (gamma < -eps) return false;
    //    if (1. - beta - gamma < -eps) return false;
    //    return true;
    //}




};