#pragma once

#include <Eigen/Core>
#include <tuple>
#include <array>

#include "./closest.hpp"
#include "./gjk.hpp"

namespace SimulMan {

    struct Sphere;
    struct AABB;
    struct Ray;
    struct Segment;
    struct Plane;

    using vec2 = Eigen::Vector2d;
    using vec3 = Eigen::Vector3d;
    using vec4 = Eigen::Vector4d;

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

        if (ray.n.squaredNorm() < 1.e-12)
            return std::make_pair(false, ray);

        double d11 = plane0.n.squaredNorm();
        double d12 = plane0.n.dot(plane1.n);
        double d22 = plane1.n.squaredNorm();

        double denom = d11 * d22 - d12 * d12;
        double k1 = (plane0.d * d22 - plane1.d * d12) / denom;
        double k2 = (plane1.d * d11 - plane0.d * d12) / denom;
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
        if (std::abs(plane0.n.dot(plane1.n)) < 1.e-12)
            return std::make_pair(false, vec3(0.0, 0.0, 0.0));
        if (std::abs(plane0.n.dot(plane2.n)) < 1.e-12)
            return std::make_pair(false, vec3(0.0, 0.0, 0.0));
        if (std::abs(plane1.n.dot(plane2.n)) < 1.e-12)
            return std::make_pair(false, vec3(0.0, 0.0, 0.0));

        vec3 u = plane1.n.cross(plane2.n);
        double denom = plane0.n.dot(u);
        if (std::abs(denom) < 1.e-12) return std::make_pair(false, vec3(0.0, 0.0, 0.0));

        vec3 result = (plane0.d * u +
            plane0.n.cross(plane2.d * plane1.n - plane1.d * plane2.n)) / denom;
        return std::make_pair(true, result);

        //// 평면위에 임의의 점
        //vec3 p0 = -plane0.d * plane0.n;
        //vec3 p1 = -plane1.d * plane1.n;
        //vec3 p2 = -plane2.d * plane2.n;

        //vec3 result;
        //result =
        //    p0.dot(plane0.n) * plane1.n.cross(plane2.n) +
        //    p1.dot(plane1.n) * plane2.n.cross(plane0.n) +
        //    p2.dot(plane2.n) * plane0.n.cross(plane1.n);
        //result /= plane0.n.dot(plane1.n.cross(plane2.n));
        //return std::make_pair(true, result);
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
    bool is_overlap(const AABB& aabb, const Sphere& sphere) {
        return is_overlap(sphere, aabb);
    }
    //✓ 구 -> 평면           GPC pp182, RTCD 160
    bool is_overlap(const Sphere& sphere, const Plane& plane) {
        vec3 closestPoint = closest(sphere.c, plane);
        double distSq = (sphere.c - closestPoint).squaredNorm();
        return distSq < sphere.r * sphere.r;
    }
    bool is_overlap(const Plane& plane, const Sphere& sphere) {
        return is_overlap(sphere, plane);
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
        double dist = plane.n.dot(c) - plane.d;
        return std::abs(dist) <= pLen;
    }
    bool is_overlap(const Plane& plane, const AABB& aabb) {
        return is_overlap(aabb, plane);
    }
    // 평면 -> 평면           GPC pp197
    bool is_overlap(const Plane& plane1, const Plane& plane2) {
        vec3 d = plane1.n.cross(plane2.n);
        return d.squaredNorm() > 1.e-12 ?
            true : (std::abs(plane1.d - plane2.d) < 1.e-12);
    }
    //✓ 구 -> 삼각형           GPC pp229, RTCD 167
    bool is_overlap(
        const Sphere& s,
        const Triangle& tri
    ) {
        vec3 clo_point = closest(s.c, Triangle{ tri.a, tri.b, tri.c });
        double magSq = (clo_point - s.c).squaredNorm();
        return magSq <= s.r * s.r;
    }
    bool is_overlap(
        const Triangle& tri,
        const Sphere& s
    ) {
        return is_overlap(s, tri);
    }
    //✓ AABB -> 삼각형           GPC pp230, RTCD 169, RTR pp1280
    bool is_overlap(
        const AABB& aabb,
        const Triangle& tri
    ) {

        //double eps = std::numeric_limits<double>::epsilon();

        // 박스 중심과 반지름 구하기
        vec3 boxCenter = (aabb.max + aabb.min) * 0.5;
        vec3 boxRadius = aabb.max - boxCenter;

        // 박스 공간으로 삼각형 정점 변환
        vec3 v0 = tri.a - boxCenter;
        vec3 v1 = tri.b - boxCenter;
        vec3 v2 = tri.c - boxCenter;

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
    bool is_overlap(
        const Triangle& tri,
        const AABB& aabb
    ) {
        return is_overlap(aabb, tri);
    }
    //✓ 평면 -> 삼각형           GPC pp235, TRT pp1277
    bool is_overlap(
        const Plane& plane,
        const Triangle& tri
    ) {

        // 삼각형 ABC의 평면 방정식 계산
        vec3 nABC = plane.n;
        double dABC = plane.d;

        // 삼각형 PQR의 꼭짓점들이 ABC 평면의 같은 쪽에 있는지 확인
        double sA = nABC.dot(tri.a) - dABC;
        double sB = nABC.dot(tri.b) - dABC;
        double sC = nABC.dot(tri.c) - dABC;

        if ((sA > 0 && sB > 0 && sC > 0) || (sA < 0 && sB < 0 && sC < 0)) {
            return false;  // 삼각형이 서로 다른 면에 있음
        }
        return true;

    //// 부호 확인
    //    int pos_count = 0, neg_count = 0;

    //    if (sA > 1.e-12) pos_count++;
    //    else if (sA < -1.e-12) neg_count++;

    //    if (sB > 1.e-12) pos_count++;
    //    else if (sB < -1.e-12) neg_count++;

    //    if (sC > 1.e-12) pos_count++;
    //    else if (sC < -1.e-12) neg_count++;

    //    // 교차 여부 판단
    //    return (pos_count > 0 && neg_count > 0) || (pos_count + neg_count < 3);

    }
    bool is_overlap(
        const Triangle& tri,
        const Plane& plane
    ) {
        return is_overlap(plane, tri);
    }
    //✓ 삼각형 -> 삼각형,     RTR pp1277 -> Guigue와 Devillers 방법
    bool is_overlap(
        const Triangle& tri0,
        const Triangle& tri1
    ) {
        // 삼각형 ABC의 평면 방정식 계산
        vec3 nABC = (tri0.b - tri0.a).cross(tri0.c - tri0.a);
        double dABC = -nABC.dot(tri0.a);

        // 삼각형 PQR의 꼭짓점들이 ABC 평면의 같은 쪽에 있는지 확인
        double sP = nABC.dot(tri1.a) + dABC;
        double sQ = nABC.dot(tri1.b) + dABC;
        double sR = nABC.dot(tri1.c) + dABC;

        if ((sP > 0 && sQ > 0 && sR > 0) || (sP < 0 && sQ < 0 && sR < 0)) {
            return false;  // 삼각형이 서로 다른 면에 있음
        }

        // 삼각형 PQR의 평면 방정식 계산
        vec3 nPQR = (tri1.b - tri1.a).cross(tri1.c - tri1.a);
        double dPQR = -nPQR.dot(tri1.a);

        // 삼각형 ABC의 꼭짓점들이 PQR 평면의 같은 쪽에 있는지 확인
        double sA = nPQR.dot(tri0.a) + dPQR;
        double sB = nPQR.dot(tri0.b) + dPQR;
        double sC = nPQR.dot(tri0.c) + dPQR;

        if ((sA > 0 && sB > 0 && sC > 0) || (sA < 0 && sB < 0 && sC < 0)) {
            return false;  // 삼각형이 서로 다른 면에 있음
        }

        // 정준 형식으로 변환
        vec3 vABC[3] = { tri0.a, tri0.b, tri0.c };
        vec3 vPQR[3] = { tri1.a, tri1.b, tri1.c };

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
        if (d1d2 < 1.e-12) {
            // 광선 겹침
            if (
                std::abs(mat.col(0).normalized().dot(ray0.n) - 1.0) < 1.e-12 &&
                std::abs(mat.col(0).normalized().dot(ray1.n) - 1.0) < 1.e-12
                ) {
                return std::make_tuple(true, 0.0, 0.0);
            }
            return std::make_tuple(false, -1.0, -1.0);
        }
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
    std::pair<bool, double> raycast(const Ray& ray, const Sphere& sphere) {
        return raycast(sphere, ray);
    }
    //✓ 선분 -> 구           RTCD 177
    bool is_overlap(const Segment& seg, const Sphere& sphere) {
        Ray ray;
        ray.p = seg.a;
        ray.n = (seg.b - seg.a).normalized();
        auto [is_collide, t] = raycast(sphere, ray);
        return is_collide && t * t <= (seg.b - seg.a).squaredNorm();
    }
    bool is_overlap(const Sphere& sphere, const Segment& seg) {
        return is_overlap(seg, sphere);
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
    std::pair<bool, double> raycast(const Ray& ray, const AABB& aabb) {
        return raycast(aabb, ray);
    }
    //✓ 선분 -> AABB           GPC pp217, RTCD 179
    bool is_overlap(const Segment& seg, const AABB& aabb) {
        Ray ray;
        ray.p = seg.a;
        ray.n = (seg.b - seg.a).normalized();
        auto [is_collide, t] = raycast(aabb, ray);
        return is_collide && t * t <= (seg.b - seg.a).squaredNorm();
    }
    bool is_overlap(const AABB& aabb, const Segment& seg) {
        return is_overlap(seg, aabb);
    }
    //✓ 광선 -> 평면           GPC pp214
    std::pair<bool, double> raycast(const Plane& plane, const Ray& ray) {
        double nd = ray.n.dot(plane.n);

        // ray는 plane이랑 거의 평행
        if (std::abs(nd) < 1.e-12) return std::make_pair(false, -1.0);

        double pn = ray.p.dot(plane.n);
        double t = (plane.d - pn) / nd;
        if (t < 0.0) return std::make_pair(false, -1.0);
        return std::make_pair(true, t);
    }
    std::pair<bool, double> raycast(const Ray& ray, const Plane& plane) {
        return raycast(plane, ray);
    }
    //✓ 선분 -> 평면           GPC pp220, RTCD 175
    bool is_overlap(const Segment& seg, const Plane& plane) {
        Ray ray;
        ray.p = seg.a;
        ray.n = (seg.b - seg.a).normalized();
        auto [is_collide, t] = raycast(plane, ray);
        return is_collide && t * t <= (seg.b - seg.a).squaredNorm();
    }
    bool is_overlap(const Plane& plane, const Segment& seg) {
        return is_overlap(seg, plane);
    }
    //✓ 광선 -> 삼각형           GPC pp244, RTCD 190, RTR 1264
    std::pair<bool, double> raycast(
        const Ray& ray,
        const Triangle& tri
    ) {
        vec3 e1 = tri.b - tri.a;
        vec3 e2 = tri.c - tri.a;
        vec3 q = ray.n.cross(e2);
        double a = e1.dot(q);
        if (a > -1.e-12 && a < 1.e-12) return std::make_pair(false, -1.0);
        double f = 1.0 / a;
        vec3 s = ray.p - tri.a;
        double u = f * s.dot(q);
        if (u < 0.0) return std::make_pair(false, -1.0);
        vec3 r = s.cross(e1);
        double v = f * ray.n.dot(r);
        if (v < 0.0 || u + v > 1.0) return std::make_pair(false, -1.0);
        double t = f * e2.dot(r);
        return std::make_pair(true, t);
    }
    std::pair<bool, double> raycast(const Triangle& tri, const Ray& ray) {
        return raycast(ray, tri);
    }
    //✓ 선분 -> 삼각형           GPC pp244, RTCD 190, RTR
    bool is_overlap(
        const Segment& seg,
        const Triangle& tri
    ) {
        Ray ray;
        ray.p = seg.a;
        ray.n = (seg.b - seg.a).normalized();
        auto [is_collide, t] = raycast(ray, tri);
        return is_collide && t * t <= (seg.b - seg.a).squaredNorm();
    }
    bool is_overlap(
        const Triangle& tri,
        const Segment& seg
    ) {
        return is_overlap(seg, tri);
    }
    //==========================================


    // GJK
    bool is_overlap(
        const ConvexPolyhedra& shape1,
        const ConvexPolyhedra& shape2
    ) {
        return GJK::is_overlap(shape1, shape2);
    }
    bool is_overlap_gjk(
        const ConvexPolyhedra& shape1,
        const ConvexPolyhedra& shape2
    ) {
        return is_overlap(shape1, shape2);
    }




}

