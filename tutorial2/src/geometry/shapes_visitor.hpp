#pragma once

#include <Eigen/Core>
#include <tuple>
#include <array>
#include <map>
#include <functional>
#include <stdexcept>
#include <utility>
#include <variant>


namespace SimulMan {

    using vec3 = Eigen::Vector3d;
    using real_t = double;
    using index_t = size_t;

    using index2_t = std::vector<std::vector<index_t>>;
    using vec_vec3_t = std::vector<vec3>;
    using triangluation_t = std::tuple<vec_vec3_t, vec_vec3_t, vec_vec3_t, index2_t>;




    // 1d
    using Point = vec3;


    // 2d
    struct Triangle {
        vec3 a{}, b{}, c{};

    };

    struct Square {
        vec3 a{}, b{}, c{}, d{};

    };

    // n . x = d
    struct Plane {
        Plane() = default;

        Plane(const vec3& normal, const real_t& distance) :
            n(normal), d(distance)
        {
            d /= n.norm();
            n.normalize();
        }

        vec3 n{};
        real_t d{};

    };

    struct Ray {

        Ray() = default;

        Ray(const vec3& point, const vec3& normal) :
            p(point), n(normal) 
        {
            n.normalize();
        }

        vec3 p{}, n{};

    };

    struct Segment {
        vec3 a{}, b{};

    };

    using ConvexPolygon = std::vector<vec3>;

    // 3d
    struct Sphere {

        Sphere() = default;

        Sphere(const vec3& center, const real_t& radius) :
            c(center), r(radius) {}

        vec3 c{};
        real_t r{};

    };

    struct AABB {

        AABB() = default;

        AABB(const vec3& minimum, const vec3& maximum) :
            min(minimum), max(maximum)
        {

        }

        vec3 min{}, max{};

    };
    struct Tetrahedra {
        vec3 a{}, b{}, c{}, d{};

    };

    using ConvexPolyhedra = std::vector<vec3>;


    using Shape = std::variant<Point, Triangle, Square,
        Plane, Ray, Segment, Sphere, AABB, ConvexPolyhedra, Tetrahedra>;



    class ShapeInsideVisitor
    {
    public:
        template<typename T, typename U>
        bool operator()(T const& p, U const& s) const {
            if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Sphere>) {
                return is_inside(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, AABB>) {
                return is_inside(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Plane>) {
                return is_inside(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Segment>) {
                return is_inside(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Ray>) {
                return is_inside(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Triangle>) {
                return is_inside(p, s);
            }

            else {
                return false;
            }
        }
    };
    bool is_inside(const Shape& shape1, const Shape& shape2)
    {
        return std::visit(ShapeInsideVisitor{}, shape1, shape2);
    }



    class ShapeClosestVisitor
    {
    public:
        template<typename T, typename U>
        vec3 operator()(T const& p, U const& s) const {
            if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Sphere>) {
                return closest(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, AABB>) {
                return closest(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Plane>) {
                return closest(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Segment>) {
                return closest(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Ray>) {
                return closest(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Triangle>) {
                return closest(p, s);
            }
            else if constexpr (std::is_same_v<T, Point> && std::is_same_v<U, Tetrahedra>) {
                return closest(p, s);
            }

            else {
                return {
                    std::numeric_limits<real_t>::max(),
                    std::numeric_limits<real_t>::max(),
                    std::numeric_limits<real_t>::max()
                };
            }
        }
    };
    vec3 closest(const Shape& shape1, const Shape& shape2)
    {
        return std::visit(ShapeClosestVisitor{}, shape1, shape2);
    }

    bool is_overlap_gjk(
        const ConvexPolyhedra& shape1,
        const ConvexPolyhedra& shape2
    );

    class ShapeOverlapVisitor
    {
    public:
        template<typename T, typename U>
        bool operator()(T const& shape1, U const& shape2) const {
            if constexpr (std::is_same_v<T, Sphere> && std::is_same_v<U, Sphere>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, Sphere> && std::is_same_v<U, AABB>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, AABB> && std::is_same_v<U, Sphere>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, Sphere> && std::is_same_v<U, Plane>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Plane> && std::is_same_v<U, Sphere>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, AABB> && std::is_same_v<U, AABB>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, AABB> && std::is_same_v<U, Plane>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Plane> && std::is_same_v<U, AABB>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, Plane> && std::is_same_v<U, Plane>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, Segment> && std::is_same_v<U, Segment>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, Segment> && std::is_same_v<U, Sphere>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Sphere> && std::is_same_v<U, Segment>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, Segment> && std::is_same_v<U, AABB>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, AABB> && std::is_same_v<U, Segment>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, Segment> && std::is_same_v<U, Plane>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Plane> && std::is_same_v<U, Segment>) {
                return is_overlap(shape1, shape2);
            }

            else if constexpr (std::is_same_v<T, Sphere> && std::is_same_v<U, Triangle>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Triangle> && std::is_same_v<U, Sphere>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, AABB> && std::is_same_v<U, Triangle>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Triangle> && std::is_same_v<U, AABB>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Plane> && std::is_same_v<U, Triangle>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Triangle> && std::is_same_v<U, Plane>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Triangle> && std::is_same_v<U, Triangle>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Segment> && std::is_same_v<U, Triangle>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, Triangle> && std::is_same_v<U, Segment>) {
                return is_overlap(shape1, shape2);
            }
            else if constexpr (std::is_same_v<T, ConvexPolyhedra> && std::is_same_v<U, ConvexPolyhedra>) {
                return is_overlap_gjk(shape1, shape2);
            }

            else {
                return false;
            }
        }
    };


    bool is_overlap(const Shape& shape1, const Shape& shape2)
    {
        return std::visit(ShapeOverlapVisitor{}, shape1, shape2);
    }


    // 레이케스트
    class ShapeRaycastVisitor
    {
    public:
        template<typename T, typename U>
        std::pair<bool, double> operator()(T const& shape1, U const& shape2) const {
            if constexpr (
                (std::is_same_v<T, Ray> && std::is_same_v<U, Triangle>) ||
                (std::is_same_v<T, Triangle> && std::is_same_v<U, Ray>)
            ) {
                return raycast(shape1, shape2);
            }
            else if constexpr (
                (std::is_same_v<T, Ray> && std::is_same_v<U, Plane>) ||
                (std::is_same_v<T, Plane> && std::is_same_v<U, Ray>)
            ) {
                return raycast(shape1, shape2);
            }
            else if constexpr (
                (std::is_same_v<T, Ray> && std::is_same_v<U, AABB>) ||
                (std::is_same_v<T, AABB> && std::is_same_v<U, Ray>)
            ) {
                return raycast(shape1, shape2);
            }
            else if constexpr (
                (std::is_same_v<T, Ray> && std::is_same_v<U, Sphere>) ||
                (std::is_same_v<T, Sphere> && std::is_same_v<U, Ray>)
            ) {
                return raycast(shape1, shape2);
            }

            else {
                return std::make_pair(false, -1);
            }

        }
    };

    std::pair<bool, double> raycast(const Shape& shape1, const Shape& shape2)
    {
        return std::visit(ShapeRaycastVisitor{}, shape1, shape2);
    }


    // 2 평면
    class ShapeTwoPlanesVisitor
    {
    public:
        template<typename T, typename U>
        std::pair<bool, Ray> operator()(T const& shape1, U const& shape2) const {
            if constexpr (
                (std::is_same_v<T, Plane> && std::is_same_v<U, Plane>) 
            ) {
                return intersect_two_planes(shape1, shape2);
            }

            else {
                return std::make_pair(false, Ray{});
            }

        }
    };
    std::pair<bool, Ray> intersect_two_planes(const Shape& shape1, const Shape& shape2)
    {
        return std::visit(ShapeTwoPlanesVisitor{}, shape1, shape2);
    }


    // 3 평면
    class ShapeTreePlanesVisitor
    {
    public:
        template<typename T, typename U, typename P>
        std::pair<bool, vec3> operator()(T const& shape1, U const& shape2, P const& shape3) const {
            if constexpr (
                (std::is_same_v<T, Plane> && std::is_same_v<U, Plane> && std::is_same_v<P, Plane>)
                ) {
                return intersect_three_planes(shape1, shape2, shape3);
            }

            else {
                return std::make_pair(false, vec3{});
            }

        }
    };
    std::pair<bool, vec3> intersect_three_planes(const Shape& shape1, const Shape& shape2, const Shape& shape3)
    {
        return std::visit(ShapeTreePlanesVisitor{}, shape1, shape2, shape3);
    }






    std::pair<std::vector<vec3>, index2_t> 삼각형() {

        std::vector<vec3> pos;
        index2_t f2v;


        pos = {
            {0,  0,  0}, { 0,  1,  0}, {1, 1,  0}
        };

        f2v = {
            {0, 1, 2}
        };

        return std::make_pair(pos, f2v);

    }
    std::pair<std::vector<vec3>, index2_t> 정십이면체() {

        std::vector<vec3> pos;
        index2_t f2v;

        // Golden ratio
        const double t = (1.0 + std::sqrt(5.0)) / 2.0;  // 황금비

        pos = {
            {-1,  t,  0}, { 1,  t,  0}, {-1, -t,  0}, { 1, -t,  0},
            { 0, -1,  t}, { 0,  1,  t}, { 0, -1, -t}, { 0,  1, -t},
            { t,  0, -1}, { t,  0,  1}, {-t,  0, -1}, {-t,  0,  1}
        };

        f2v = {
            {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
            {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
            {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
            {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
        };

        return std::make_pair(pos, f2v);

    }












    class ShapeTriangulator {
    public:

        template<typename T>
        triangluation_t operator()(T const& s) const {

            if constexpr (std::is_same_v<T, AABB>) {
                //       4 -------- 5
                //      /|         /|
                //     / |        / |
                //    /  |       /  |
                //   7 -------- 6   |
                //   |   |      |   |
                //   |   0 -----|-- 1
                //   |  /       |  /
                //   | /        | /
                //   |/         |/
                //   3 -------- 2	
                vec3 radius = 0.5 * (s.max - s.min);
                vec3 center = s.max - radius;
                vec3 v0 = { center[0] - radius[0], center[1] - radius[1], center[2] - radius[2] };
                vec3 v1 = { center[0] + radius[0], center[1] - radius[1], center[2] - radius[2] };
                vec3 v2 = { center[0] + radius[0], center[1] + radius[1], center[2] - radius[2] };
                vec3 v3 = { center[0] - radius[0], center[1] + radius[1], center[2] - radius[2] };
                vec3 v4 = { center[0] - radius[0], center[1] - radius[1], center[2] + radius[2] };
                vec3 v5 = { center[0] + radius[0], center[1] - radius[1], center[2] + radius[2] };
                vec3 v6 = { center[0] + radius[0], center[1] + radius[1], center[2] + radius[2] };
                vec3 v7 = { center[0] - radius[0], center[1] + radius[1], center[2] + radius[2] };

                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos;
                std::vector<vec3> pos = { v0,v1,v2,v3,v4,v5,v6,v7 };
                index2_t f2v = {
                    { 1, 0, 2 },
                    { 0, 1, 5 },
                    { 1, 2, 5 },
                    { 2, 0, 3 },
                    { 3, 0, 7 },
                    { 2, 3, 7 },
                    { 4, 0, 5 },
                    { 0, 4, 7 },
                    { 4, 5, 7 },
                    { 5, 2, 6 },
                    { 6, 2, 7 },
                    { 5, 6, 7 }
                };
                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }
            else if constexpr (std::is_same_v<T, Sphere>) {

                const double t = (1.0 + std::sqrt(5.0)) / 2.0;  // 황금비

                // 정이십면체의 초기 정점
                std::vector<vec3> pos = {
                    {-1,  t,  0}, { 1,  t,  0}, {-1, -t,  0}, { 1, -t,  0},
                    { 0, -1,  t}, { 0,  1,  t}, { 0, -1, -t}, { 0,  1, -t},
                    { t,  0, -1}, { t,  0,  1}, {-t,  0, -1}, {-t,  0,  1}
                };

                // 각 삼각형 면을 정의하는 인덱스
                index2_t f2v = {
                    {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
                    {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
                    {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
                    {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
                };

                // 각 정점을 구면에 맞게 정규화
                for (auto& vertex : pos) {
                    vertex.normalize();
                }


                auto getMidPoint = [&](index_t v1, index_t v2, std::map<std::pair<index_t, index_t>, index_t>& midPointCache) {
                    std::pair<index_t, index_t> key = std::minmax(v1, v2);

                    if (midPointCache.find(key) != midPointCache.end()) {
                        return midPointCache[key];
                    }

                    vec3 midPoint = vec3(
                        (pos[v1][0] + pos[v2][0]) / 2.0,
                        (pos[v1][1] + pos[v2][1]) / 2.0,
                        (pos[v1][2] + pos[v2][2]) / 2.0
                    ).normalized();

                    pos.push_back(midPoint);
                    index_t index = pos.size() - 1;
                    midPointCache[key] = index;

                    return index;
                    };

                auto subdivide = [&]() {
                    index2_t newFaces;
                    std::map<std::pair<index_t, index_t>, index_t> midPointCache;

                    for (const auto& face : f2v) {
                        index_t a = getMidPoint(face[0], face[1], midPointCache);
                        index_t b = getMidPoint(face[1], face[2], midPointCache);
                        index_t c = getMidPoint(face[2], face[0], midPointCache);

                        newFaces.push_back({ face[0], a, c });
                        newFaces.push_back({ face[1], b, a });
                        newFaces.push_back({ face[2], c, b });
                        newFaces.push_back({ a, b, c });
                    }

                    f2v = newFaces;

                    };

                subdivide();
                subdivide();
                //subdivide();


                for (auto& p : pos) {
                    p *= s.r;
                    p += s.c;
                }

                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos;


                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }
            else if constexpr (std::is_same_v<T, vec3>) {

                //double size = 0.1;
                //// 삼각형의 높이 계산
                //double height = size * std::sqrt(3.0) * 0.5;
                //const auto& center = point.p;
                //// 정점 생성
                //std::vector<vec3> pos = {
                //    {center[0], center[1] + height / 3.0, center[2]},                  // 상단 정점
                //    {center[0] - size / 2.0, center[1] - height / 6.0, center[2]},         // 좌하단 정점
                //    {center[0] + size / 2.0, center[1] - height / 6.0, center[2]}          // 우하단 정점
                //};
                //index2_t f2v = { {0, 1, 2} };


                std::vector<vec3> real_pos = { s };// { s.p };
                std::vector<vec3> line_pos;
                std::vector<vec3> pos;
                index2_t f2v;

                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }
            else if constexpr (std::is_same_v<T, Triangle>) {

                std::vector<vec3> pos = { s.a, s.b, s.c };
                index2_t f2v = { { 0, 1, 2 } };
                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos;

                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }
            else if constexpr (std::is_same_v<T, Square>) {

                // 정점 생성 (시계 방향)
                std::vector<vec3> pos = {
                    s.a,  // 좌하단
                    s.b,   // 우하단
                    s.c,    // 우상단
                    s.d    // 좌상단
                };

                // 인덱스 생성 (두 개의 삼각형으로 구성)
                index2_t f2v = {
                    {0, 1, 2},  // 첫 번째 삼각형
                    {0, 2, 3}   // 두 번째 삼각형
                };

                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos;

                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }
            else if constexpr (std::is_same_v<T, Plane>) {

                double size = 1.0;

                // 정규화된 법선 벡터
                vec3 normal = s.n.normalized();

                // 평면 위의 한 점 찾기
                vec3 point = vec3(normal[0] * s.d, normal[1] * s.d, normal[2] * s.d);

                // 평면 위의 두 수직 벡터 찾기
                vec3 u = normal.cross(vec3(0, 1, 0));
                if (u.squaredNorm() < 1e-6) {
                    u = normal.cross(vec3(1, 0, 0));
                }
                u = u.normalized();
                vec3 v = normal.cross(u);

                std::vector<vec3> pos;
                pos.push_back(vec3(point[0] + size * (u[0] - v[0]), point[1] + size * (u[1] - v[1]), point[2] + size * (u[2] - v[2])));
                pos.push_back(vec3(point[0] + size * (u[0] + v[0]), point[1] + size * (u[1] + v[1]), point[2] + size * (u[2] + v[2])));
                pos.push_back(vec3(point[0] + size * (-u[0] + v[0]), point[1] + size * (-u[1] + v[1]), point[2] + size * (-u[2] + v[2])));
                pos.push_back(vec3(point[0] + size * (-u[0] - v[0]), point[1] + size * (-u[1] - v[1]), point[2] + size * (-u[2] - v[2])));
                //pos.push_back(vec3(0, 0, 0));
                //pos.push_back(vec3(5, 0, 0));
                //pos.push_back(vec3(0, 5, 0));
                //pos.push_back(vec3(5, 5, 0));

                index2_t f2v = { {0, 1, 2}, {0, 2, 3} };
                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos;

                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }
            else if constexpr (std::is_same_v<T, Ray>) {

                std::vector<vec3> pos;
                index2_t f2v;
                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos = { s.p, s.p + 5.0 * s.n };

                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }
            else if constexpr (std::is_same_v<T, Segment>) {

                std::vector<vec3> pos;
                index2_t f2v;
                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos = { s.a, s.b };

                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }
            // 정 십이면체
            else if constexpr (std::is_same_v<T, ConvexPolyhedra>) {

                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos;

                //auto [pos, f2v] = 정십이면체();
                auto [pos, f2v] = 삼각형();
                pos = s;

                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }
            else if constexpr (std::is_same_v<T, Tetrahedra>) {

                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos;

                std::vector<vec3> pos = { s.a, s.b, s.c, s.d };
                index2_t f2v = {
                    {0,1,2},
                    {0,2,3},
                    {0,3,1},
                    {1,3,2}
                };

                return std::make_tuple(real_pos, line_pos, pos, f2v);
                }
            else {
                std::vector<vec3> pos;
                index2_t f2v;
                std::vector<vec3> real_pos;
                std::vector<vec3> line_pos;

                return std::make_tuple(real_pos, line_pos, pos, f2v);
            }

        }

    };

    triangluation_t trianglation(const Shape& s)
    {
        return std::visit(ShapeTriangulator{}, s);
    }





}

