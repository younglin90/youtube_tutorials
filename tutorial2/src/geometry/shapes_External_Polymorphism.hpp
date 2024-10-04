#pragma once

#include <Eigen/Core>
#include <tuple>
#include <array>
#include <map>
#include <functional>
#include <stdexcept>
#include <utility>


namespace SimulMan {

    using vec3 = Eigen::Vector3d;
    using real_t = double;
    using index_t = size_t;

    using index2_t = std::vector<std::vector<index_t>>;
    using vec_vec3_t = std::vector<vec3>;
    using triangluation_t = std::tuple<vec_vec3_t, vec_vec3_t, vec_vec3_t, index2_t>;




    // 1d
    struct Point {
        vec3 p{};

    };


    // 2d
    struct Triangle {
        vec3 a{}, b{}, c{};

    };

    struct Square {
        vec3 a{}, b{}, c{}, d{};

    };

    struct Plane {
        vec3 n{};
        real_t d{};

    };

    struct Ray {
        vec3 p{}, n{};

    };

    struct Segment {
        vec3 a{}, b{};

    };


    // 3d
    struct Sphere {
        vec3 c{};
        real_t r{};

    };

    struct AABB {
        AABB(const vec3& minimum, const vec3& maximum) :
            min(minimum), max(maximum)
        {

        }

        vec3 min{}, max{};

    };



    class ShapeConcept
    {
    public:
        virtual ~ShapeConcept() = default;

        virtual bool is_inside(const vec3& p) const = 0;
        virtual vec3 closest(const vec3& p) const = 0;

        virtual bool is_overlap(const Sphere& a) const = 0;
        virtual bool is_overlap(const AABB& a) const = 0;
        virtual bool is_overlap(const Plane& a) const = 0;
        virtual bool is_overlap(const Segment& a) const = 0;

        // ... Potentially more polymorphic operations
    };

    class PrimerInsideStrategy;
    class PrimerClosesterStrategy;
    class PrimerOverlaperStrategy;

    template< typename ShapeT
        , typename InsideStrategy = PrimerInsideStrategy
        , typename ClosesterStrategy = PrimerClosesterStrategy
        , typename OverlaperStrategy = PrimerOverlaperStrategy >
    class ShapeModel : public ShapeConcept
    {
    public:
        explicit ShapeModel(ShapeT shape
            , InsideStrategy insider = PrimerInsideStrategy{}
            , ClosesterStrategy closester = PrimerClosesterStrategy{}
            , OverlaperStrategy overlaper = PrimerOverlaperStrategy{}
        )
            : shape_{ std::move(shape) }
            , insider_{ std::move(insider) }
            , closester_{ std::move(closester) }
            , overlaper_{ std::move(overlaper) }
        {}

        bool is_inside(const vec3& p) const override { return insider_(p, shape_); }
        vec3 closest(const vec3& p) const override { return closester_(p, shape_); }

        bool is_overlap(const Sphere& s) const override { return overlaper_(shape_, s); }
        bool is_overlap(const AABB& s) const override { return overlaper_(shape_, s); }
        bool is_overlap(const Plane& s) const override { return overlaper_(shape_, s); }
        bool is_overlap(const Segment& s) const override { return overlaper_(shape_, s); }

    private:
        ShapeT shape_;
        InsideStrategy insider_;
        ClosesterStrategy closester_;
        OverlaperStrategy overlaper_;
    };



    bool is_inside(const vec3& p, const Sphere& s);
    bool is_inside(const vec3& p, const AABB& aabb);
    bool is_inside(const vec3& p, const Plane& plane);
    bool is_inside(const vec3& p, const Segment& seg);
    bool is_inside(const vec3& p, const Ray& ray);
    bool is_inside(
        const vec3& p,
        const vec3& a, const vec3& b, const vec3& c
    );


    class PrimerInsideStrategy
    {
    public:
        explicit PrimerInsideStrategy( /* Drawing related arguments */)
        {}

        template<typename T>
        bool operator()(const vec3& p, const T& s) const { return is_inside(p, s); }
        //bool operator()(const vec3& p, const AABB& aabb) const { return is_inside(p, aabb); }
        //bool operator()(const vec3& p, const Plane& plane) const { return is_inside(p, plane); }
        //bool operator()(const vec3& p, const Segment& seg) const { return is_inside(p, seg); }
        //bool operator()(const vec3& p, const Ray& ray) const { return is_inside(p, ray); }
        bool operator()(
            const vec3& p,
            const vec3& a, const vec3& b, const vec3& c) const 
        {
            return is_inside(p, a, b, c); 
        }

    private:
        /* Drawing related data members, e.g., colors, textures, ... */
    };


    vec3 closest(const vec3& p, const Sphere& s);
    vec3 closest(const vec3& p, const AABB& aabb);
    vec3 closest(const vec3& p, const Plane& plane);
    vec3 closest(const vec3& p, const Segment& seg);
    vec3 closest(const vec3& p, const Ray& ray);
    vec3 closest(
        const vec3& p,
        const vec3& a, const vec3& b, const vec3& c);
    vec3 closest(
        const vec3& p,
        const vec3& a, const vec3& b, const vec3& c, const vec3& d
    );

    class PrimerClosesterStrategy
    {
    public:
        explicit PrimerClosesterStrategy()
        {}

        template<typename T>
        vec3 operator()(const vec3& p, const T& s) const { return closest(p, s); }
        //vec3 operator()(const vec3& p, const AABB& aabb) const { return closest(p, aabb); }
        //vec3 operator()(const vec3& p, const Plane& plane) const { return closest(p, plane); }
        //vec3 operator()(const vec3& p, const Segment& seg) const { return closest(p, seg); }
        //vec3 operator()(const vec3& p, const Ray& ray) const { return closest(p, ray); }
        vec3 operator()(
            const vec3& p,
            const vec3& a, const vec3& b, const vec3& c) const
        {
            return closest(p, a, b, c);
        }
        vec3 operator()(
            const vec3& p,
            const vec3& a, const vec3& b, const vec3& c, const vec3& d) const
        {
            return closest(p, a, b, c, d);
        }

    private:

    };



    std::pair<bool, Ray> intersect_two_planes(
        const Plane& plane0,
        const Plane& plane1
    );
    std::pair<bool, vec3> intersect_three_planes(
        const Plane& plane0,
        const Plane& plane1,
        const Plane& plane2
    );
    bool is_overlap(const Sphere& s1, const Sphere& s2);
    bool is_overlap(const Sphere& sphere, const AABB& aabb);
    bool is_overlap(const Sphere& sphere, const Plane& plane);
    bool is_overlap(const AABB& aabb1, const AABB& aabb2);
    bool is_overlap(const AABB& aabb, const Plane& plane);
    bool is_overlap(const Plane& plane1, const Plane& plane2);
    bool is_overlap(
        const Sphere& s,
        const vec3& a, const vec3& b, const vec3& c
    );
    bool is_overlap(
        const AABB& aabb,
        const vec3& a, const vec3& b, const vec3& c
    );
    bool is_overlap(
        const Plane& plane,
        const vec3& A, const vec3& B, const vec3& C
    );
    bool is_overlap(
        const vec3& A, const vec3& B, const vec3& C,
        const vec3& P, const vec3& Q, const vec3& R
    );
    bool is_overlap(const Segment& seg0, const Segment& seg1);
    bool is_overlap(const Segment& seg, const Sphere& sphere);
    bool is_overlap(const Segment& seg, const AABB& aabb);
    bool is_overlap(const Segment& seg, const Plane& plane);
    bool is_overlap(
        const Segment& seg,
        const vec3& ta, const vec3& tb, const vec3& tc
    );
    bool is_overlap(
        const std::vector<vec3>& shape1,
        const std::vector<vec3>& shape2
    );

    std::tuple<bool, double, double> raycast(const Ray& ray0, const Ray& ray1);
    std::pair<bool, double> raycast(const Sphere& sphere, const Ray& ray);
    std::pair<bool, double> raycast(const AABB& aabb, const Ray& ray);
    std::pair<bool, double> raycast(const Plane& plane, const Ray& ray);
    std::pair<bool, double> raycast(
        const vec3& ta, const vec3& tb, const vec3& tc,
        const Ray& ray
    );


    class PrimerOverlaperStrategy
    {
    public:
        explicit PrimerOverlaperStrategy()
        {}

        template<typename T1, typename T2>
        bool operator()(const T1& s1, const T2& s2) const { return is_overlap(s1, s2); }

        //bool operator()(const Sphere& s1, const Sphere& s2) const { return is_overlap(s1, s2); }
        //bool operator()(const Sphere& s, const AABB& aabb) const { return is_overlap(s, aabb); }
        //bool operator()(const AABB& aabb, const Sphere& s) const { return is_overlap(s, aabb); }
        //bool operator()(const Sphere& s, const Plane& plane) const { return is_overlap(s, plane); }
        //bool operator()(const Plane& plane, const Sphere& s) const { return is_overlap(s, plane); }
        //bool operator()(const AABB& aabb1, const AABB& aabb2) const { return is_overlap(aabb1, aabb2); }
        //bool operator()(const AABB& aabb, const Plane& plane) const { return is_overlap(aabb, plane); }
        //bool operator()(const Plane& plane, const AABB& aabb) const { return is_overlap(aabb, plane); }
        //bool operator()(const Plane& plane1, const Plane& plane2) const { return is_overlap(plane1, plane2); }
        bool operator()(
            const Sphere& s,
            const vec3& a, const vec3& b, const vec3& c
        ) const {
            return is_overlap(s, a, b, c);
        }
        bool operator()(
            const AABB& aabb,
            const vec3& a, const vec3& b, const vec3& c
        ) const {
            return is_overlap(aabb, a, b, c);
        }
        bool operator()(
            const Plane& plane,
            const vec3& A, const vec3& B, const vec3& C
        ) const {
            return is_overlap(plane, A, B, C);
        }
        bool operator()(
            const vec3& A, const vec3& B, const vec3& C,
            const vec3& P, const vec3& Q, const vec3& R
        ) const {
            return is_overlap(A, B, C, P, Q, R);
        }
        //bool operator()(const Segment& seg0, const Segment& seg1) const { return is_overlap(seg0, seg1); }
        //bool operator()(const Segment& seg, const Sphere& sphere) const { return is_overlap(seg, sphere); }
        //bool operator()(const Sphere& sphere, const Segment& seg) const { return is_overlap(seg, sphere); }
        //bool operator()(const Segment& seg, const AABB& aabb) const { return is_overlap(seg, aabb); }
        //bool operator()(const AABB& aabb, const Segment& seg) const { return is_overlap(seg, aabb); }
        //bool operator()(const Segment& seg, const Plane& plane) const { return is_overlap(seg, plane); }
        //bool operator()(const Plane& plane, const Segment& seg) const { return is_overlap(seg, plane); }
        bool operator()(
            const Segment& seg,
            const vec3& ta, const vec3& tb, const vec3& tc
        ) const {
            return is_overlap(seg, ta, tb, tc);
        }
        bool operator()(
            const std::vector<vec3>& shape1,
            const std::vector<vec3>& shape2
        ) const {
            return is_overlap(shape1, shape2);
        }


    private:

    };

    


}

