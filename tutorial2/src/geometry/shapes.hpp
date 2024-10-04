#pragma once

#include <Eigen/Core>
#include <tuple>
#include <array>
#include <map>

namespace SimulMan {

    using vec3 = Eigen::Vector3d;
    using real_t = double;
    using index_t = size_t;

    using index2_t = std::vector<std::vector<index_t>>;
    using vec_vec3_t = std::vector<vec3>;
    using triangluation_t = std::tuple<vec_vec3_t, vec_vec3_t, vec_vec3_t, index2_t>;


    class ShapeConcept {
    public:
        virtual ~ShapeConcept() = default;

        virtual void draw() const = 0;
        virtual triangluation_t trianglation() const = 0;       

    };

    //struct DefaultDrawer {
    //    template<typename T>
    //    void operator()(T const& obj) const {
    //        draw(obj);
    //    }
    //};

    template<typename ShapeT, 
        typename DrawStrategy,
        typename TriangulatorStrategy
    >
    class ShapeModel : public ShapeConcept {
    public:
        //using DrawStrategy = std::function<void(ShapeT const&)>;

        explicit ShapeModel(ShapeT shape, DrawStrategy drawer, TriangulatorStrategy triangulator) :
            shape_{ std::move(shape) },
            drawer_{ std::move(drawer) },
            triangulator_{ std::move(triangulator) } {

        }

        void draw() const override { drawer_(shape_); }
        triangluation_t trianglation() const override {
            return triangulator_(shape_); 
        }


    private:
        ShapeT shape_;
        DrawStrategy drawer_;
        TriangulatorStrategy triangulator_;
    };


    class Easy3dDrawStrategy;
    class Easy3dTriangulatorStrategy;


    // 타입소거
    class Shape {
    protected:
        using vec2 = Eigen::Vector2d;
        using vec3 = Eigen::Vector3d;
        using vec4 = Eigen::Vector4d;
        using pos_t = std::vector<vec3>;
        using idx_t = std::vector<std::vector<int>>;

    public:
        template<typename ShapeT, 
            typename DrawStrategy = Easy3dDrawStrategy,
            typename TriangulatorStrategy = Easy3dTriangulatorStrategy>
        Shape(ShapeT shape, 
            DrawStrategy drawer = Easy3dDrawStrategy{}, 
            TriangulatorStrategy triangulator = Easy3dTriangulatorStrategy{}) {
            using Model = ShapeModel<ShapeT, DrawStrategy, TriangulatorStrategy>;
            pimpl_ = std::make_unique<Model>(std::move(shape), std::move(drawer), std::move(triangulator));
        }


    private:
        std::unique_ptr<ShapeConcept> pimpl_;

        //friend void draw(const Shape& shape) {
        //    shape.pimpl_->draw();
        //}
        //friend std::pair<std::vector<vec3>, index2_t> trianglation(const Shape& shape) {
        //    return shape.pimpl_->trianglation();
        //}
        friend void draw(const Shape& shape) {
            shape.pimpl_->draw();
        }
        friend triangluation_t trianglation(const Shape& shape) {
            return shape.pimpl_->trianglation();
        }


    };


    // 1d
    struct Point {
        vec3 p{};

        void test() {
            std::cout << "Point" << std::endl;
        }
    };


    // 2d
    struct Triangle {
        vec3 a{}, b{}, c{};

        void test() {
            std::cout << "Triangle" << std::endl;

        }
    };

    struct Square {
        vec3 a{}, b{}, c{}, d{};

        void test() {
            std::cout << "Square" << std::endl;

        }
    };

    struct Plane {
        vec3 n{};
        real_t d{};

        void test() {
            std::cout << "Plane" << std::endl;

        }
    };

    struct Ray {
        vec3 p{}, n{};

        void test() {
            std::cout << "Ray" << std::endl;

        }
    };

    struct Segment {
        vec3 a{}, b{};

        void test() {
            std::cout << "Segment" << std::endl;

        }
    };


    // 3d
    struct Sphere {
        vec3 c{};
        real_t r{};

        void test() {
            std::cout << "Sphere" << std::endl;

        }
    };

    struct AABB {
        AABB(const vec3& minimum, const vec3& maximum) :
            min(minimum), max(maximum)
        {

        }

        vec3 min{}, max{};

        void test() {
            std::cout << "AABB" << std::endl;

        }
    };


    


    class Easy3dDrawStrategy {
    public:
        explicit Easy3dDrawStrategy() {

        }

        //void operator()(Circle const& circle) const {

        //}
        //void operator()(Square const& square) const {

        //}
        void operator()(AABB const& aabb) const {
        }
        void operator()(Sphere const& sphere) const {
        }
        void operator()(Point const& point) const {
        }
        void operator()(Triangle const&) const {
        }
        void operator()(Plane const&) const {
        }
        void operator()(Segment const&) const {
        }
        void operator()(Ray const&) const {
        }
    };


    class Easy3dTriangulatorStrategy {
    public:
        explicit Easy3dTriangulatorStrategy() {

        }

        triangluation_t operator()(AABB const& aabb) const {
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
            vec3 radius = 0.5 * (aabb.max - aabb.min);
            vec3 center = aabb.max - radius;
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
        triangluation_t operator()(Sphere const& sphere) const {

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
                p *= sphere.r;
                p += sphere.c;
            }

            std::vector<vec3> real_pos;
            std::vector<vec3> line_pos;


            return std::make_tuple(real_pos, line_pos, pos, f2v);
        }


        triangluation_t operator()(Point const& point) const {

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


            std::vector<vec3> real_pos = { point.p };
            std::vector<vec3> line_pos;
            std::vector<vec3> pos;
            index2_t f2v;

            return std::make_tuple(real_pos, line_pos, pos, f2v);
        }



        triangluation_t operator()(Triangle const& triangle) const {

            std::vector<vec3> pos = { triangle.a, triangle.b, triangle.c };
            index2_t f2v = { { 0, 1, 2 } };
            std::vector<vec3> real_pos;
            std::vector<vec3> line_pos;

            return std::make_tuple(real_pos, line_pos, pos, f2v);
        }

        triangluation_t operator()(Square const& square) const {
            
            // 정점 생성 (시계 방향)
            std::vector<vec3> pos = {
                square.a,  // 좌하단
                square.b,   // 우하단
                square.c,    // 우상단
                square.d    // 좌상단
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

        triangluation_t operator()(Plane const& plane) const {

            double size = 1.0;

            // 정규화된 법선 벡터
            vec3 normal = plane.n.normalized();

            // 평면 위의 한 점 찾기
            vec3 point = vec3(normal[0] * plane.d, normal[1] * plane.d, normal[2] * plane.d);

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

        triangluation_t operator()(Ray const& ray) const {

            std::vector<vec3> pos;
            index2_t f2v;
            std::vector<vec3> real_pos;
            std::vector<vec3> line_pos = { ray.p, ray.p + 5.0 * ray.n };

            return std::make_tuple(real_pos, line_pos, pos, f2v);
        }

        triangluation_t operator()(Segment const& segment) const {

            std::vector<vec3> pos;
            index2_t f2v;
            std::vector<vec3> real_pos;
            std::vector<vec3> line_pos = { segment.a, segment.b };

            return std::make_tuple(real_pos, line_pos, pos, f2v);
        }
    };





    //class Shape {
    //protected:
    //    using vec2 = Eigen::Vector2d;
    //    using vec3 = Eigen::Vector3d;
    //    using vec4 = Eigen::Vector4d;
    //    using pos_t = std::vector<vec3>;
    //    using idx_t = std::vector<std::vector<int>>;

    //public:
    //    virtual ~Shape() = default;
    //    virtual void draw() const = 0;

    //    virtual pos_t& get_pos() = 0;
    //    virtual idx_t& get_f2v() = 0;

    //    struct Drawer {
    //        pos_t pos;
    //        idx_t f2v;
    //        idx_t c2v;
    //    } drawer;

    //};

    //class ShapeVisitor {
    //public:
    //    virtual ~ShapeVisitor() = default;

    //    virtual void visit(AABB const&) const = 0;
    //    virtual void visit(Sphere const&) const = 0;
    //};

    //class Draw : public ShapeVisitor {
    //public:
    //    void visit(AABB const&) const override;
    //    void visit(Sphere const&) const override;
    //};

  //  struct AABB : public Shape {
  //      AABB() = default;
  //      AABB(const vec3& min_in, const vec3& max_in) :
  //          min(min_in), max(max_in)
  //      {
  //      //       4 -------- 5
		////      /|         /|
		////     / |        / |
		////    /  |       /  |
		////   7 -------- 6   |
		////   |   |      |   |
		////   |   0 -----|-- 1
		////   |  /       |  /
		////   | /        | /
		////   |/         |/
		////   3 -------- 2	
  //          vec3 radius = 0.5 * (max - min);
  //          vec3 center = max - radius;
  //          vec3 v0 = { center[0] - radius[0], center[1] - radius[1], center[2] - radius[2] };
  //          vec3 v1 = { center[0] + radius[0], center[1] - radius[1], center[2] - radius[2] };
  //          vec3 v2 = { center[0] + radius[0], center[1] + radius[1], center[2] - radius[2] };
  //          vec3 v3 = { center[0] - radius[0], center[1] + radius[1], center[2] - radius[2] };
  //          vec3 v4 = { center[0] - radius[0], center[1] - radius[1], center[2] + radius[2] };
  //          vec3 v5 = { center[0] + radius[0], center[1] - radius[1], center[2] + radius[2] };
  //          vec3 v6 = { center[0] + radius[0], center[1] + radius[1], center[2] + radius[2] };
  //          vec3 v7 = { center[0] - radius[0], center[1] + radius[1], center[2] + radius[2] };
  //          drawer.pos = { v0,v1,v2,v3,v4,v5,v6,v7 };
  //          drawer.f2v = {
  //              { 1, 0, 2 },
  //              { 0, 1, 5 },
  //              { 1, 2, 5 },
  //              { 2, 0, 3 },
  //              { 3, 0, 7 },
  //              { 2, 3, 7 },
  //              { 4, 0, 5 },
  //              { 0, 4, 7 },
  //              { 4, 5, 7 },
  //              { 5, 2, 6 },
  //              { 6, 2, 7 },
  //              { 5, 6, 7 }
  //          };
  //      }

  //      vec3 min{}, max{};

  //      void draw() const override {

  //      }
  //      pos_t& get_pos() override {
  //          return drawer.pos;
  //      }
  //      idx_t& get_f2v() override {
  //          return drawer.f2v;
  //      }
  //  };
  //  struct Sphere : public Shape {
  //      Sphere() = default;
  //      Sphere(const vec3& c_in, const double& r_in) :
  //          c(c_in), r(r_in) 
  //      {

  //          const float t = (1.0 + std::sqrt(5.0)) / 2.0;  // 황금비

  //          // 정이십면체의 초기 정점
  //          drawer.pos = {
  //              {-1,  t,  0}, { 1,  t,  0}, {-1, -t,  0}, { 1, -t,  0},
  //              { 0, -1,  t}, { 0,  1,  t}, { 0, -1, -t}, { 0,  1, -t},
  //              { t,  0, -1}, { t,  0,  1}, {-t,  0, -1}, {-t,  0,  1}
  //          };

  //          // 각 삼각형 면을 정의하는 인덱스
  //          drawer.f2v = {
  //              {0, 11, 5}, {0, 5, 1}, {0, 1, 7}, {0, 7, 10}, {0, 10, 11},
  //              {1, 5, 9}, {5, 11, 4}, {11, 10, 2}, {10, 7, 6}, {7, 1, 8},
  //              {3, 9, 4}, {3, 4, 2}, {3, 2, 6}, {3, 6, 8}, {3, 8, 9},
  //              {4, 9, 5}, {2, 4, 11}, {6, 2, 10}, {8, 6, 7}, {9, 8, 1}
  //          };

  //          // 각 정점을 구면에 맞게 정규화
  //          for (auto& vertex : drawer.pos) {
  //              vertex.normalize();
  //          }


  //          auto getMidPoint = [&](int v1, int v2, std::map<std::pair<int, int>, int>& midPointCache) {
  //              std::pair<int, int> key = std::minmax(v1, v2);

  //              if (midPointCache.find(key) != midPointCache.end()) {
  //                  return midPointCache[key];
  //              }

  //              vec3 midPoint = vec3(
  //                  (drawer.pos[v1][0] + drawer.pos[v2][0]) / 2.0,
  //                  (drawer.pos[v1][1] + drawer.pos[v2][1]) / 2.0,
  //                  (drawer.pos[v1][2] + drawer.pos[v2][2]) / 2.0
  //              ).normalized();

  //              drawer.pos.push_back(midPoint);
  //              int index = drawer.pos.size() - 1;
  //              midPointCache[key] = index;

  //              return index;
  //              };

  //          auto subdivide = [&]() {
  //              idx_t newFaces;
  //              std::map<std::pair<int, int>, int> midPointCache;

  //              for (const auto& face : drawer.f2v) {
  //                  int a = getMidPoint(face[0], face[1], midPointCache);
  //                  int b = getMidPoint(face[1], face[2], midPointCache);
  //                  int c = getMidPoint(face[2], face[0], midPointCache);

  //                  newFaces.push_back({ face[0], a, c });
  //                  newFaces.push_back({ face[1], b, a });
  //                  newFaces.push_back({ face[2], c, b });
  //                  newFaces.push_back({ a, b, c });
  //              }

  //              drawer.f2v = newFaces;

  //              };

  //          subdivide();
  //          subdivide();
  //          //subdivide();


  //      }

  //      vec3 c{};
  //      double r{};

  //      void draw() const override {

  //      }
  //      pos_t& get_pos() override {
  //          return drawer.pos;
  //      }
  //      idx_t& get_f2v() override {
  //          return drawer.f2v;
  //      }
  //  };
  //  struct Plane : public Shape {
  //      // plane: x.normal = d
  //      Plane() = default;
  //      Plane(const vec3& n_in, const double& d_in) :
  //          n(n_in), d(d_in) {}

  //      vec3 n{};
  //      double d{};

  //      void draw() const override {

  //      }
  //      pos_t& get_pos() override {
  //          return drawer.pos;
  //      }
  //      idx_t& get_f2v() override {
  //          return drawer.f2v;
  //      }
  //  };
  //  struct Segment : public Shape {
  //      Segment() = default;
  //      Segment(const vec3& a_in, const vec3& b_in) :
  //          a(a_in), b(b_in) {}

  //      vec3 a{}, b{};

  //      void draw() const override {

  //      }
  //      pos_t& get_pos() override {
  //          return drawer.pos;
  //      }
  //      idx_t& get_f2v() override {
  //          return drawer.f2v;
  //      }
  //  };
  //  struct Ray : public Shape {
  //      Ray() = default;
  //      Ray(const vec3& p_in, const vec3& n_in) :
  //          p(p_in), n(n_in) {}

  //      vec3 p{}, n{};

  //      void draw() const override {

  //      }
  //      pos_t& get_pos() override {
  //          return drawer.pos;
  //      }
  //      idx_t& get_f2v() override {
  //          return drawer.f2v;
  //      }
  //  };

}

