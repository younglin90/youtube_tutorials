#include <stdlib.h>
#include <variant>
#include <Eigen/Core>

#include "benchmark/benchmark.h"



namespace ���������� {

    enum ShapeType {
        Sphere0,
        AABB0,
        Triangle0
    };

    class Shape {
    protected:
        explicit Shape(ShapeType type) :
            type_(type) {}

    public:
        virtual ~Shape() = default;
        ShapeType getType() const {
            return type_;
        }
        virtual bool is_overlap(const Shape& other) const = 0;

    private:
        ShapeType type_;
    };

    class Sphere : public Shape {
    public:
        Sphere() : Shape(ShapeType::Sphere0) {}

        Eigen::Vector3d c;
        double r;

        bool is_overlap(const Shape& other) const override {
            switch (other.getType()) {
            case ShapeType::Sphere0 :
                return true;
            case ShapeType::AABB0:
                return true;
            case ShapeType::Triangle0:
                return true;
            }
        }
    };
    class AABB : public Shape {
    public:
        AABB() : Shape(ShapeType::AABB0) {}

        Eigen::Vector3d min, max;

        bool is_overlap(const Shape& other) const override {
            switch (other.getType()) {
            case ShapeType::Sphere0:
                return true;
            case ShapeType::AABB0:
                return true;
            case ShapeType::Triangle0:
                return true;
            }
        }
    };
    class Triangle : public Shape {
    public:
        Triangle() : Shape(ShapeType::Triangle0) {}

        Eigen::Vector3d a, b, c;

        bool is_overlap(const Shape& other) const override {
            switch (other.getType()) {
            case ShapeType::Sphere0:
                return true;
            case ShapeType::AABB0:
                return true;
            case ShapeType::Triangle0:
                return true;
            }
        }
    };

}

void �����������Լ�(benchmark::State& state) {
    ����������::Sphere sphere;
    ����������::AABB aabb;
    for (auto _ : state) {
        sphere.is_overlap(aabb);
    }
}

namespace OOP���� {

    class Shape {
    public:
        Shape() = default;
        virtual ~Shape() = default;
        virtual bool is_overlap(const std::unique_ptr<Shape>& other) const = 0;
    };

    class Sphere : public Shape {
    public:
        bool is_overlap(const std::unique_ptr<Shape>& other) const override {
            return true;
        }
    };
    class AABB : public Shape {
    public:
        bool is_overlap(const std::unique_ptr<Shape>& other) const override {
            return true;
        }
    };
    class Triangle : public Shape {
    public:
        bool is_overlap(const std::unique_ptr<Shape>& other) const override {
            return true;
        }
    };

    bool is_overlap(const std::unique_ptr<Shape>& shape1, const std::unique_ptr<Shape>& shape2) {
        return shape1->is_overlap(shape2);
    }


}

void OOP�����Լ�(benchmark::State& state) {
    std::unique_ptr<OOP����::Shape> sphere = std::make_unique<OOP����::Sphere>();
    std::unique_ptr<OOP����::Shape> aabb = std::make_unique<OOP����::AABB>();
    for (auto _ : state) {
        OOP����::is_overlap(aabb, sphere);
    }
}



namespace CRTP���� {

    template<typename Derived>
    class Shape {
    public:
        bool is_overlap(const Shape& other) const {
            return static_cast<const Derived*>(this)->is_overlap_impl(other);
        }
    };

    class Sphere : public Shape<Sphere> {
    public:
        bool is_overlap_impl(const Shape& other) const {
            return true;
        }
        Eigen::Vector3d c;
        double r;
    };
    class AABB : public Shape<Sphere> {
    public:
        bool is_overlap_impl(const Shape& other) const {
            return true;
        }
        Eigen::Vector3d min, max;
    };
    class Triangle : public Shape<Sphere> {
    public:
        bool is_overlap_impl(const Shape& other) const {
            return true;
        }
        Eigen::Vector3d a, b, c;
    };


}

void CRTP�����Լ�(benchmark::State& state) {
    CRTP����::Sphere sphere;
    CRTP����::AABB aabb;
    for (auto _ : state) {
        sphere.is_overlap(aabb);
    }
}



//=================================================================
namespace �湮������ {


    class Sphere
    {
    public:
        Eigen::Vector3d c;
        double r;
    };
    class AABB
    {
    public:
        Eigen::Vector3d min, max;
    };
    class Triangle
    {
    public:
        Eigen::Vector3d a, b, c;
    };


    using Shape = std::variant<Sphere, AABB>;

    class Overlaper
    {
    public:
        template<typename T, typename U>
        void operator()(T const& shape1, U const& shape2) const {
            if constexpr (std::is_same_v<T, Sphere> && std::is_same_v<U, Sphere>) {
            }
            else if constexpr (std::is_same_v<T, Sphere> && std::is_same_v<U, AABB>) {
            }
            else if constexpr (std::is_same_v<T, AABB> && std::is_same_v<U, Sphere>) {
            }
            else if constexpr (std::is_same_v<T, AABB> && std::is_same_v<U, AABB>) {
            }
        }
    };


    void is_overlap(Shape const& shape1, Shape const& shape2)
    {
        std::visit(Overlaper{}, shape1, shape2);
    }

}

void �湮�������Լ�(benchmark::State& state) {
    �湮������::Shape sphere = �湮������::Sphere{};
    �湮������::Shape aabb = �湮������::AABB{};
    for (auto _ : state) {
        �湮������::is_overlap(sphere, aabb);
    }
}




BENCHMARK(�����������Լ�)->Iterations(10000000);
BENCHMARK(OOP�����Լ�)->Iterations(10000000);
BENCHMARK(CRTP�����Լ�)->Iterations(10000000);
BENCHMARK(�湮�������Լ�)->Iterations(10000000);

BENCHMARK_MAIN();