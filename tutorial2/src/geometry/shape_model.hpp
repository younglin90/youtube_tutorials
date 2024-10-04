#pragma once

#include <Eigen/Core>
#include <tuple>
#include <array>

#include "./shapes.hpp"

namespace SimulMan {

    class ShapeModel {
    private:
        using vec3 = Eigen::Vector3d;
        using idx_t = size_t;

    public:
        std::vector<vec3> pos;
        std::vector<std::vector<idx_t>> f2v;

        //Shape shape;
    };


}

