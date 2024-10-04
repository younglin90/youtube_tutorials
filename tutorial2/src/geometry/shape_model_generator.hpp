#pragma once

#include <Eigen/Core>
#include <tuple>
#include <array>

#include "./shape_model.hpp"

namespace SimulMan {

    class ShapeModelGenerator {
    private:
        using vec2 = Eigen::Vector2d;
        using vec3 = Eigen::Vector3d;
        using vec4 = Eigen::Vector4d;

    public:


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
		static ShapeModel AABB(
			vec3 v0, vec3 v1, vec3 v2, vec3 v3, vec3 v4, vec3 v5, vec3 v6, vec3 v7
		) {
			ShapeModel shape;
			shape.pos = { v0,v1,v2,v3,v4,v5,v6,v7 };
			//shape.c2v = {
			//	{0, 1, 2, 5},
			//	{0, 2, 3, 7},
			//	{0, 5, 4, 7},
			//	{2, 5, 6, 7},
			//	{0, 2, 5, 7}
			//};
			shape.f2v = {
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

			vec3 max = { -1.e200,-1.e200 ,-1.e200 };
			vec3 min = { 1.e200,1.e200 ,1.e200 };
			for (const auto& p : shape.pos) {
				for (int j = 0; j < 3; ++j) {
					max[j] = std::max(max[j], p[j]);
					min[j] = std::min(min[j], p[j]);
				}
			}

			//shape.shape = SimulMan::AABB(min, max);

			return shape;
		}



		////        5
		////       /|\
		////      / | \
		////     /  |  \
		////    3 --+-- 4
		////    |   |   |
		////    |   2   |
		////    |  / \  |
		////    | /   \ |
		////    |/     \|
		////    0 ----- 1
		//static ShapeModel prism(
		//	vec3 v0, vec3 v1, vec3 v2, vec3 v3, vec3 v4, vec3 v5
		//) {
		//	ShapeModel shape;
		//	shape.pos = { v0,v1,v2,v3,v4,v5 };
		//	//rigid.c2v = {
		//	//	{0, 1, 2, 3},
		//	//	{1, 2, 3, 4},
		//	//	{2, 3, 4, 5}
		//	//};
		//	shape.f2v = {
		//		{ 0, 1, 2 },
		//		{ 3, 4, 5 },
		//		{ 0, 1, 3 },
		//		{ 1, 3, 4 },
		//		{ 0, 2, 3 },
		//		{ 2, 3, 5 },
		//		{ 1, 2, 4 },
		//		{ 2, 4, 5 }
		//	};


		//	return shape;
		//}



		static ShapeModel sphere(
			vec3 center, double radius, int segments
		) {

			ShapeModel shape;
			// 정점 생성
			shape.pos.clear();
			for (int i = 0; i <= segments; ++i) {
				double phi = 3.141592 * double(i) / double(segments);
				for (int j = 0; j <= segments; ++j) {
					double theta = 2.0 * 3.141592 * double(j) / double(segments);
					double x = center[0] + radius * std::sin(phi) * std::cos(theta);
					double y = center[1] + radius * std::sin(phi) * std::sin(theta);
					double z = center[2] + radius * std::cos(phi);
					shape.pos.push_back(Eigen::Vector3d(x, y, z));
				}
			}
			shape.pos.push_back(Eigen::Vector3d(0, 0, 0));
			uint32_t vc = shape.pos.size() - 1;

			// 면 생성
			shape.f2v.clear();
			//shape.c2v.clear();
			for (int i = 0; i < segments; ++i) {
				for (int j = 0; j < segments; ++j) {
					uint32_t v1 = i * (segments + 1) + j;
					uint32_t v2 = v1 + 1;
					uint32_t v3 = (i + 1) * (segments + 1) + j;
					uint32_t v4 = v3 + 1;

					shape.f2v.push_back({ v1,v3,v2 });
					shape.f2v.push_back({ v2,v3,v4 });

					//rigid.c2v.push_back({ v1,v3,v2,vc });
					//rigid.c2v.push_back({ v2,v3,v4,vc });
				}
			}

			//shape.shape = SimulMan::Sphere(center, radius);

			return shape;
		}



    };


}

