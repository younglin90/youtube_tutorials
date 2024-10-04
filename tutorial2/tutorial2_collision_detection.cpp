#include <iostream>
#include <array>
#include <vector>
#include <Eigen/Dense>

//#include "./src/renderer.hpp"
//#include "./src/rigid_body.hpp"



//#include "./src/geometry/geometry.hpp"
//#include "./src/geometry/shape_model.hpp"
//#include "./src/geometry/shape_model_generator.hpp"

//
//#include "./src/geometry/shapes.hpp"
//#include "./src/renderer.hpp"

//#include "./src/geometry/shapes_External_Polymorphism.hpp"
#include "./src/geometry/shapes_visitor.hpp"
#include "./src/geometry/inside.hpp"
#include "./src/geometry/closest.hpp"
#include "./src/geometry/overlap.hpp"

#include "./src/renderer.hpp"




int main() {

	//shapes.push_back(SimulMan::Plane{ { 0, 0, 0 } , 1.0 });
	//shapes.push_back(SimulMan::vec3{ 0, 0, 0 });

	//SimulMan::Renderer renderer(
	//	SimulMan::AABB{ { -0.5, -0.5, -0.5 } , { 0.5, 0.5, 0.5 } },
	//	SimulMan::vec3{ 0, 0, 0 },
	//	true);
	//SimulMan::Renderer renderer(
	//	SimulMan::Sphere{ { -0.8, -0.8, -0.8 } , 0.6 },
	//	SimulMan::vec3{ 0, 0, 0 },
	//	true);
	//SimulMan::Renderer renderer(
	//	SimulMan::Ray{ { -0.5, -0.5, -0.5 } , { 1.0, 0.0, 0.0 } },
	//	SimulMan::vec3{ 0, 0, 0 },
	//	true);
	//SimulMan::Renderer renderer(
	//	SimulMan::Segment{ { 0, 0, 0 } , { 1.0, 0.0, 0.0 } },
	//	SimulMan::vec3{ 0, 0, 0 },
	//	true);
	//SimulMan::Renderer renderer(
	//	SimulMan::Plane{ { 1, 1, 1 } , 1.0 },
	//	SimulMan::vec3{ 0, 0, 0 },
	//	true);

	//구 구
	//SimulMan::Renderer renderer(
	//	SimulMan::Sphere{ { -0.8, -0.8, -0.8 } , 0.6 },
	//	SimulMan::Sphere{ { -0.8, -0.8, -0.8 } , 0.6 },
	//	SimulMan::Renderer::Tester::Overlap);
	//구 aabb
	//SimulMan::Renderer renderer(
	//	SimulMan::Sphere{ { -0.8, -0.8, -0.8 } , 0.6 },
	//	SimulMan::AABB{ { -0.5, -0.5, -0.5 } , { 0.5, 0.5, 0.5 } },
	//	SimulMan::Renderer::Tester::Overlap);
	//구 평면
	//SimulMan::Renderer renderer(
	//	SimulMan::Sphere{ { -0.8, -0.8, -0.8 } , 0.6 },
	//	SimulMan::Plane{ { 1, 1, 1 } , 1.0 },
	//	SimulMan::Renderer::Tester::Overlap);
	//aabb aabb
	//SimulMan::Renderer renderer(
	//	SimulMan::AABB{ { -0.5, -0.5, -0.5 } , { 0.5, 0.5, 0.5 } },
	//	SimulMan::AABB{ { -0.5, -0.5, -0.5 } , { 0.5, 0.5, 0.5 } },
	//	SimulMan::Renderer::Tester::Overlap);
	//aabb 평면
	//SimulMan::Renderer renderer(
	//	SimulMan::Sphere{ { -0.8, -0.8, -0.8 } , 0.6 },
	//	SimulMan::Plane{ { 1, 1, 1 } , 1.0 },
	//	SimulMan::Renderer::Tester::Overlap);
	//평면 평면
	//SimulMan::Renderer renderer(
	//	SimulMan::Plane{ { 1, 1, 1 } , 1.0 },
	//	SimulMan::Plane{ { 1, 1, 1 } , 1.0 },
	//	SimulMan::Renderer::Tester::Overlap);
	//구 삼각형
	//SimulMan::Renderer renderer(
	//	SimulMan::Sphere{ { -0.8, -0.8, -0.8 } , 0.6 },
	//	SimulMan::Triangle{ { 0, 0, 0 } , { 1, 1, 0 }, { 0, 0, 1 } },
	//	SimulMan::Renderer::Tester::Overlap);
	//aabb 삼각형
	//SimulMan::Renderer renderer(
	//	SimulMan::AABB{ { -0.5, -0.5, -0.5 } , { 0.5, 0.5, 0.5 } },
	//	SimulMan::Triangle{ { 0, 0, 0 } , { 1, 1, 0 }, { 0, 0, 1 } },
	//	SimulMan::Renderer::Tester::Overlap);
	//평면 삼각형
	SimulMan::Renderer renderer(
		SimulMan::Plane{ { 1, 1, 1 } , 1.0 },
		SimulMan::Triangle{ { 0, 0, 0 } , { 1, 1, 0 }, { 0, 0, 1 } },
		SimulMan::Renderer::Tester::Overlap);
	//삼각형 삼각형
	//SimulMan::Renderer renderer(
	//	SimulMan::Triangle{ { 0, 0, 0 } , { 0, 1, 1 }, { 0, 0, 1 } },
	//	SimulMan::Triangle{ { 0, 0, 0 } , { 1, 1, 0 }, { 0, 0, 1 } },
	//	SimulMan::Renderer::Tester::Overlap);
	//선분 선분
	//SimulMan::Renderer renderer(
	//	SimulMan::Segment{ { 0, 0, 0 } , { 0, -1, -1 } },
	//	SimulMan::Segment{ { 0, 0, 0 } , { 0, 1, 1 } },
	//	SimulMan::Renderer::Tester::Overlap);
	//선분 구
	//SimulMan::Renderer renderer(
	//	SimulMan::Segment{ { 0, 0, 0 } , { 0, 1, 1 } },
	//	SimulMan::Sphere{ { -0.8, -0.8, -0.8 } , 0.6 },
	//	SimulMan::Renderer::Tester::Overlap);
	//선분 aabb
	//SimulMan::Renderer renderer(
	//	SimulMan::Segment{ { 0, 0, 0 } , { 0, 1, 1 } },
	//	SimulMan::AABB{ { -0.5, -0.5, -0.5 } , { 0.5, 0.5, 0.5 } },
	//	SimulMan::Renderer::Tester::Overlap);
	//선분 평면
	//SimulMan::Renderer renderer(
	//	SimulMan::Segment{ { 0, 0, 0 } , { 0, 1, 1 } },
	//	SimulMan::Plane{ { 1, 1, 1 } , 1.0 },
	//	SimulMan::Renderer::Tester::Overlap);
	//선분 삼각형
	//SimulMan::Renderer renderer(
	//	SimulMan::Segment{ { 0, 0, 0 } , { 0, 1, 1 } },
	//	SimulMan::Triangle{ { 0, 0, 0 } , { 1, 1, 0 }, { 0, 0, 1 } },
	//	SimulMan::Renderer::Tester::Overlap);
	






	////GJK
	//SimulMan::ConvexPolyhedra poly0 = { { 0, 0, 0 } , { 0, 1, 1 }, { 0, 0, 1 } };
	//SimulMan::ConvexPolyhedra poly1 = { { 0, 0, 0 } , { 1, 1, 0 }, { 0, 0, 1 } };

	//SimulMan::Renderer renderer(
	//	poly0,
	//	poly1,
	//	SimulMan::Renderer::Tester::Overlap);

	renderer.run();

	

	//std::vector<SimulMan::Shape> shapes;

	//shapes.push_back(SimulMan::Shape(
	//	SimulMan::AABB(SimulMan::vec3(0.5, 0.5, 0.5), SimulMan::vec3(1, 1, 1))
	//));
	//shapes.push_back(SimulMan::Shape(
	//	SimulMan::Point(SimulMan::vec3(2, 2, 2))
	//));
	////shapes.push_back(SimulMan::Shape(
	////	SimulMan::AABB(SimulMan::vec3(-1, -1, -1), SimulMan::vec3(-0.5, -0.5, -0.5))
	////));

	////shapes.push_back(SimulMan::Shape(
	////	SimulMan::Ray(SimulMan::vec3(-1, -1, -1), SimulMan::vec3(-1, -1, -1))
	////));
	////shapes.push_back(SimulMan::Shape(
	////	SimulMan::Segment(SimulMan::vec3(-1, -1, -1), SimulMan::vec3(1, 1, 1))
	////));
	////shapes.push_back(SimulMan::Shape(
	////	SimulMan::Point(SimulMan::vec3(2, 2, 2))
	////));
	////shapes.push_back(SimulMan::Shape(
	////	SimulMan::Plane(SimulMan::vec3(2, 2, 2), 0.5)
	////));
	////shapes.push_back(SimulMan::Shape(
	////	SimulMan::Sphere(SimulMan::vec3(-1, -1, -1), 0.5)
	////));
	////shapes.push_back(SimulMan::Shape(
	////	SimulMan::Sphere(SimulMan::vec3(1, 1, 1), 0.5)
	////));
	////shapes.push_back(SimulMan::Shape(
	////	SimulMan::AABB(SimulMan::vec3(0.5, 0.5, 0.5), SimulMan::vec3(1, 1, 1))
	////));


	//SimulMan::Renderer renderer(shapes);

	//renderer.run();


	//Shape shape(Circle{ 3.14 }, aaaa);

	//draw(shape);


	//using Shapes = std::vector<std::shared_ptr<SimulMan::Shape>>;

	//Shapes shapes;
	//shapes.push_back(std::make_unique<SimulMan::AABB>(
	//	Eigen::Vector3d(1, 1, 1), Eigen::Vector3d(3, 3, 3)
	//));
	//shapes.push_back(std::make_unique<SimulMan::Sphere>(
	//	Eigen::Vector3d(0, 0, 0), 0.5
	//));
	//shapes.push_back(std::make_unique<SimulMan::AABB>(
	//	Eigen::Vector3d(0, 0, 0), Eigen::Vector3d(1, 1, 1)
	//));



	////for (auto const& shape : shapes) {
	////	shape->draw();
	////}

 //   SimulMan::Renderer renderer;

	//renderer.shapes_ = shapes;
	//renderer.init();
 //   renderer.run();


	//auto aabb = SimulMan::ShapeModelGenerator::AABB(
	//	{ 0, 0, 0 }, // 0
	//    { 0.5, 0, 0 }, // 1
	//    { 0.5, 0.5, 0 }, // 2
	//    { 0, 0.5, 0 }, // 3
	//    { 0, 0, 0.5 }, // 4
	//    { 0.5, 0, 0.5 }, // 5
	//    { 0.5, 0.5, 0.5 }, // 6
	//    { 0, 0.5, 0.5 }  // 7
	//);
	

 //   RigidBodyGenerator gen;

 //   std::vector<RigidBody> rigids;
 //   rigids.push_back(gen.hexahedron(
 //       { 0, 0, 0 }, // 0
 //       { 0.5, 0, 0 }, // 1
 //       { 0.5, 0.5, 0 }, // 2
 //       { 0, 0.5, 0 }, // 3
 //       { 0, 0, 0.5 }, // 4
 //       { 0.5, 0, 0.5 }, // 5
 //       { 0.5, 0.5, 0.5 }, // 6
 //       { 0, 0.5, 0.5 }  // 7
 //   ));
 //   rigids.push_back(gen.sphere({ 0,0,0 }, 0.5, 10));


 //   rigids[0].initialization();
 //   rigids[1].initialization();


 //   SimulMan::Renderer render(rigids);


 //   //render.viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
 //   //    (void)v;
 //   //    void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, render.surface->vertex_buffer(), GL_WRITE_ONLY);
 //   //    easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
 //   //    if (!vertices) return false;

 //   //    return true;
 //   //};

 //   render.run();
 //   

 //   ConvexityBased con;

 //   std::vector<ConvexityBased::vec3> shape0, shape1;
 //   shape0.push_back({ 0,0,0 });
 //   shape0.push_back({ 1,0,0 });
 //   shape0.push_back({ 0,1,0 });
 //   shape0.push_back({ 0,0,1 });

 //   shape1.push_back({ 1.1,0,0 });
 //   shape1.push_back({ 2.1,0,0 });
 //   shape1.push_back({ 1.1,1,0 });
 //   shape1.push_back({ 1.1,0,1 });

 //   std::cout << con.intersection_gjk(shape0, shape1);

 //   return 0;

 //   GeometryPrimer primer;

 //   exactinit();
 //   double p1[3] = { 0, 0, 0 };
 //   double p2[3] = { 1, 1e-8, 1e-8 };
 //   double p3[3] = { 2, 2e-8, 2e-8 };
 //   double p4[3] = { 3, 2.9e-8, 3.1e-8 };
 //   std::cout << orient3d(p1, p2, p3, p4);

 //   PredicatesSimple pred;
 //   std::cout << pred.orient3d(Eigen::Vector3d(p1), Eigen::Vector3d(p2), Eigen::Vector3d(p3), Eigen::Vector3d(p4));

 //   //test1
 //   //Vector3d A(0, 0, 0), B(1, 0, 0), C(0, 1, 0);
 //   //Vector3d P(0, 0, 1), Q(1, 0, 1), R(0, 1, 1);
 //   //test1
 //   Eigen::Vector3d A(0, 0, 0), B(1, 0, 0), C(0, 1, 0);
 //   Eigen::Vector3d P(0, 0.5, 1), Q(1, 0.5, 1), R(0, 0, -1);

 //   //bool intersect = triangleIntersection(A, B, C, P, Q, R);
 //   //std::cout << "Triangles " << (intersect ? "intersect" : "do not intersect") << std::endl;


	////std::vector<RigidBody> rigids;

	////SimulMan::Renderer renderer(rigids);

	////renderer.run();


	return 0;


}
