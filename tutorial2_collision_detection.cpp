#include <iostream>
#include <array>
#include <vector>
#include <Eigen/Dense>

//#include "./src/renderer.hpp"
//#include "./src/rigid_body.hpp"
//
//#include "./src/bounding_volume.hpp"


#include "./src/geometry_primer.hpp"
#include "./src/predicates.h"
#include "./src/predicates_simple.hpp"

int main() {

    GeometryPrimer primer;

    exactinit();
    double p1[3] = { 0, 0, 0 };
    double p2[3] = { 1, 1e-8, 1e-8 };
    double p3[3] = { 2, 2e-8, 2e-8 };
    double p4[3] = { 3, 2.9e-8, 3.1e-8 };
    std::cout << orient3d(p1, p2, p3, p4);

    PredicatesSimple pred;
    std::cout << pred.orient3d(Eigen::Vector3d(p1), Eigen::Vector3d(p2), Eigen::Vector3d(p3), Eigen::Vector3d(p4));

    //test1
    //Vector3d A(0, 0, 0), B(1, 0, 0), C(0, 1, 0);
    //Vector3d P(0, 0, 1), Q(1, 0, 1), R(0, 1, 1);
    //test1
    Eigen::Vector3d A(0, 0, 0), B(1, 0, 0), C(0, 1, 0);
    Eigen::Vector3d P(0, 0.5, 1), Q(1, 0.5, 1), R(0, 0, -1);

    //bool intersect = triangleIntersection(A, B, C, P, Q, R);
    //std::cout << "Triangles " << (intersect ? "intersect" : "do not intersect") << std::endl;


	//std::vector<RigidBody> rigids;

	//SimulMan::Renderer renderer(rigids);

	//renderer.run();


	return 0;


}
