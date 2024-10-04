
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/camera.h>
#include <easy3d/renderer/vertex_array_object.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>
#include <easy3d/core/poly_mesh.h>
#include <easy3d/algo/surface_mesh_geometry.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/renderer/drawable_lines.h>


using namespace easy3d;


#include "./src/constraints.hpp"
#include "./src/rigid_body.hpp"



void init_easy3d_udf(
    Viewer& viewer,
    TrianglesDrawable* surface,
    std::vector<RigidBody>& rigids,
    std::vector<unsigned int>& pos_sizes
) {

    // 그리드 생성
    auto grid = new easy3d::LinesDrawable("grid");

    // 그리드 데이터 설정
    std::vector<easy3d::vec3> lines;
    float size = 5.0f;
    int divisions = 10;
    float step = size / divisions;

    for (int i = 0; i <= divisions; ++i) {
        float pos = -size / 2 + i * step;
        lines.push_back(easy3d::vec3(pos, -size / 2, 0));
        lines.push_back(easy3d::vec3(pos, size / 2, 0));
        lines.push_back(easy3d::vec3(-size / 2, pos, 0));
        lines.push_back(easy3d::vec3(size / 2, pos, 0));
    }

    grid->update_vertex_buffer(lines);
    grid->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 1.0f)); // 회색

    // 뷰어에 그리드 추가
    viewer.add_drawable(grid);


    std::vector<vec3> points;
    for (auto& rigid : rigids) {
        for (auto& p : rigid.pos) {
            points.push_back(vec3(p[0], p[1], p[2]));
        }
    }
    // Each consecutive 3 indices represent a triangle.
    pos_sizes.push_back(0);
    std::vector<unsigned int> indices;
    for (auto& rigid : rigids) {
        for (auto& vs : rigid.f2v) {
            indices.push_back(static_cast<unsigned int>(pos_sizes.back() + vs[0]));
            indices.push_back(static_cast<unsigned int>(pos_sizes.back() + vs[1]));
            indices.push_back(static_cast<unsigned int>(pos_sizes.back() + vs[2]));
        }
        pos_sizes.push_back(pos_sizes.back() + rigid.pos.size());
    }
    //-------------------------------------------------------------
    // Create a TrianglesDrawable to visualize the surface of the "bunny".
    surface->update_vertex_buffer(points, true);
    surface->update_element_buffer(indices);
    viewer.add_drawable(surface);



    //viewer.fit_screen();
    viewer.camera()->setZClippingCoefficient(10.0f);
    //viewer.camera()->setPosition(vec3(0, 0, 4));
    //mat4 mvm = viewer.camera()->modelViewMatrix();
    //mvm(0, 3) = 2.0;
    //std::cout << mvm << std::endl;
    //viewer.camera()->set_modelview_matrix(mvm);
    viewer.set_animation(true);

}


//========================
// 볼 조인트 : 진자운동
int test_ball_joint() {

    RigidBodyGenerator gen;

    std::vector<RigidBody> rigids;
    rigids.push_back(gen.hexahedron(
        { 0, 0, 0 }, // 0
        { 0.5, 0, 0 }, // 1
        { 0.5, 0.5, 0 }, // 2
        { 0, 0.5, 0 }, // 3
        { 0, 0, 0.5 }, // 4
        { 0.5, 0, 0.5 }, // 5
        { 0.5, 0.5, 0.5 }, // 6
        { 0, 0.5, 0.5 }  // 7
    ));
    rigids.push_back(rigids[0]);

    for (auto& rigid : rigids) {
        rigid.initialization();
    }

    rigids[1].scale(0.4);

    rigids[0].xc = Eigen::Vector3d(0.0, 2.0, 2.5);
    rigids[1].xc = Eigen::Vector3d(0.0, 0.0, 2.5);

    std::vector<std::pair<int, int>> rigid_pairs;



    Eigen::Vector3d pointPos(0.0, 0.0, 2.5);
    //PBD::ParticleBallJoint constraint(
    //    rigids[0], pointPos
    //);
    PBD::BallJoint constraint(
        rigids[0], rigids[1], pointPos
    );

    //-------------------------------------------------------------
    initialize();
    Viewer viewer;
    TrianglesDrawable* surface = new TrianglesDrawable("faces");
    std::vector<unsigned int> pos_sizes;

    init_easy3d_udf(viewer, surface, rigids, pos_sizes);


    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        (void)v;
        void* pointer = VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
        vec3* vertices = reinterpret_cast<vec3*>(pointer);
        if (!vertices) return false;

        for (std::size_t irigid = 0; irigid < rigids.size(); ++irigid) {
            auto& rigid = rigids[irigid];
            for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
                Eigen::Vector3d newx = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
                vertices[pos_sizes[irigid] + i] = vec3(newx[0], newx[1], newx[2]);
            }
        }

        double dt = 0.02;
        int nSub = 15;
        double h = dt / static_cast<double>(nSub);
        for (int iSub = 0; iSub < nSub; ++iSub) {

            auto& rigid0 = rigids[0];
            auto& rigid1 = rigids[1];

            Eigen::Vector3d force(0.0, 0.0, -9.81 * rigid0.mass);
            Eigen::Vector3d torque(0.0, 0.0, 0.0);

            Eigen::Vector3d xc_old = rigid0.xc;
            Eigen::Quaterniond orient_old = rigid0.orient;
            rigid0.update_Crank_Nicolson(h, force, torque);

            //double lambda = 0.0;
            Eigen::Vector3d lambda;
            lambda.setZero();
            for (int iciter = 0; iciter < 5; ++iciter) {
                Eigen::Vector3d dx0, dx1;
                Eigen::Quaterniond dq0, dq1;


                //constraint.update_and_solve(
                //    rigid0, 0.1, h, lambda, dx0, dq0, dx1, dq1
                //);
                constraint.update_and_solve(
                    rigid0, rigid1, 0.1, h, lambda, dx0, dq0, dx1, dq1
                );

                rigid0.xc += dx0;
                rigid0.orient.coeffs() += dq0.coeffs();
                rigid0.orient.normalize();

            }

            // 업데이트
            rigid0.vc = (rigid0.xc - xc_old) / h;
            Eigen::Quaterniond dorient = rigid0.orient * orient_old.conjugate();
            rigid0.omega = 2.0 / h * dorient.vec();
            rigid0.omega = dorient.w() >= 0.0 ? rigid0.omega : -rigid0.omega;

        }

        VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
        viewer.update();
        return true;
        };
    return viewer.run();
}




//========================
// 유니버셜
int test_universal_joint() {

    RigidBodyGenerator gen;

    std::vector<RigidBody> rigids;
    rigids.push_back(gen.hexahedron(
        { 0, 0, 0 }, // 0
        { 0.5, 0, 0 }, // 1
        { 0.5, 0.5, 0 }, // 2
        { 0, 0.5, 0 }, // 3
        { 0, 0, 0.1 }, // 4
        { 0.5, 0, 0.1 }, // 5
        { 0.5, 0.5, 0.1 }, // 6
        { 0, 0.5, 0.1 }  // 7
    ));
    rigids.push_back(rigids[0]);

    for (auto& rigid : rigids) {
        rigid.initialization();
    }


    rigids[0].xc = Eigen::Vector3d(0.0, 0.0, 1.5);
    rigids[1].xc = Eigen::Vector3d(0.0, 0.5, 1.5);

    rigids[1].orient = Eigen::Quaterniond(Eigen::AngleAxisd(3.14 / 2.0, Eigen::Vector3d(0.0, 1.0, 0.0)));


    std::vector<std::pair<int, int>> rigid_pairs;



    Eigen::Vector3d cPoint(0.0, 0.25, 1.5);
    Eigen::Vector3d cAxis0(1.0, 0.0, 0.0);
    Eigen::Vector3d cAxis1(0.0, 0.0, 1.0);
    PBD::UniversalJoint constraint(
        rigids[0], rigids[1], cPoint, cAxis0, cAxis1
    );

    //-------------------------------------------------------------
    initialize();
    Viewer viewer;
    TrianglesDrawable* surface = new TrianglesDrawable("faces");
    std::vector<unsigned int> pos_sizes;

    init_easy3d_udf(viewer, surface, rigids, pos_sizes);


    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        (void)v;
        void* pointer = VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
        vec3* vertices = reinterpret_cast<vec3*>(pointer);
        if (!vertices) return false;

        for (std::size_t irigid = 0; irigid < rigids.size(); ++irigid) {
            auto& rigid = rigids[irigid];
            for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
                Eigen::Vector3d newx = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
                vertices[pos_sizes[irigid] + i] = vec3(newx[0], newx[1], newx[2]);
            }
        }

        double dt = 0.02;
        int nSub = 15;
        double h = dt / static_cast<double>(nSub);
        for (int iSub = 0; iSub < nSub; ++iSub) {

            auto& rigid0 = rigids[0];
            auto& rigid1 = rigids[1];

            Eigen::Vector3d force0(0.0, 0.0, 0.0);
            Eigen::Vector3d torque0(0.001, 0.0, 0.0);
            Eigen::Vector3d force1(0.0, 0.0, 0.0);
            Eigen::Vector3d torque1(0.0, 0.0, 0.0);

            Eigen::Vector3d xc0_old = rigid0.xc;
            Eigen::Vector3d xc1_old = rigid1.xc;
            Eigen::Quaterniond orient0_old = rigid0.orient;
            Eigen::Quaterniond orient1_old = rigid1.orient;

            rigid0.update_Crank_Nicolson(h, force0, torque0);
            rigid1.update_Crank_Nicolson(h, force1, torque1);

            //double lambda = 0.0;
            Eigen::Vector3d lambda;
            lambda.setZero();
            for (int iciter = 0; iciter < 5; ++iciter) {
                Eigen::Vector3d dx0, dx1;
                Eigen::Quaterniond dq0, dq1;

                constraint.update_and_solve(
                    rigid0, rigid1, 0.1, h, lambda, dx0, dq0, dx1, dq1
                );

                rigid0.xc += dx0;
                rigid0.orient.coeffs() += dq0.coeffs();
                rigid0.orient.normalize();

                rigid1.xc += dx1;
                rigid1.orient.coeffs() += dq1.coeffs();
                rigid1.orient.normalize();

            }

            // 업데이트
            {
                rigid0.vc = (rigid0.xc - xc0_old) / h;
                Eigen::Quaterniond dorient = rigid0.orient * orient0_old.conjugate();
                rigid0.omega = 2.0 / h * dorient.vec();
                rigid0.omega = dorient.w() >= 0.0 ? rigid0.omega : -rigid0.omega;
            }
            {
                rigid1.vc = (rigid1.xc - xc1_old) / h;
                Eigen::Quaterniond dorient = rigid1.orient * orient1_old.conjugate();
                rigid1.omega = 2.0 / h * dorient.vec();
                rigid1.omega = dorient.w() >= 0.0 ? rigid1.omega : -rigid1.omega;
            }

        }

        VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
        viewer.update();
        return true;
        };
    return viewer.run();
}



//========================
// 힌지
int test_hinge_joint() {

    RigidBodyGenerator gen;

    std::vector<RigidBody> rigids;
    rigids.push_back(gen.hexahedron(
        { 0, 0, 0 }, // 0
        { 0.5, 0, 0 }, // 1
        { 0.5, 0.5, 0 }, // 2
        { 0, 0.5, 0 }, // 3
        { 0, 0, 0.1 }, // 4
        { 0.5, 0, 0.1 }, // 5
        { 0.5, 0.5, 0.1 }, // 6
        { 0, 0.5, 0.1 }  // 7
    ));
    rigids.push_back(rigids[0]);

    for (auto& rigid : rigids) {
        rigid.initialization();
    }

    rigids[0].xc = Eigen::Vector3d(0.0, 0.0, 1.5);
    rigids[1].xc = Eigen::Vector3d(0.0, 0.5, 1.5);

    Eigen::Vector3d cPoint(0.0, 0.25, 1.5);
    Eigen::Vector3d cAxis(1.0, 0.0, 0.0);
    PBD::HingeJoint constraint(
        rigids[0], rigids[1], cPoint, cAxis
    );

    //-------------------------------------------------------------
    initialize();
    Viewer viewer;
    TrianglesDrawable* surface = new TrianglesDrawable("faces");
    std::vector<unsigned int> pos_sizes;

    init_easy3d_udf(viewer, surface, rigids, pos_sizes);


    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        (void)v;
        void* pointer = VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
        vec3* vertices = reinterpret_cast<vec3*>(pointer);
        if (!vertices) return false;

        for (std::size_t irigid = 0; irigid < rigids.size(); ++irigid) {
            auto& rigid = rigids[irigid];
            for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
                Eigen::Vector3d newx = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
                vertices[pos_sizes[irigid] + i] = vec3(newx[0], newx[1], newx[2]);
            }
        }

        double dt = 0.02;
        int nSub = 15;
        double h = dt / static_cast<double>(nSub);
        for (int iSub = 0; iSub < nSub; ++iSub) {

            auto& rigid0 = rigids[0];
            auto& rigid1 = rigids[1];

            Eigen::Vector3d force0(0.0, 0.0, -1.0);
            Eigen::Vector3d torque0(0.001, 0.0, 0.0);
            //Eigen::Vector3d force1(0.0, 0.0, 0.0);
            //Eigen::Vector3d torque1(0.0, 0.0, 0.0);

            Eigen::Vector3d xc0_old = rigid0.xc;
            Eigen::Vector3d xc1_old = rigid1.xc;
            Eigen::Quaterniond orient0_old = rigid0.orient;
            Eigen::Quaterniond orient1_old = rigid1.orient;

            rigid0.update_Crank_Nicolson(h, force0, torque0);
            //rigid1.update_Crank_Nicolson(h, force1, torque1);

            //double lambda = 0.0;
            Eigen::Vector3d lambda;
            lambda.setZero();
            for (int iciter = 0; iciter < 5; ++iciter) {
                Eigen::Vector3d dx0, dx1;
                Eigen::Quaterniond dq0, dq1;

                constraint.update_and_solve(
                    rigid0, rigid1, 0.1, h, lambda, dx0, dq0, dx1, dq1
                );

                rigid0.xc += dx0;
                rigid0.orient.coeffs() += dq0.coeffs();
                rigid0.orient.normalize();

                //rigid1.xc += dx1;
                //rigid1.orient.coeffs() += dq1.coeffs();
                //rigid1.orient.normalize();

            }

            // 업데이트
            {
                rigid0.vc = (rigid0.xc - xc0_old) / h;
                Eigen::Quaterniond dorient = rigid0.orient * orient0_old.conjugate();
                rigid0.omega = 2.0 / h * dorient.vec();
                rigid0.omega = dorient.w() >= 0.0 ? rigid0.omega : -rigid0.omega;
            }
            //{
            //    rigid1.vc = (rigid1.xc - xc1_old) / h;
            //    Eigen::Quaterniond dorient = rigid1.orient * orient1_old.conjugate();
            //    rigid1.omega = 2.0 / h * dorient.vec();
            //    rigid1.omega = dorient.w() >= 0.0 ? rigid1.omega : -rigid1.omega;
            //}

        }

        VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
        viewer.update();
        return true;
        };
    return viewer.run();
}


//========================
// 실린더
int test_cylinder_joint() {

    RigidBodyGenerator gen;

    std::vector<RigidBody> rigids;
    rigids.push_back(gen.hexahedron(
        { 0, 0, 0 }, // 0
        { 0.5, 0, 0 }, // 1
        { 0.5, 0.5, 0 }, // 2
        { 0, 0.5, 0 }, // 3
        { 0, 0, 0.1 }, // 4
        { 0.5, 0, 0.1 }, // 5
        { 0.5, 0.5, 0.1 }, // 6
        { 0, 0.5, 0.1 }  // 7
    ));
    rigids.push_back(rigids[0]);

    for (auto& rigid : rigids) {
        rigid.initialization();
    }

    rigids[0].xc = Eigen::Vector3d(0.0, 0.0, 1.5);
    rigids[1].xc = Eigen::Vector3d(0.0, 0.5, 1.5);

    Eigen::Vector3d cPoint(0.0, 0.25, 1.5);
    Eigen::Vector3d cAxis(1.0, 0.0, 0.0);
    PBD::CylindricalJoint constraint(
        rigids[0], rigids[1], cPoint, cAxis
    );

    //-------------------------------------------------------------
    initialize();
    Viewer viewer;
    TrianglesDrawable* surface = new TrianglesDrawable("faces");
    std::vector<unsigned int> pos_sizes;

    init_easy3d_udf(viewer, surface, rigids, pos_sizes);


    double time = 0.0;
    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        (void)v;
        void* pointer = VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
        vec3* vertices = reinterpret_cast<vec3*>(pointer);
        if (!vertices) return false;

        for (std::size_t irigid = 0; irigid < rigids.size(); ++irigid) {
            auto& rigid = rigids[irigid];
            for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
                Eigen::Vector3d newx = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
                vertices[pos_sizes[irigid] + i] = vec3(newx[0], newx[1], newx[2]);
            }
        }

        double dt = 0.02;
        int nSub = 15;
        double h = dt / static_cast<double>(nSub);
        for (int iSub = 0; iSub < nSub; ++iSub) {
            time += h;

            auto& rigid0 = rigids[0];
            auto& rigid1 = rigids[1];

            Eigen::Vector3d force0(0.01 * std::cos(time), 0.0, -1.0);
            Eigen::Vector3d torque0(0.0, 0.0, 0.0);
            //Eigen::Vector3d force1(0.0, 0.0, 0.0);
            //Eigen::Vector3d torque1(0.0, 0.0, 0.0);

            Eigen::Vector3d xc0_old = rigid0.xc;
            Eigen::Vector3d xc1_old = rigid1.xc;
            Eigen::Quaterniond orient0_old = rigid0.orient;
            Eigen::Quaterniond orient1_old = rigid1.orient;

            rigid0.update_Crank_Nicolson(h, force0, torque0);
            //rigid1.update_Crank_Nicolson(h, force1, torque1);

            //double lambda = 0.0;
            Eigen::Vector3d lambda;
            lambda.setZero();
            for (int iciter = 0; iciter < 5; ++iciter) {
                Eigen::Vector3d dx0, dx1;
                Eigen::Quaterniond dq0, dq1;

                constraint.update_and_solve(
                    rigid0, rigid1, 0.1, h, lambda, dx0, dq0, dx1, dq1
                );

                rigid0.xc += dx0;
                rigid0.orient.coeffs() += dq0.coeffs();
                rigid0.orient.normalize();

                //rigid1.xc += dx1;
                //rigid1.orient.coeffs() += dq1.coeffs();
                //rigid1.orient.normalize();

            }

            // 업데이트
            {
                rigid0.vc = (rigid0.xc - xc0_old) / h;
                Eigen::Quaterniond dorient = rigid0.orient * orient0_old.conjugate();
                rigid0.omega = 2.0 / h * dorient.vec();
                rigid0.omega = dorient.w() >= 0.0 ? rigid0.omega : -rigid0.omega;
            }
            //{
            //    rigid1.vc = (rigid1.xc - xc1_old) / h;
            //    Eigen::Quaterniond dorient = rigid1.orient * orient1_old.conjugate();
            //    rigid1.omega = 2.0 / h * dorient.vec();
            //    rigid1.omega = dorient.w() >= 0.0 ? rigid1.omega : -rigid1.omega;
            //}

        }

        VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
        viewer.update();
        return true;
        };
    return viewer.run();
}


//========================
// 프리즈매틱
int test_prismatic_joint() {

    RigidBodyGenerator gen;

    std::vector<RigidBody> rigids;
    rigids.push_back(gen.hexahedron(
        { 0, 0, 0 }, // 0
        { 0.5, 0, 0 }, // 1
        { 0.5, 0.5, 0 }, // 2
        { 0, 0.5, 0 }, // 3
        { 0, 0, 0.1 }, // 4
        { 0.5, 0, 0.1 }, // 5
        { 0.5, 0.5, 0.1 }, // 6
        { 0, 0.5, 0.1 }  // 7
    ));
    rigids.push_back(rigids[0]);

    for (auto& rigid : rigids) {
        rigid.initialization();
    }

    rigids[0].xc = Eigen::Vector3d(0.0, 0.0, 1.5);
    rigids[1].xc = Eigen::Vector3d(0.0, 0.5, 1.5);

    Eigen::Vector3d cPoint(0.0, 0.25, 1.5);
    Eigen::Vector3d cAxis(1.0, 0.0, 0.0);
    PBD::PrismaticJoint constraint(
        rigids[0], rigids[1], cPoint, cAxis
    );

    //-------------------------------------------------------------
    initialize();
    Viewer viewer;
    TrianglesDrawable* surface = new TrianglesDrawable("faces");
    std::vector<unsigned int> pos_sizes;

    init_easy3d_udf(viewer, surface, rigids, pos_sizes);


    double time = 0.0;
    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        (void)v;
        void* pointer = VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
        vec3* vertices = reinterpret_cast<vec3*>(pointer);
        if (!vertices) return false;

        for (std::size_t irigid = 0; irigid < rigids.size(); ++irigid) {
            auto& rigid = rigids[irigid];
            for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
                Eigen::Vector3d newx = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
                vertices[pos_sizes[irigid] + i] = vec3(newx[0], newx[1], newx[2]);
            }
        }

        double dt = 0.02;
        int nSub = 15;
        double h = dt / static_cast<double>(nSub);
        for (int iSub = 0; iSub < nSub; ++iSub) {
            time += h;

            auto& rigid0 = rigids[0];
            auto& rigid1 = rigids[1];

            Eigen::Vector3d force0(0.01 * std::cos(time), 0.0, -1.0);
            Eigen::Vector3d torque0(0.0, 0.0, 0.0);
            //Eigen::Vector3d force1(0.0, 0.0, 0.0);
            //Eigen::Vector3d torque1(0.0, 0.0, 0.0);

            Eigen::Vector3d xc0_old = rigid0.xc;
            Eigen::Vector3d xc1_old = rigid1.xc;
            Eigen::Quaterniond orient0_old = rigid0.orient;
            Eigen::Quaterniond orient1_old = rigid1.orient;

            rigid0.update_Crank_Nicolson(h, force0, torque0);
            //rigid1.update_Crank_Nicolson(h, force1, torque1);

            //double lambda = 0.0;
            Eigen::Vector3d lambda;
            lambda.setZero();
            for (int iciter = 0; iciter < 5; ++iciter) {
                Eigen::Vector3d dx0, dx1;
                Eigen::Quaterniond dq0, dq1;

                constraint.update_and_solve(
                    rigid0, rigid1, 0.1, h, lambda, dx0, dq0, dx1, dq1
                );

                rigid0.xc += dx0;
                rigid0.orient.coeffs() += dq0.coeffs();
                rigid0.orient.normalize();

                //rigid1.xc += dx1;
                //rigid1.orient.coeffs() += dq1.coeffs();
                //rigid1.orient.normalize();

            }

            // 업데이트
            {
                rigid0.vc = (rigid0.xc - xc0_old) / h;
                Eigen::Quaterniond dorient = rigid0.orient * orient0_old.conjugate();
                rigid0.omega = 2.0 / h * dorient.vec();
                rigid0.omega = dorient.w() >= 0.0 ? rigid0.omega : -rigid0.omega;
            }
            //{
            //    rigid1.vc = (rigid1.xc - xc1_old) / h;
            //    Eigen::Quaterniond dorient = rigid1.orient * orient1_old.conjugate();
            //    rigid1.omega = 2.0 / h * dorient.vec();
            //    rigid1.omega = dorient.w() >= 0.0 ? rigid1.omega : -rigid1.omega;
            //}

        }

        VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
        viewer.update();
        return true;
        };
    return viewer.run();
}


//
////========================
//// 평판 부등식
//int test_plane_inequality() {
//
//    RigidBodyGenerator gen;
//
//    std::vector<RigidBody> rigids;
//    rigids.push_back(gen.hexahedron(
//        { 0, 0, 0 }, // 0
//        { 5.0, 0, 0 }, // 1
//        { 5.0, 5.0, 0 }, // 2
//        { 0, 5.0, 0 }, // 3
//        { 0, 0, 0.1 }, // 4
//        { 5.0, 0, 0.1 }, // 5
//        { 5.0, 5.0, 0.1 }, // 6
//        { 0, 5.0, 0.1 }  // 7
//    ));
//    rigids.push_back(gen.hexahedron(
//        { 0, 0, 0 }, // 0
//        { 0.5, 0, 0 }, // 1
//        { 0.5, 0.5, 0 }, // 2
//        { 0, 0.5, 0 }, // 3
//        { 0, 0, 0.5 }, // 4
//        { 0.5, 0, 0.5 }, // 5
//        { 0.5, 0.5, 0.5 }, // 6
//        { 0, 0.5, 0.5 }  // 7
//    ));
//
//    for (auto& rigid : rigids) {
//        rigid.initialization();
//    }
//
//    rigids[0].set_density(1.0);
//    rigids[1].set_density(1.0);
//
//    rigids[0].xc = Eigen::Vector3d(0.0, 0.0, 0.0);
//    rigids[1].xc = Eigen::Vector3d(0.0, 0.5, 1.5);
//
//    Eigen::Vector3d cPoint(0.0, 0.0, 0.0);
//    Eigen::Vector3d cNormal(0.0, 0.0, 1.0);
//    PBD::PlaneInequality constraint(
//        rigids[0], rigids[1], cPoint, cNormal
//    );
//
//    //-------------------------------------------------------------
//    initialize();
//    Viewer viewer;
//    TrianglesDrawable* surface = new TrianglesDrawable("faces");
//    std::vector<unsigned int> pos_sizes;
//
//    init_easy3d_udf(viewer, surface, rigids, pos_sizes);
//
//
//    double time = 0.0;
//    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
//        (void)v;
//        void* pointer = VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
//        vec3* vertices = reinterpret_cast<vec3*>(pointer);
//        if (!vertices) return false;
//
//        for (std::size_t irigid = 0; irigid < rigids.size(); ++irigid) {
//            auto& rigid = rigids[irigid];
//            for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
//                Eigen::Vector3d newx = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
//                vertices[pos_sizes[irigid] + i] = vec3(newx[0], newx[1], newx[2]);
//            }
//        }
//
//        double dt = 0.02;
//        int nSub = 15;
//        double h = dt / static_cast<double>(nSub);
//        for (int iSub = 0; iSub < nSub; ++iSub) {
//            time += h;
//
//            auto& rigid0 = rigids[0];
//            auto& rigid1 = rigids[1];
//
//            //Eigen::Vector3d force0(0.01 * std::cos(time), 0.0, -1.0);
//            //Eigen::Vector3d torque0(0.0, 0.0, 0.0);
//            Eigen::Vector3d force1(0.0, 0.0, -9.8 * rigid1.mass);
//            Eigen::Vector3d torque1(0.0, 0.0, 0.0);
//
//            Eigen::Vector3d xc0_old = rigid0.xc;
//            Eigen::Vector3d xc1_old = rigid1.xc;
//            Eigen::Quaterniond orient0_old = rigid0.orient;
//            Eigen::Quaterniond orient1_old = rigid1.orient;
//
//            //rigid0.update_Crank_Nicolson(h, force0, torque0);
//            rigid1.update_Crank_Nicolson(h, force1, torque1);
//
//            //double lambda = 0.0;
//            Eigen::Vector3d lambda;
//            lambda.setZero();
//            for (int iciter = 0; iciter < 5; ++iciter) {
//                Eigen::Vector3d dx0, dx1;
//                Eigen::Quaterniond dq0, dq1;
//
//                constraint.update_and_solve(
//                    rigid0, rigid1, 0.1, h, lambda, dx0, dq0, dx1, dq1
//                );
//
//                //rigid0.xc += dx0;
//                //rigid0.orient.coeffs() += dq0.coeffs();
//                //rigid0.orient.normalize();
//
//                rigid1.xc += dx1;
//                rigid1.orient.coeffs() += dq1.coeffs();
//                rigid1.orient.normalize();
//
//            }
//
//            // 업데이트
//            {
//                rigid0.vc = (rigid0.xc - xc0_old) / h;
//                Eigen::Quaterniond dorient = rigid0.orient * orient0_old.conjugate();
//                rigid0.omega = 2.0 / h * dorient.vec();
//                rigid0.omega = dorient.w() >= 0.0 ? rigid0.omega : -rigid0.omega;
//            }
//            {
//                rigid1.vc = (rigid1.xc - xc1_old) / h;
//                Eigen::Quaterniond dorient = rigid1.orient * orient1_old.conjugate();
//                rigid1.omega = 2.0 / h * dorient.vec();
//                rigid1.omega = dorient.w() >= 0.0 ? rigid1.omega : -rigid1.omega;
//            }
//
//        }
//
//        VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
//        viewer.update();
//        return true;
//        };
//    return viewer.run();
//}




//========================
// 힌지 부등식
int test_hinge_inequality() {

    RigidBodyGenerator gen;

    std::vector<RigidBody> rigids;
    rigids.push_back(gen.hexahedron(
        { 0, 0, 0 }, // 0
        { 0.5, 0, 0 }, // 1
        { 0.5, 0.5, 0 }, // 2
        { 0, 0.5, 0 }, // 3
        { 0, 0, 0.1 }, // 4
        { 0.5, 0, 0.1 }, // 5
        { 0.5, 0.5, 0.1 }, // 6
        { 0, 0.5, 0.1 }  // 7
    ));
    rigids.push_back(rigids[0]);

    for (auto& rigid : rigids) {
        rigid.initialization();
    }

    rigids[0].xc = Eigen::Vector3d(0.0, 0.0, 1.5);
    rigids[1].xc = Eigen::Vector3d(0.0, 0.5, 1.5);

    Eigen::Vector3d cPoint(0.0, 0.25, 1.5);
    Eigen::Vector3d cAxis(1.0, 0.0, 0.0);
    PBD::HingeInequality constraint(
        rigids[0], rigids[1], cPoint, cAxis, 3.141592 / 3.0
    );

    //-------------------------------------------------------------
    initialize();
    Viewer viewer;
    TrianglesDrawable* surface = new TrianglesDrawable("faces");
    std::vector<unsigned int> pos_sizes;

    init_easy3d_udf(viewer, surface, rigids, pos_sizes);


    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        (void)v;
        void* pointer = VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
        vec3* vertices = reinterpret_cast<vec3*>(pointer);
        if (!vertices) return false;

        for (std::size_t irigid = 0; irigid < rigids.size(); ++irigid) {
            auto& rigid = rigids[irigid];
            for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
                Eigen::Vector3d newx = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
                vertices[pos_sizes[irigid] + i] = vec3(newx[0], newx[1], newx[2]);
            }
        }

        double dt = 0.002;
        int nSub = 5;
        double h = dt / static_cast<double>(nSub);
        for (int iSub = 0; iSub < nSub; ++iSub) {

            auto& rigid0 = rigids[0];
            auto& rigid1 = rigids[1];

            Eigen::Vector3d force0(0.0, 0.0, -1.0);
            Eigen::Vector3d torque0(0.001, 0.0, 0.0);
            //Eigen::Vector3d force1(0.0, 0.0, 0.0);
            //Eigen::Vector3d torque1(0.0, 0.0, 0.0);

            Eigen::Vector3d xc0_old = rigid0.xc;
            Eigen::Vector3d xc1_old = rigid1.xc;
            Eigen::Quaterniond orient0_old = rigid0.orient;
            Eigen::Quaterniond orient1_old = rigid1.orient;

            rigid0.update_Crank_Nicolson(h, force0, torque0);
            //rigid1.update_Crank_Nicolson(h, force1, torque1);

            //double lambda = 0.0;
            Eigen::Vector3d lambda;
            lambda.setZero();
            for (int iciter = 0; iciter < 5; ++iciter) {
                Eigen::Vector3d dx0, dx1;
                Eigen::Quaterniond dq0, dq1;

                constraint.update_and_solve(
                    rigid0, rigid1, 0.1, h, lambda, dx0, dq0, dx1, dq1
                );

                rigid0.xc += dx0;
                rigid0.orient.coeffs() += dq0.coeffs();
                rigid0.orient.normalize();

                //rigid1.xc += dx1;
                //rigid1.orient.coeffs() += dq1.coeffs();
                //rigid1.orient.normalize();

            }

            // 업데이트
            {
                rigid0.vc = (rigid0.xc - xc0_old) / h;
                Eigen::Quaterniond dorient = rigid0.orient * orient0_old.conjugate();
                rigid0.omega = 2.0 / h * dorient.vec();
                rigid0.omega = dorient.w() >= 0.0 ? rigid0.omega : -rigid0.omega;
            }
            //{
            //    rigid1.vc = (rigid1.xc - xc1_old) / h;
            //    Eigen::Quaterniond dorient = rigid1.orient * orient1_old.conjugate();
            //    rigid1.omega = 2.0 / h * dorient.vec();
            //    rigid1.omega = dorient.w() >= 0.0 ? rigid1.omega : -rigid1.omega;
            //}

        }

        VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
        viewer.update();
        return true;
        };
    return viewer.run();
}





//========================
// 유니버설 조인트 -> 체인
int test_chain() {

    RigidBodyGenerator gen;

    std::vector<RigidBody> rigids;
    rigids.push_back(gen.hexahedron(
        { 0, 0, 0 }, // 0
        { 0.2, 0, 0 }, // 1
        { 0.2, 0.5, 0 }, // 2
        { 0, 0.5, 0 }, // 3
        { 0, 0, 0.1 }, // 4
        { 0.2, 0, 0.1 }, // 5
        { 0.2, 0.5, 0.1 }, // 6
        { 0, 0.5, 0.1 }  // 7
    ));
    rigids.push_back(rigids[0]);
    rigids.push_back(rigids[0]);
    rigids.push_back(rigids[0]);

    for (auto& rigid : rigids) {
        rigid.initialization();
    }

    rigids[0].xc = Eigen::Vector3d(0.0, 0.25, 1.5);
    rigids[1].xc = Eigen::Vector3d(0.0, 0.75, 1.5);
    rigids[2].xc = Eigen::Vector3d(0.0, 1.25, 1.5);
    rigids[3].xc = Eigen::Vector3d(0.0, 1.75, 1.5);

    rigids[1].orient = Eigen::Quaterniond(Eigen::AngleAxisd(3.14 / 2.0, Eigen::Vector3d(0.0, 1.0, 0.0)));
    rigids[3].orient = Eigen::Quaterniond(Eigen::AngleAxisd(3.14 / 2.0, Eigen::Vector3d(0.0, 1.0, 0.0)));


    Eigen::Vector3d cAxis0(1.0, 0.0, 0.0);
    Eigen::Vector3d cAxis1(0.0, 0.0, 1.0);
    std::vector<std::unique_ptr<PBD::Constraint>> constraints;
    constraints.push_back(
        std::make_unique<PBD::ParticleBallJoint>(rigids[0], Eigen::Vector3d(0.0, 0.0, 1.5))
    );
    constraints.push_back(
        std::make_unique<PBD::BallJoint>(rigids[0], rigids[1], Eigen::Vector3d(0.0, 0.0, 1.5))
    );
    constraints.push_back(
        std::make_unique<PBD::UniversalJoint>(rigids[0], rigids[1], Eigen::Vector3d(0.0, 0.5, 1.5),
            cAxis0, cAxis1)
    );
    constraints.push_back(
        std::make_unique<PBD::UniversalJoint>(rigids[1], rigids[2], Eigen::Vector3d(0.0, 1.0, 1.5),
            cAxis1, cAxis0)
    );
    constraints.push_back(
        std::make_unique<PBD::UniversalJoint>(rigids[2], rigids[3], Eigen::Vector3d(0.0, 1.5, 1.5),
            cAxis0, cAxis1)
    );
    std::vector<std::pair<int, int>> rigid_pairs;
    rigid_pairs.push_back(std::make_pair(0, 0));
    rigid_pairs.push_back(std::make_pair(0, 1));
    rigid_pairs.push_back(std::make_pair(0, 1));
    rigid_pairs.push_back(std::make_pair(1, 2));
    rigid_pairs.push_back(std::make_pair(2, 3));


    //-------------------------------------------------------------
    initialize();
    Viewer viewer;
    TrianglesDrawable* surface = new TrianglesDrawable("faces");
    std::vector<unsigned int> pos_sizes;

    init_easy3d_udf(viewer, surface, rigids, pos_sizes);


    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        (void)v;
        void* pointer = VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
        vec3* vertices = reinterpret_cast<vec3*>(pointer);
        if (!vertices) return false;

        for (std::size_t irigid = 0; irigid < rigids.size(); ++irigid) {
            auto& rigid = rigids[irigid];
            for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
                Eigen::Vector3d newx = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
                vertices[pos_sizes[irigid] + i] = vec3(newx[0], newx[1], newx[2]);
            }
        }

        double dt = 0.02;
        int nSub = 5;
        double h = dt / static_cast<double>(nSub);
        for (int iSub = 0; iSub < nSub; ++iSub) {

            std::vector<Eigen::Vector3d> xc_olds;
            std::vector<Eigen::Quaterniond> orient_olds;
            xc_olds.reserve(rigids.size());
            orient_olds.reserve(rigids.size());
            for (auto& rigid : rigids) {
                xc_olds.push_back(rigid.xc);
                orient_olds.push_back(rigid.orient);

                rigid.update_Crank_Nicolson(h, Eigen::Vector3d(0.0, 0.0, -9.8 * rigid.mass),
                    Eigen::Vector3d(0.0, 0.0, 0.0));
            }

            //double lambda = 0.0;
            Eigen::Vector3d lambda;
            lambda.setZero();
            for (int iciter = 0; iciter < 5; ++iciter) {

                for (int iconst = 0; iconst < constraints.size(); ++iconst) {
                    Eigen::Vector3d dx0, dx1;
                    Eigen::Quaterniond dq0, dq1;

                    auto& [irigid0, irigid1] = rigid_pairs[iconst];
                    auto& rigid0 = rigids[irigid0];
                    auto& rigid1 = rigids[irigid1];
                    constraints[iconst]->update_and_solve(
                        rigid0,
                        rigid1,
                        0.1, h, lambda,
                        dx0, dq0, dx1, dq1
                    );

                    rigid0.xc += dx0;
                    rigid0.orient.coeffs() += dq0.coeffs();
                    rigid0.orient.normalize();

                    rigid1.xc += dx1;
                    rigid1.orient.coeffs() += dq1.coeffs();
                    rigid1.orient.normalize();
                }

            }

            // 업데이트
            for (int irigid = 0; irigid < rigids.size(); ++irigid) {
                auto& rigid = rigids[irigid];
                auto& xc_old = xc_olds[irigid];
                auto& orient_old = orient_olds[irigid];
                rigid.vc = (rigid.xc - xc_old) / h;
                Eigen::Quaterniond dorient = rigid.orient * orient_old.conjugate();
                rigid.omega = 2.0 / h * dorient.vec();
                rigid.omega = dorient.w() >= 0.0 ? rigid.omega : -rigid.omega;
            }

        }

        VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
        viewer.update();
        return true;
        };
    return viewer.run();
}




//========================
// 로봇팔
int test_robot_arm() {

    RigidBodyGenerator gen;


    auto rigid_fix = gen.hexahedron(
        { 0, 0, 0 }, // 0
        { 0.2, 0, 0 }, // 1
        { 0.2, 0.2, 0 }, // 2
        { 0, 0.2, 0 }, // 3
        { 0, 0, 0.2 }, // 4
        { 0.2, 0, 0.2 }, // 5
        { 0.2, 0.2, 0.2 }, // 6
        { 0, 0.2, 0.2 }  // 7
    );

    std::vector<RigidBody> rigids;
    rigids.push_back(gen.hexahedron(
        { 0, 0, 0 }, // 0
        { 0.2, 0, 0 }, // 1
        { 0.2, 0.2, 0 }, // 2
        { 0, 0.2, 0 }, // 3
        { 0, 0, 0.6 }, // 4
        { 0.2, 0, 0.6 }, // 5
        { 0.2, 0.2, 0.6 }, // 6
        { 0, 0.2, 0.6 }  // 7
    ));
    rigids.push_back(gen.hexahedron(
        { 0, 0, 0.6 }, // 0
        { 0.2, 0, 0.6 }, // 1
        { 0.2, 0.2, 0.6 }, // 2
        { 0, 0.2, 0.6 }, // 3
        { 0, 0, 1.4 }, // 4
        { 0.2, 0, 1.4 }, // 5
        { 0.2, 0.2, 1.4 }, // 6
        { 0, 0.2, 1.4 }  // 7
    ));
    rigids.push_back(gen.hexahedron(
        { 0.2, 0, 0.8 }, // 0
        { 0.4, 0, 0.8 }, // 1
        { 0.4, 0.2, 0.8 }, // 2
        { 0.2, 0.2, 0.8 }, // 3
        { 0.2, 0, 1.4 }, // 4
        { 0.4, 0, 1.4 }, // 5
        { 0.4, 0.2, 1.4 }, // 6
        { 0.2, 0.2, 1.4 }  // 7
    ));
    rigids.push_back(gen.hexahedron(
        { 0.2, 0.2, 0.8 }, // 0
        { 0.4, 0.2, 0.8 }, // 1
        { 0.4, 0.6, 0.8 }, // 2
        { 0.2, 0.6, 0.8 }, // 3
        { 0.2, 0.2, 1.0 }, // 4
        { 0.4, 0.2, 1.0 }, // 5
        { 0.4, 0.6, 1.0 }, // 6
        { 0.2, 0.6, 1.0 }  // 7
    ));
    rigids.push_back(gen.hexahedron(
        { 0.2, 0.6, 0.7 }, // 0
        { 0.4, 0.6, 0.7 }, // 1
        { 0.4, 0.8, 0.7 }, // 2
        { 0.2, 0.8, 0.7 }, // 3
        { 0.2, 0.6, 1.1 }, // 4
        { 0.4, 0.6, 1.1 }, // 5
        { 0.4, 0.8, 1.1 }, // 6
        { 0.2, 0.8, 1.1 }  // 7
    ));

    for (auto& rigid : rigids) {
        rigid.initialization();
    }



    Eigen::Vector3d cAxis0(1.0, 0.0, 0.0);
    Eigen::Vector3d cAxis1(0.0, 0.0, 1.0);
    std::vector<std::unique_ptr<PBD::Constraint>> constraints;
    //constraints.push_back(
    //    std::make_unique<PBD::ParticleBallJoint>(rigids[0], Eigen::Vector3d(0.1, 0.1, 0.1))
    //);
    constraints.push_back(
        std::make_unique<PBD::ParticleHingeJoint>(rigids[0],
            Eigen::Vector3d(0.1, 0.1, 0.0), Eigen::Vector3d(0.0, 0.0, 1.0))
    );
    constraints.push_back(
        std::make_unique<PBD::HingeJoint>(rigids[0], rigids[1],
            Eigen::Vector3d(0.1, 0.1, 0.6), Eigen::Vector3d(1.0, 0.0, 0.0))
    );
    constraints.push_back(
        std::make_unique<PBD::HingeJoint>(rigids[1], rigids[2],
            Eigen::Vector3d(0.1, 0.1, 1.4), Eigen::Vector3d(1.0, 0.0, 0.0))
    );
    constraints.push_back(
        std::make_unique<PBD::HingeJoint>(rigids[2], rigids[3],
            Eigen::Vector3d(0.1, 0.2, 0.8), Eigen::Vector3d(1.0, 0.0, 0.0))
    );
    constraints.push_back(
        std::make_unique<PBD::HingeJoint>(rigids[3], rigids[4],
            Eigen::Vector3d(0.3, 0.6, 0.9), Eigen::Vector3d(0.0, 1.0, 0.0))
    );


    std::vector<std::pair<int, int>> rigid_pairs;
    rigid_pairs.push_back(std::make_pair(0, 0));
    rigid_pairs.push_back(std::make_pair(0, 1));
    rigid_pairs.push_back(std::make_pair(1, 2));
    rigid_pairs.push_back(std::make_pair(2, 3));
    rigid_pairs.push_back(std::make_pair(3, 4));


    //-------------------------------------------------------------
    initialize();
    Viewer viewer;
    TrianglesDrawable* surface = new TrianglesDrawable("faces");
    std::vector<unsigned int> pos_sizes;

    init_easy3d_udf(viewer, surface, rigids, pos_sizes);


    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        (void)v;
        void* pointer = VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
        vec3* vertices = reinterpret_cast<vec3*>(pointer);
        if (!vertices) return false;

        for (std::size_t irigid = 0; irigid < rigids.size(); ++irigid) {
            auto& rigid = rigids[irigid];
            for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
                Eigen::Vector3d newx = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
                vertices[pos_sizes[irigid] + i] = vec3(newx[0], newx[1], newx[2]);
            }
        }

        double dt = 0.02;
        int nSub = 5;
        double h = dt / static_cast<double>(nSub);
        for (int iSub = 0; iSub < nSub; ++iSub) {

            std::vector<Eigen::Vector3d> xc_olds;
            std::vector<Eigen::Quaterniond> orient_olds;
            xc_olds.reserve(rigids.size());
            orient_olds.reserve(rigids.size());
            for (auto& rigid : rigids) {
                xc_olds.push_back(rigid.xc);
                orient_olds.push_back(rigid.orient);

                rigid.update_Crank_Nicolson(h, Eigen::Vector3d::Random(),
                    0.3 * Eigen::Vector3d::Random());
            }

            //double lambda = 0.0;
            Eigen::Vector3d lambda;
            lambda.setZero();
            for (int iciter = 0; iciter < 5; ++iciter) {

                for (int iconst = 0; iconst < constraints.size(); ++iconst) {
                    Eigen::Vector3d dx0, dx1;
                    Eigen::Quaterniond dq0, dq1;

                    auto& [irigid0, irigid1] = rigid_pairs[iconst];
                    auto& rigid0 = rigids[irigid0];
                    auto& rigid1 = rigids[irigid1];
                    constraints[iconst]->update_and_solve(
                        rigid0,
                        rigid1,
                        0.1, h, lambda,
                        dx0, dq0, dx1, dq1
                    );

                    rigid0.xc += dx0;
                    rigid0.orient.coeffs() += dq0.coeffs();
                    rigid0.orient.normalize();

                    rigid1.xc += dx1;
                    rigid1.orient.coeffs() += dq1.coeffs();
                    rigid1.orient.normalize();
                }

            }

            // 업데이트
            for (int irigid = 0; irigid < rigids.size(); ++irigid) {
                auto& rigid = rigids[irigid];
                auto& xc_old = xc_olds[irigid];
                auto& orient_old = orient_olds[irigid];
                rigid.vc = (rigid.xc - xc_old) / h;
                Eigen::Quaterniond dorient = rigid.orient * orient_old.conjugate();
                rigid.omega = 2.0 / h * dorient.vec();
                rigid.omega = dorient.w() >= 0.0 ? rigid.omega : -rigid.omega;
            }

        }

        VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
        viewer.update();
        return true;
        };
    return viewer.run();
}




// 메인
int main() {
    test_robot_arm();
}

