#pragma once

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
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/algo/tessellator.h>


#include <Eigen/Geometry>
#include "./geometry/shapes_visitor.hpp"
//#include "./geometry/geometry.hpp"


namespace SimulMan {


    class MyViewer : public easy3d::Viewer {
    public:
        SimulMan::Shape* selected_model_{};
        easy3d::PointsDrawable* pointsDrawer{};
        easy3d::LinesDrawable* linesDrawer{};
        easy3d::TrianglesDrawable* surfaceDrawer{};
        std::vector<unsigned int>* pos_sizes{};
        std::vector<unsigned int>* pos_str{};
        int selected_model_id = 0;


        bool on_insider = false;
        bool on_overlap = false;

        SimulMan::Shape shape0;
        SimulMan::Shape shape1;

        bool shape0_is_line = false;
        bool shape0_is_surface = false;

    protected:

        bool key_press_event(int key, int modifiers) override {
            if (true) {
                Eigen::Vector3d translation(0, 0, 0);
                Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
                double trans_vec = 0.02;
                double ten_rad = 3.141592 / 180.0 * 10.0;
                double angle = 0.0; // 3.141592 / 180.0 * 10.0;  // 예: 1도
                switch (key) {
                case KEY_W:  translation[0] = -trans_vec; break;
                case KEY_S:  translation[0] = trans_vec; break;
                case KEY_D:  translation[1] = trans_vec; break;
                case KEY_A:  translation[1] = -trans_vec; break;
                case KEY_Q:  translation[2] = trans_vec; break;
                case KEY_E:  translation[2] = -trans_vec; break;
                case KEY_R:  axis = Eigen::Vector3d::UnitX(); angle = ten_rad; break;
                case KEY_T:  axis = Eigen::Vector3d::UnitX(); angle = -ten_rad; break;
                case KEY_F:  axis = Eigen::Vector3d::UnitY(); angle = ten_rad; break;
                case KEY_G:  axis = Eigen::Vector3d::UnitY(); angle = -ten_rad; break;
                case KEY_V:  axis = Eigen::Vector3d::UnitZ(); angle = ten_rad; break;
                case KEY_B:  axis = Eigen::Vector3d::UnitZ(); angle = -ten_rad; break;
                default: return false;
                }
                Eigen::AngleAxisd rotation(angle, axis);


                if(shape0_is_surface == true) {
                    void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surfaceDrawer->vertex_buffer(), GL_WRITE_ONLY);
                    easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
                    if (!vertices) return false;

                    int i = selected_model_id;
                    auto str = (*pos_str)[i];
                    auto end = str + (*pos_sizes)[i];
                    for (int j = str; j < end; ++j) {
                        vertices[j][0] += translation[0];
                        vertices[j][1] += translation[1];
                        vertices[j][2] += translation[2];
                    }
                    easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surfaceDrawer->vertex_buffer());
                }

                if (shape0_is_line == true) {
                    void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, linesDrawer->vertex_buffer(), GL_WRITE_ONLY);
                    easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
                    if (!vertices) return false;

                    vertices[0][0] += translation[0]; vertices[0][1] += translation[1]; vertices[0][2] += translation[2];
                    vertices[1][0] += translation[0]; vertices[1][1] += translation[1]; vertices[1][2] += translation[2];

                    easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, linesDrawer->vertex_buffer());
                }

                
                //SimulMan::is_inside();
                //surfaceDrawer->set_highlight(easy3d::vec4(1.0f, 0.0f, 0.0f, 1.0f));
                //linesDrawer->set_highlight(easy3d::vec4(1.0f, 0.0f, 0.0f, 1.0f));
                //pointsDrawer->set_color(easy3d::vec4(1.0f, 0.0f, 0.0f, 1.0f));


                update();

                if (auto pval = std::get_if<SimulMan::AABB>(&shape0)) {
                    pval->min += translation;
                    pval->max += translation;
                }
                if (auto pval = std::get_if<SimulMan::Ray>(&shape0)) {
                    pval->p += translation;
                }
                if (auto pval = std::get_if<SimulMan::Sphere>(&shape0)) {
                    pval->c += translation;
                }
                if (auto pval = std::get_if<SimulMan::Segment>(&shape0)) {
                    pval->a += translation;
                    pval->b += translation;
                }
                if (auto pval = std::get_if<SimulMan::Plane>(&shape0)) {
                    pval->d += pval->n.dot(translation);
                }
                if (auto pval = std::get_if<SimulMan::Triangle>(&shape0)) {
                    pval->a += translation;
                    pval->b += translation;
                    pval->c += translation;
                }
                if (auto pval = std::get_if<SimulMan::ConvexPolyhedra>(&shape0)) {
                    for (auto& p : (*pval)) {
                        p += translation;
                    }
                }

                if (on_insider) {

                    if (is_inside(shape1, shape0)) {
                        surfaceDrawer->set_highlight_range(std::make_pair(0, 1000));
                        linesDrawer->set_highlight_range(std::make_pair(0, 1000));
                        pointsDrawer->set_highlight_range(std::make_pair(0, 1000));

                        surfaceDrawer->set_highlight(true);
                        linesDrawer->set_highlight(true);
                        pointsDrawer->set_highlight(true);

                    }
                    else {
                        surfaceDrawer->set_highlight(false);
                        linesDrawer->set_highlight(false);
                        pointsDrawer->set_highlight(false);
                    }
                }

                if (on_overlap) {

                    if (is_overlap(shape1, shape0)) {
                        surfaceDrawer->set_highlight_range(std::make_pair(0, 1000));
                        linesDrawer->set_highlight_range(std::make_pair(0, 1000));
                        pointsDrawer->set_highlight_range(std::make_pair(0, 1000));

                        surfaceDrawer->set_highlight(true);
                        linesDrawer->set_highlight(true);
                        pointsDrawer->set_highlight(true);

                    }
                    else {
                        surfaceDrawer->set_highlight(false);
                        linesDrawer->set_highlight(false);
                        pointsDrawer->set_highlight(false);
                    }
                }

                return true;
            }

            return false;
        }
    };


    class Renderer {
    public:
        //using Shapes = std::vector<std::shared_ptr<SimulMan::Shape>>;
        using Shapes = std::vector<SimulMan::Shape>;
        MyViewer viewer{};
        easy3d::PointsDrawable* pointsDrawer{};
        easy3d::LinesDrawable* linesDrawer{};
        easy3d::TrianglesDrawable* surfaceDrawer{};
        std::vector<unsigned int> pos_sizes{};
        std::vector<unsigned int> pos_str{};
        Shapes shapes_;

        float real_point_size = 15.0f;
        float line_point_size = 8.0f;

        enum class Tester {
            Inside,
            Overlap,
        };

        explicit Renderer(Shapes& shapes) :
            shapes_(shapes)
        {
            init();
        }

        explicit Renderer(Shapes& shapes, bool on_insider) :
            shapes_(shapes)
        {
            viewer.on_insider = on_insider;
            viewer.shape0 = shapes[0];
            viewer.shape1 = shapes[1];

            init();
        }

        explicit Renderer(const Shape& shape0, const Shape& shape1, Tester tester)
        {
            shapes_.push_back(shape0);
            shapes_.push_back(shape1);

            if (auto shape0_p = std::get_if<SimulMan::Ray>(&shape0)) {
                viewer.shape0_is_line = true;
                viewer.shape0_is_surface = false;
            }
            else if (auto shape0_p = std::get_if<SimulMan::Segment>(&shape0)) {
                viewer.shape0_is_line = true;
                viewer.shape0_is_surface = false;
            }
            else {
                viewer.shape0_is_line = false;
                viewer.shape0_is_surface = true;
            }

            if (tester == Tester::Inside) {
                viewer.on_insider = true;
            }
            if (tester == Tester::Overlap) {
                viewer.on_overlap = true;
            }
            viewer.shape0 = shape0;
            viewer.shape1 = shape1;

            init();
        }

        void init() {
            easy3d::initialize();
            this->pointsDrawer = new easy3d::PointsDrawable("points");
            this->linesDrawer = new easy3d::LinesDrawable("line");
            this->surfaceDrawer = new easy3d::TrianglesDrawable("faces");

            init_easy3d_udf();


            viewer.pos_sizes = &pos_sizes;
            viewer.pos_str = &pos_str;
            viewer.pointsDrawer = pointsDrawer;
            viewer.linesDrawer = linesDrawer;
            viewer.surfaceDrawer = surfaceDrawer;

            viewer.selected_model_id = 0;
            viewer.selected_model_ = &shapes_[viewer.selected_model_id];

            //set_animation();
        }

        int run() {
            return viewer.run();
        }

        //void set_animation() {

        //    std::size_t face_size = 0;
        //    for (std::size_t irigid = 0; irigid < shapes_.size(); ++irigid) {
        //        auto& shape = shapes_[irigid];
        //        face_size += shape->drawer.f2v.size();
        //    }
        //    surface->set_highlight_range(std::make_pair(0, face_size - 1));

        //    viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
        //        (void)v;
        //        void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
        //        easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
        //        if (!vertices) return false;


        //        for (std::size_t ishape = 0; ishape < shapes_.size(); ++ishape) {
        //            auto const& shape = shapes_[ishape];
        //            for (std::size_t i = 0; i < shape->drawer.pos.size(); ++i) {
        //                //rigid.pos[i] = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
        //                auto& p = shape->drawer.pos[i];
        //                //std::cout << i << " " << p.transpose() << std::endl;
        //                //vertices[pos_sizes[ishape] + i] = easy3d::vec3(p[0], p[1], p[2]);
        //            }
        //        }

        //        bool is_collide = false;

        //        //GeometryPrimer geo;
        //        //GeometryPrimer::AABB aabb;
        //        //aabb.max = { -1.e200,-1.e200 ,-1.e200 };
        //        //aabb.min = { 1.e200,1.e200 ,1.e200 };
        //        //for (auto& p : rigids_[0].pos) {
        //        //    for (int j = 0; j < 3; ++j) {
        //        //        aabb.max[j] = std::max(aabb.max[j], p[j]);
        //        //        aabb.min[j] = std::min(aabb.min[j], p[j]);
        //        //    }
        //        //}
        //        //is_collide = geo.is_overlap(
        //        //    GeometryPrimer::Sphere(Eigen::Vector3d(0.0, 0.0, 0.0), 0.5), aabb
        //        //);

        //        //ConvexityBased con;
        //        //is_collide = con.intersection_gjk(rigids_[0].pos, rigids_[1].pos);
        //        //if (is_collide) {
        //        //    surface->set_highlight(true);
        //        //}
        //        //else {
        //        //    surface->set_highlight(false);
        //        //}

        //        easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
        //        viewer.update();

        //        return true;
        //        };

        //}

        void init_easy3d_udf() {

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


            // 포인트 그리기
            std::vector<easy3d::vec3> real_points;
            for (auto& shape : shapes_) {
                auto [real_point, dummy0, dummy1, dummy2] = trianglation(shape);
                for (auto& p : real_point) {
                    real_points.push_back(easy3d::vec3((float)p[0], (float)p[1], (float)p[2]));
                }
            }
            if (real_points.size() != 0) {
                pointsDrawer->update_vertex_buffer(real_points);
                pointsDrawer->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 1.0f));
                pointsDrawer->set_point_size(real_point_size);
                viewer.add_drawable(pointsDrawer);
            }


            // 라인 그리기
            std::vector<easy3d::vec3> line_points;
            for (auto& shape : shapes_) {
                auto [dummy0, line_point, dummy1, dummy2] = trianglation(shape);
                for (auto& p : line_point) {
                    line_points.push_back(easy3d::vec3((float)p[0], (float)p[1], (float)p[2]));
                }
            }
            if (line_points.size() != 0) {
                linesDrawer->update_vertex_buffer(line_points);
                linesDrawer->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 1.0f));
                linesDrawer->set_line_width(line_point_size);
                viewer.add_drawable(linesDrawer);
            }



            // solid 표면 그리기
            std::vector<easy3d::vec3> points;
            std::vector<unsigned int> indices;
            //unsigned int pos_str = 0;
            pos_str.push_back(0);
            for (int ii = 0; auto & shape : shapes_) {
                auto [dummy0, dummy1, pos, f2v] = trianglation(shape);

                for (auto& p : pos) {
                    points.push_back(easy3d::vec3((float)p[0], (float)p[1], (float)p[2]));
                }
                for (auto& vs : f2v) {
                    indices.push_back(pos_str[ii] + static_cast<unsigned int>(vs[0]));
                    indices.push_back(pos_str[ii] + static_cast<unsigned int>(vs[1]));
                    indices.push_back(pos_str[ii] + static_cast<unsigned int>(vs[2]));
                }
                pos_str.push_back(pos_str[ii++] + static_cast<unsigned int>(pos.size()));
                pos_sizes.push_back(static_cast<unsigned int>(pos.size()));
            }

            if (points.size() != 0) {
                surfaceDrawer->update_vertex_buffer(points, true);
                surfaceDrawer->update_element_buffer(indices);
                surfaceDrawer->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 1.0f));
                viewer.add_drawable(surfaceDrawer);
            }



            //viewer.fit_screen();
            viewer.camera()->setZClippingCoefficient(10.0f);
            //viewer.camera()->setPosition(vec3(0, 0, 4));
            //mat4 mvm = viewer.camera()->modelViewMatrix();
            //mvm(0, 3) = 2.0;
            //std::cout << mvm << std::endl;
            //viewer.camera()->set_modelview_matrix(mvm);
            viewer.set_animation(false);


            //auto mesh = new easy3d::SurfaceMesh;

            //{ // face 1: a concave quad
            //    auto v0 = mesh->add_vertex(easy3d::vec3(0, 0, 0));
            //    auto v1 = mesh->add_vertex(easy3d::vec3(800, 0, 0));
            //    auto v2 = mesh->add_vertex(easy3d::vec3(800, 800, 0));
            //    auto v3 = mesh->add_vertex(easy3d::vec3(600, 300, 0));
            //    mesh->add_quad(v0, v1, v2, v3);
            //}

            //{ // face 2: a self-intersecting face representing a star
            //    auto vertices = {
            //            mesh->add_vertex(easy3d::vec3(1500, 0, 0)),
            //            mesh->add_vertex(easy3d::vec3(1300, 800, 0)),
            //            mesh->add_vertex(easy3d::vec3(1100, 0, 0)),
            //            mesh->add_vertex(easy3d::vec3(1700, 500, 0)),
            //            mesh->add_vertex(easy3d::vec3(900, 500, 0))
            //    };
            //    mesh->add_face(vertices);
            //}

            //{ // face 3: a quad face with a hole
            //    auto vertices = {
            //            mesh->add_vertex(easy3d::vec3(1800, 0, 0)),
            //            mesh->add_vertex(easy3d::vec3(2200, 0, 0)),
            //            mesh->add_vertex(easy3d::vec3(2200, 700, 0)),
            //            mesh->add_vertex(easy3d::vec3(1800, 700, 0))
            //    };
            //    auto f = mesh->add_face(vertices);

            //    // let's create a hole (also a quad shape) in this face
            //    auto holes = mesh->add_face_property<std::vector<easy3d::vec3>>("f:holes");
            //    holes[f] = {
            //            easy3d::vec3(1900, 100, 0),
            //            easy3d::vec3(2100, 100, 0),
            //            easy3d::vec3(2100, 600, 0),
            //            easy3d::vec3(1900, 600, 0)
            //    };
            //}

            //auto triangulate = [](easy3d::SurfaceMesh* mesh) {
            //    if (!mesh)
            //        return;

            //    mesh->update_face_normals();
            //    auto normals = mesh->face_property<easy3d::vec3>("f:normal");
            //    auto holes = mesh->get_face_property<std::vector<easy3d::vec3>>("f:holes");

            //    easy3d::Tessellator tessellator;
            //    for (auto f : mesh->faces()) {
            //        tessellator.begin_polygon(normals[f]);

            //        tessellator.set_winding_rule(easy3d::Tessellator::WINDING_NONZERO);
            //        tessellator.begin_contour();
            //        for (auto h : mesh->halfedges(f)) {
            //            easy3d::SurfaceMesh::Vertex v = mesh->target(h);
            //            tessellator.add_vertex(mesh->position(v), v.idx());
            //        }
            //        tessellator.end_contour();

            //        if (holes && holes[f].size() >= 3) { // has a valid hole
            //            tessellator.set_winding_rule(easy3d::Tessellator::WINDING_ODD);
            //            tessellator.begin_contour();
            //            for (const auto& p : holes[f])
            //                tessellator.add_vertex(p);
            //            tessellator.end_contour();
            //        }

            //        tessellator.end_polygon();
            //    }

            //    // now the tessellation is done. We can clear the old mesh and
            //    // fill it will the new set of triangles

            //    mesh->clear();

            //    const auto& triangles = tessellator.elements();
            //    if (triangles.empty())
            //        return; // in degenerate cases num can be zero

            //    const auto& vts = tessellator.vertices();
            //    for (auto v : vts)
            //        mesh->add_vertex(easy3d::vec3(v->data()));

            //    for (const auto& t : triangles) {
            //        mesh->add_triangle(
            //            easy3d::SurfaceMesh::Vertex(static_cast<int>(t[0])),
            //            easy3d::SurfaceMesh::Vertex(static_cast<int>(t[1])),
            //            easy3d::SurfaceMesh::Vertex(static_cast<int>(t[2]))
            //        );
            //    }
            //    };


            //triangulate(mesh);


            //viewer.add_model(mesh, true);

        }



    };

}



//
//#include "./rigid_body.hpp"
//
//#include "./gjk.hpp"
//#include "./geometry_primer.hpp"
//
//#include "./src/geometry/geometry.hpp"
//
//namespace SimulMan {
//
//
//    class MyViewer : public easy3d::Viewer {
//    public:
//        std::shared_ptr<SimulMan::Shape> selected_model_ = nullptr;
//
//    protected:
//
//        bool key_press_event(int key, int modifiers) override {
//            if (true) {
//                Eigen::Vector3d translation(0, 0, 0);
//                Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
//                double ten_rad = 3.141592 / 180.0 * 10.0;
//                double angle = 0.0; // 3.141592 / 180.0 * 10.0;  // 예: 1도
//                switch (key) {
//                case KEY_W:  translation[0] = -0.1; break;
//                case KEY_S:  translation[0] = 0.1; break;
//                case KEY_D:  translation[1] = 0.1; break;
//                case KEY_A:  translation[1] = -0.1; break;
//                case KEY_Q:  translation[2] = 0.1; break;
//                case KEY_E:  translation[2] = -0.1; break;
//                case KEY_R:  axis = Eigen::Vector3d::UnitX(); angle = ten_rad; break;
//                case KEY_T:  axis = Eigen::Vector3d::UnitX(); angle = -ten_rad; break;
//                case KEY_F:  axis = Eigen::Vector3d::UnitY(); angle = ten_rad; break;
//                case KEY_G:  axis = Eigen::Vector3d::UnitY(); angle = -ten_rad; break;
//                case KEY_V:  axis = Eigen::Vector3d::UnitZ(); angle = ten_rad; break;
//                case KEY_B:  axis = Eigen::Vector3d::UnitZ(); angle = -ten_rad; break;
//                default: return false;
//                }
//                Eigen::AngleAxisd rotation(angle, axis);
//
//                for (auto& p : selected_model_->drawer.pos) {
//                    p += translation;
//                }
//
//                //selected_model_->orient = rotation * selected_model_->orient;
//                //selected_model_->xc += translation;
//                update();
//                return true;
//            }
//            return false;
//        }
//    };
//
//
//    class Renderer {
//    public:
//        using Shapes = std::vector<std::shared_ptr<SimulMan::Shape>>;
//        easy3d::Viewer viewer;
//        easy3d::TrianglesDrawable* surface;
//        std::vector<unsigned int> pos_sizes;
//        Shapes shapes_;
//
//        Renderer() = default;
//
//        void init() {
//            easy3d::initialize();
//            this->surface = new easy3d::TrianglesDrawable("faces");
//
//            init_easy3d_udf();
//
//            //set_animation();
//        }
//
//        int run() {
//            return viewer.run();
//        }
//
//        void set_animation() {
//
//            std::size_t face_size = 0;
//            for (std::size_t irigid = 0; irigid < shapes_.size(); ++irigid) {
//                auto& shape = shapes_[irigid];
//                face_size += shape->drawer.f2v.size();
//            }
//            surface->set_highlight_range(std::make_pair(0, face_size - 1));
//
//            viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
//                (void)v;
//                void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
//                easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
//                if (!vertices) return false;
//
//
//                for (std::size_t ishape = 0; ishape < shapes_.size(); ++ishape) {
//                    auto const& shape = shapes_[ishape];
//                    for (std::size_t i = 0; i < shape->drawer.pos.size(); ++i) {
//                        //rigid.pos[i] = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
//                        auto& p = shape->drawer.pos[i];
//                        //std::cout << i << " " << p.transpose() << std::endl;
//                        //vertices[pos_sizes[ishape] + i] = easy3d::vec3(p[0], p[1], p[2]);
//                    }
//                }
//
//                bool is_collide = false;
//
//                //GeometryPrimer geo;
//                //GeometryPrimer::AABB aabb;
//                //aabb.max = { -1.e200,-1.e200 ,-1.e200 };
//                //aabb.min = { 1.e200,1.e200 ,1.e200 };
//                //for (auto& p : rigids_[0].pos) {
//                //    for (int j = 0; j < 3; ++j) {
//                //        aabb.max[j] = std::max(aabb.max[j], p[j]);
//                //        aabb.min[j] = std::min(aabb.min[j], p[j]);
//                //    }
//                //}
//                //is_collide = geo.is_overlap(
//                //    GeometryPrimer::Sphere(Eigen::Vector3d(0.0, 0.0, 0.0), 0.5), aabb
//                //);
//
//                //ConvexityBased con;
//                //is_collide = con.intersection_gjk(rigids_[0].pos, rigids_[1].pos);
//                //if (is_collide) {
//                //    surface->set_highlight(true);
//                //}
//                //else {
//                //    surface->set_highlight(false);
//                //}
//
//                easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
//                viewer.update();
//
//                return true;
//                };
//
//        }
//
//        void init_easy3d_udf() {
//
//            // 그리드 생성
//            auto grid = new easy3d::LinesDrawable("grid");
//
//            // 그리드 데이터 설정
//            std::vector<easy3d::vec3> lines;
//            float size = 5.0f;
//            int divisions = 10;
//            float step = size / divisions;
//
//            for (int i = 0; i <= divisions; ++i) {
//                float pos = -size / 2 + i * step;
//                lines.push_back(easy3d::vec3(pos, -size / 2, 0));
//                lines.push_back(easy3d::vec3(pos, size / 2, 0));
//                lines.push_back(easy3d::vec3(-size / 2, pos, 0));
//                lines.push_back(easy3d::vec3(size / 2, pos, 0));
//            }
//
//            grid->update_vertex_buffer(lines);
//            grid->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 1.0f)); // 회색
//
//
//
//            // 뷰어에 그리드 추가
//            viewer.add_drawable(grid);
//
//
//            std::vector<easy3d::vec3> points;
//            for (auto& shape : shapes_) {
//                for (auto& p : shape->drawer.pos) {
//                    points.push_back(easy3d::vec3((float)p[0], (float)p[1], (float)p[2]));
//                }
//            }
//            // Each consecutive 3 indices represent a triangle.
//            pos_sizes.push_back(0);
//            std::vector<unsigned int> indices;
//            for (auto& shape : shapes_) {
//                for (auto& vs : shape->drawer.f2v) {
//                    indices.push_back(static_cast<unsigned int>(pos_sizes.back() + vs[0]));
//                    indices.push_back(static_cast<unsigned int>(pos_sizes.back() + vs[1]));
//                    indices.push_back(static_cast<unsigned int>(pos_sizes.back() + vs[2]));
//                }
//                pos_sizes.push_back(shape->drawer.pos.size());
//            }
//
//            //std::cout << shapes_[0]->drawer.pos.size() << std::endl;
//            //std::cout << shapes_[1]->drawer.pos.size() << std::endl;
//            //std::cout << points.size() << std::endl;
//            //std::cout << shapes_[0]->drawer.f2v.size() << std::endl;
//            //std::cout << shapes_[1]->drawer.f2v.size() << std::endl;
//            //std::cout << indices.size() << std::endl;
//            //-------------------------------------------------------------
//            // Create a TrianglesDrawable to visualize the surface of the "bunny".
//            surface->update_vertex_buffer(points, true);
//            surface->update_element_buffer(indices);
//
//            surface->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 1.0f));
//
//            viewer.add_drawable(surface);
//
//
//
//            //viewer.fit_screen();
//            viewer.camera()->setZClippingCoefficient(10.0f);
//            //viewer.camera()->setPosition(vec3(0, 0, 4));
//            //mat4 mvm = viewer.camera()->modelViewMatrix();
//            //mvm(0, 3) = 2.0;
//            //std::cout << mvm << std::endl;
//            //viewer.camera()->set_modelview_matrix(mvm);
//            viewer.set_animation(false);
//
//            for (auto& p : points) {
//                std::cout << p << std::endl;
//            }
//            std::cout << std::endl;
//            for (int i = 0; i < indices.size() / 3; ++i) {
//                std::cout << indices[i*3+0] << " " << indices[i * 3 + 1] <<  " " << indices[i * 3 + 2] << std::endl;
//            }
//
//        }
//
//
//
//    };
//
//
//
//    //class MyViewer : public easy3d::Viewer {
//    //public:
//    //    RigidBody* selected_model_ = nullptr;
//
//    //protected:
//
//    //    bool key_press_event(int key, int modifiers) override {
//    //        if (true) {
//    //            Eigen::Vector3d translation(0, 0, 0);
//    //            Eigen::Vector3d axis = Eigen::Vector3d::UnitX();
//    //            double ten_rad = 3.141592 / 180.0 * 10.0;
//    //            double angle = 0.0; // 3.141592 / 180.0 * 10.0;  // 예: 1도
//    //            switch (key) {
//    //            case KEY_W:  translation[0] = -0.1; break;
//    //            case KEY_S:  translation[0] = 0.1; break;
//    //            case KEY_D:  translation[1] = 0.1; break;
//    //            case KEY_A:  translation[1] = -0.1; break;
//    //            case KEY_Q:  translation[2] = 0.1; break;
//    //            case KEY_E:  translation[2] = -0.1; break;
//    //            case KEY_R:  axis = Eigen::Vector3d::UnitX(); angle = ten_rad; break;
//    //            case KEY_T:  axis = Eigen::Vector3d::UnitX(); angle = -ten_rad; break;
//    //            case KEY_F:  axis = Eigen::Vector3d::UnitY(); angle = ten_rad; break;
//    //            case KEY_G:  axis = Eigen::Vector3d::UnitY(); angle = -ten_rad; break;
//    //            case KEY_V:  axis = Eigen::Vector3d::UnitZ(); angle = ten_rad; break;
//    //            case KEY_B:  axis = Eigen::Vector3d::UnitZ(); angle = -ten_rad; break;
//    //            default: return false;
//    //            }
//    //            Eigen::AngleAxisd rotation(angle, axis);
//    //            selected_model_->orient = rotation * selected_model_->orient;
//    //            selected_model_->xc += translation;
//    //            update();
//    //            return true;
//    //        }
//    //        return false;
//    //    }
//    //};
//
//    //class Renderer {
//    //public:
//    //    MyViewer viewer;
//    //    //easy3d::Viewer viewer;
//    //    easy3d::TrianglesDrawable* surface;
//    //    std::vector<unsigned int> pos_sizes;
//    //    std::vector<RigidBody>& rigids_;
//
//    //    Renderer(std::vector<RigidBody>& rigids) : rigids_(rigids)
//    //    {
//    //        viewer.selected_model_ = &rigids.front();
//
//    //        easy3d::initialize();
//    //        this->surface = new easy3d::TrianglesDrawable("faces");
//
//    //        init_easy3d_udf(viewer, surface, rigids, pos_sizes);
//
//    //        set_animation();
//
//    //    }
//
//    //    int run() {
//    //        return viewer.run();
//    //    }
//
//    //    void set_animation() {
//
//    //        std::size_t face_size = 0;
//    //        for (std::size_t irigid = 0; irigid < rigids_.size(); ++irigid) {
//    //            auto& rigid = rigids_[irigid];
//    //            face_size += rigid.f2v.size();
//    //        }
//    //        surface->set_highlight_range(std::make_pair(0, face_size - 1));
//
//    //        viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
//    //            (void)v;
//    //            void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
//    //            easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
//    //            if (!vertices) return false;
//    //            for (std::size_t irigid = 0; irigid < rigids_.size(); ++irigid) {
//    //                auto& rigid = rigids_[irigid];
//    //                for (std::size_t i = 0; i < rigid.pos.size(); ++i) {
//    //                    rigid.pos[i] = rigid.xc + rigid.orient.toRotationMatrix() * rigid.r[i];
//    //                    vertices[pos_sizes[irigid] + i] = easy3d::vec3(
//    //                        rigid.pos[i][0], rigid.pos[i][1], rigid.pos[i][2]
//    //                    );
//    //                }
//    //            }
//
//    //            bool is_collide = false;
//
//    //            GeometryPrimer geo;
//    //            GeometryPrimer::AABB aabb;
//    //            aabb.max = { -1.e200,-1.e200 ,-1.e200 };
//    //            aabb.min = { 1.e200,1.e200 ,1.e200 };
//    //            for (auto& p : rigids_[0].pos) {
//    //                for (int j = 0; j < 3; ++j) {
//    //                    aabb.max[j] = std::max(aabb.max[j], p[j]);
//    //                    aabb.min[j] = std::min(aabb.min[j], p[j]);
//    //                }
//    //            }
//    //            is_collide = geo.is_overlap(
//    //                GeometryPrimer::Sphere(Eigen::Vector3d(0.0,0.0,0.0), 0.5), aabb
//    //            );
//
//    //            //ConvexityBased con;
//    //            //is_collide = con.intersection_gjk(rigids_[0].pos, rigids_[1].pos);
//    //            if (is_collide) {
//    //                surface->set_highlight(true);
//    //            }
//    //            else {
//    //                surface->set_highlight(false);
//    //            }
//
//    //            easy3d::VertexArrayObject::unmap_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer());
//    //            viewer.update();
//
//    //            return true;
//    //        };
//
//    //    }
//
//    //    void init_easy3d_udf(
//    //        easy3d::Viewer& viewer,
//    //        easy3d::TrianglesDrawable* surface,
//    //        std::vector<RigidBody>& rigids,
//    //        std::vector<unsigned int>& pos_sizes
//    //    ) {
//
//    //        // 그리드 생성
//    //        auto grid = new easy3d::LinesDrawable("grid");
//
//    //        // 그리드 데이터 설정
//    //        std::vector<easy3d::vec3> lines;
//    //        float size = 5.0f;
//    //        int divisions = 10;
//    //        float step = size / divisions;
//
//    //        for (int i = 0; i <= divisions; ++i) {
//    //            float pos = -size / 2 + i * step;
//    //            lines.push_back(easy3d::vec3(pos, -size / 2, 0));
//    //            lines.push_back(easy3d::vec3(pos, size / 2, 0));
//    //            lines.push_back(easy3d::vec3(-size / 2, pos, 0));
//    //            lines.push_back(easy3d::vec3(size / 2, pos, 0));
//    //        }
//
//    //        grid->update_vertex_buffer(lines);
//    //        grid->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 1.0f)); // 회색
//
//    //        
//
//    //        // 뷰어에 그리드 추가
//    //        viewer.add_drawable(grid);
//
//
//    //        std::vector<easy3d::vec3> points;
//    //        for (auto& rigid : rigids) {
//    //            for (auto& p : rigid.pos) {
//    //                points.push_back(easy3d::vec3(p[0], p[1], p[2]));
//    //            }
//    //        }
//    //        // Each consecutive 3 indices represent a triangle.
//    //        pos_sizes.push_back(0);
//    //        std::vector<unsigned int> indices;
//    //        for (auto& rigid : rigids) {
//    //            for (auto& vs : rigid.f2v) {
//    //                indices.push_back(static_cast<unsigned int>(pos_sizes.back() + vs[0]));
//    //                indices.push_back(static_cast<unsigned int>(pos_sizes.back() + vs[1]));
//    //                indices.push_back(static_cast<unsigned int>(pos_sizes.back() + vs[2]));
//    //            }
//    //            pos_sizes.push_back(pos_sizes.back() + rigid.pos.size());
//    //        }
//    //        //-------------------------------------------------------------
//    //        // Create a TrianglesDrawable to visualize the surface of the "bunny".
//    //        surface->update_vertex_buffer(points, true);
//    //        surface->update_element_buffer(indices);
//
//    //        surface->set_uniform_coloring(easy3d::vec4(0.5f, 0.5f, 0.5f, 1.0f));
//
//    //        viewer.add_drawable(surface);
//
//
//
//    //        //viewer.fit_screen();
//    //        viewer.camera()->setZClippingCoefficient(10.0f);
//    //        //viewer.camera()->setPosition(vec3(0, 0, 4));
//    //        //mat4 mvm = viewer.camera()->modelViewMatrix();
//    //        //mvm(0, 3) = 2.0;
//    //        //std::cout << mvm << std::endl;
//    //        //viewer.camera()->set_modelview_matrix(mvm);
//    //        viewer.set_animation(true);
//
//    //    }
//
//
//
//    //};
//
//
//}
