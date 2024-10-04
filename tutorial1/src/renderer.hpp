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


#include "./rigid_body.hpp"

namespace SimulMan {

    class Renderer {
    public:
        easy3d::Viewer viewer;
        easy3d::TrianglesDrawable* surface;
        std::vector<unsigned int> pos_sizes;

        Renderer(std::vector<RigidBody>& rigids) {

            easy3d::initialize();
            this->surface = new easy3d::TrianglesDrawable("faces");

            init_easy3d_udf(viewer, surface, rigids, pos_sizes);

        }

        int run() {
            return viewer.run();
        }

        void set_animation() {

            viewer.animation_func_ = [&](easy3d::Viewer* v) -> bool {
                (void)v;
                void* pointer = easy3d::VertexArrayObject::map_buffer(GL_ARRAY_BUFFER, surface->vertex_buffer(), GL_WRITE_ONLY);
                easy3d::vec3* vertices = reinterpret_cast<easy3d::vec3*>(pointer);
                if (!vertices) return false;

                return true;
                };

        }

        void init_easy3d_udf(
            easy3d::Viewer& viewer,
            easy3d::TrianglesDrawable* surface,
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


            std::vector<easy3d::vec3> points;
            for (auto& rigid : rigids) {
                for (auto& p : rigid.pos) {
                    points.push_back(easy3d::vec3(p[0], p[1], p[2]));
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



    };


}
