#pragma once

#include <vector>
#include <Eigen/Core>

class ConvexityBased {
public:
    using vec3 = Eigen::Vector3d;

    bool gjk(
        const std::vector<vec3>& shape1,
        const std::vector<vec3>& shape2,
        std::vector<vec3>& simplex
    ) {
        vec3 direction = vec3::UnitX();
        for (int i = 0; i < shape1.size(); ++i) {
            for (int j = i; j < shape2.size(); ++j) {
                direction = shape1[i] - shape2[j];
                if (!direction.isZero(1.e-12)) break;
            }
            if (!direction.isZero(1.e-12)) break;
        }
        simplex.push_back(support(shape1, shape2, direction));
        direction = -simplex[0];

        while (true) {
            vec3 A = support(shape1, shape2, direction);
            if (A.dot(direction) <= 0.0) {
                return false;
            }
            simplex.push_back(A);
            if (handleSimplex(simplex, direction)) {
                return true;
            }
        }
    }




        // Define a simplex class
        class Simplex {
        public:
            std::vector<vec3> points;

            void add(const vec3& point) {
                points.push_back(point);
            }

            vec3& operator[](size_t i) {
                return points[i];
            }

            size_t size() const {
                return points.size();
            }

            void remove(size_t index) {
                points.erase(points.begin() + index);
            }
        };

        vec3 furthest_point(
            const std::vector<vec3>& shape,
            const vec3& dir
        ) {
            double max_dot = -std::numeric_limits<double>::infinity();
            vec3 furthest;
            for (const auto& point : shape) {
                double dot_product = point.dot(dir);
                if (dot_product > max_dot) {
                    max_dot = dot_product;
                    furthest = point;
                }
            }
            return furthest;
        };

        vec3 support(
            const std::vector<vec3>& shape1,
            const std::vector<vec3>& shape2,
            const vec3& direction,
            double margin = 1.e-12  // ������ ���� margin �߰�
        ) {

            vec3 p1 = furthest_point(shape1, direction);
            vec3 p2 = furthest_point(shape2, -direction);
            vec3 support_point = p1 - p2;

            //// ������ �߰�
            //vec3 margin_dir = direction;
            //margin_dir = vec3(
            //    margin_dir[0] / std::sqrt(margin_dir.dot(margin_dir)),
            //    margin_dir[1] / std::sqrt(margin_dir.dot(margin_dir)),
            //    margin_dir[2] / std::sqrt(margin_dir.dot(margin_dir))
            //);
            //support_point += margin_dir * margin;

            return support_point;
        }

        bool handleSimplex(
            std::vector<vec3>& simplex,
            vec3& direction
        ) {
            if (simplex.size() == 2) {
                vec3 A = simplex[1];
                vec3 B = simplex[0];
                vec3 AB = B - A;
                vec3 AO = -A;
                if (AB.dot(AO) > 0) {
                    direction = AB.cross(AO).cross(AB);
                }
                else {
                    simplex.erase(simplex.begin() + 0);
                    direction = AO;
                }
            }
            else if (simplex.size() == 3) {
                vec3 A = simplex[2];
                vec3 B = simplex[1];
                vec3 C = simplex[0];
                vec3 AB = B - A;
                vec3 AC = C - A;
                vec3 AO = -A;
                vec3 ABC = AB.cross(AC);

                if (ABC.cross(AC).dot(AO) > 0) {
                    if (AC.dot(AO) > 0) {
                        simplex.erase(simplex.begin() + 1);
                        direction = AC.cross(AO).cross(AC);
                    }
                    else {
                        simplex.erase(simplex.begin() + 0);
                        handleSimplex(simplex, direction);
                    }
                }
                else {
                    if (AB.cross(ABC).dot(AO) > 0) {
                        simplex.erase(simplex.begin() + 0);
                        handleSimplex(simplex, direction);
                    }
                    else {
                        if (ABC.dot(AO) > 0) {
                            direction = ABC;
                        }
                        else {
                            std::swap(simplex[0], simplex[1]);
                            direction = -ABC;
                        }
                    }
                }
            }
            else if (simplex.size() == 4) {
                vec3 A = simplex[3];
                vec3 B = simplex[2];
                vec3 C = simplex[1];
                vec3 D = simplex[0];
                vec3 AB = B - A;
                vec3 AC = C - A;
                vec3 AD = D - A;
                vec3 AO = -A;

                vec3 ABC = AB.cross(AC);
                vec3 ACD = AC.cross(AD);
                vec3 ADB = AD.cross(AB);

                if (ABC.dot(AO) > 0) {
                    simplex.erase(simplex.begin() + 0);
                    handleSimplex(simplex, direction);
                }
                else if (ACD.dot(AO) > 0) {
                    simplex.erase(simplex.begin() + 2);
                    handleSimplex(simplex, direction);
                }
                else if (ADB.dot(AO) > 0) {
                    simplex.erase(simplex.begin() + 1);
                    handleSimplex(simplex, direction);
                }
                else {
                    return true;
                }
            }
            return false;
        }





        // https://github.com/kevinmoran/GJK
        vec3 support_test(const std::vector<vec3>& shape, vec3 dir) {
            //dir = matRS_inverse * dir;

            vec3 furthest_point = shape[0];
            double max_dot = furthest_point.dot(dir);

            for (int i = 1; i < shape.size(); ++i) {
                vec3 v = shape[i];
                double d = v.dot(dir);
                if (d > max_dot) {
                    max_dot = d;
                    furthest_point = v;
                }
            }
            return furthest_point;
            //vec3 result = matRS * furthest_point + pos; //convert support to world space
            //return result;
        }



        // https://github.com/kevinmoran/GJK
        void update_simplex3(
            vec3& a, vec3& b, vec3& c, vec3& d,
            int& simp_dim, vec3& search_dir
        ) {
            vec3 n = (b - a).cross(c - a);
            vec3 AO = -a;

            if ((b - a).cross(n).dot(AO) > 0) {
                c = a;
                search_dir = (b - a).cross(AO).cross(b - a);
            }
            else if (n.cross(c - a).dot(AO) > 0) {
                b = a;
                search_dir = (c - a).cross(AO).cross(c - a);
            }
            else {
                simp_dim = 3;
                if (n.dot(AO) > 0) {
                    d = c;
                    c = b;
                    b = a;
                    search_dir = n;
                }
                else {
                    d = b;
                    b = a;
                    search_dir = -n;
                }
            }
        }

        // https://github.com/kevinmoran/GJK
        bool update_simplex4(
            vec3& a, vec3& b, vec3& c, vec3& d,
            int& simp_dim, vec3& search_dir
        ) {
            vec3 ABC = (b - a).cross(c - a);
            vec3 ACD = (c - a).cross(d - a);
            vec3 ADB = (d - a).cross(b - a);
            vec3 AO = -a;
            simp_dim = 3;

            if (ABC.dot(AO) > 0) {
                d = c;
                c = b;
                b = a;
                search_dir = ABC;
                return false;
            }
            if (ACD.dot(AO) > 0) {
                b = a;
                search_dir = ACD;
                return false;
            }
            if (ADB.dot(AO) > 0) {
                c = d;
                d = b;
                b = a;
                search_dir = ADB;
                return false;
            }
            return true;
        }










    // https://github.com/kevinmoran/GJK
    std::pair<bool, vec3> gjk_epa_penetration(
        const std::vector<vec3>& shape1,
        const std::vector<vec3>& shape2
    ) {

        std::vector<vec3> simplex(4);
        vec3 search_dir;
        for (int i = 0; i < shape1.size(); ++i) {
            for (int j = i; j < shape2.size(); ++j) {
                search_dir = shape1[i] - shape2[j];
                if (!search_dir.isZero(1.e-12)) break;
            }
            if (!search_dir.isZero(1.e-12)) break;
        }
        simplex[1] = support_test(shape2, search_dir) - support_test(shape1, -search_dir);
        search_dir = -simplex[1];

        simplex[0] = support_test(shape2, search_dir) - support_test(shape1, -search_dir);
        if (simplex[0].dot(search_dir) < 0.0)
            return std::make_pair(false, vec3(semo::SEMO_MIN_VALUE, semo::SEMO_MIN_VALUE, semo::SEMO_MIN_VALUE));

        search_dir = (simplex[1] - simplex[0]).cross(-simplex[0]).cross(simplex[1] - simplex[0]);
        if (search_dir.isZero(1.e-12)) {
            search_dir = (simplex[1] - simplex[0]).cross(vec3(1, 0, 0));
            if (search_dir.isZero(1.e-12)) search_dir = (simplex[1] - simplex[0]).cross(vec3(0, 0, -1));
        }
        //std::cout << " aa " << search_dir.transpose() << std::endl;

        int simp_dim = 2;
        int GJK_MAX_NUM_ITERATIONS = 64;
        for (int iterations = 0; iterations < GJK_MAX_NUM_ITERATIONS; ++iterations) {
            simplex[simp_dim] = support_test(shape2, search_dir) - support_test(shape1, -search_dir);
            if (simplex[simp_dim].dot(search_dir) < 0)
                return std::make_pair(false, vec3(semo::SEMO_MIN_VALUE, semo::SEMO_MIN_VALUE, semo::SEMO_MIN_VALUE));
            ++simp_dim;

            if (simp_dim == 3) {
                update_simplex3(simplex[2], simplex[1], simplex[0], simplex[3], simp_dim, search_dir);
            }
            else if (update_simplex4(simplex[3], simplex[2], simplex[1], simplex[0], simp_dim, search_dir)) {
                return std::make_pair(true, epa2(shape1, shape2, simplex));
            }
        }
        return std::make_pair(false, vec3(semo::SEMO_MIN_VALUE, semo::SEMO_MIN_VALUE, semo::SEMO_MIN_VALUE));

    }




    // https://github.com/kevinmoran/GJK
    bool intersection_gjk(
        const std::vector<vec3>& shape1,
        const std::vector<vec3>& shape2
    ) {

        std::vector<vec3> simplex(4);
        vec3 search_dir;
        for (int i = 0; i < shape1.size(); ++i) {
            for (int j = i; j < shape2.size(); ++j) {
                search_dir = shape1[i] - shape2[j];
                if (!search_dir.isZero(1.e-12)) break;
            }
            if (!search_dir.isZero(1.e-12)) break;
        }
        simplex[1] = support_test(shape2, search_dir) - support_test(shape1, -search_dir);
        search_dir = -simplex[1];

        simplex[0] = support_test(shape2, search_dir) - support_test(shape1, -search_dir);
        if (simplex[0].dot(search_dir) < 0.0)
            return false;

        search_dir = (simplex[1] - simplex[0]).cross(-simplex[0]).cross(simplex[1] - simplex[0]);
        if (search_dir.isZero(1.e-12)) {
            search_dir = (simplex[1] - simplex[0]).cross(vec3(1, 0, 0));
            if (search_dir.isZero(1.e-12)) search_dir = (simplex[1] - simplex[0]).cross(vec3(0, 0, -1));
        }
        //std::cout << " aa " << search_dir.transpose() << std::endl;

        int simp_dim = 2;
        int GJK_MAX_NUM_ITERATIONS = 64;
        for (int iterations = 0; iterations < GJK_MAX_NUM_ITERATIONS; ++iterations) {
            simplex[simp_dim] = support_test(shape2, search_dir) - support_test(shape1, -search_dir);
            if (simplex[simp_dim].dot(search_dir) < 0)
                return false;
            ++simp_dim;

            if (simp_dim == 3) {
                update_simplex3(simplex[2], simplex[1], simplex[0], simplex[3], simp_dim, search_dir);
            }
            else if (update_simplex4(simplex[3], simplex[2], simplex[1], simplex[0], simp_dim, search_dir)) {
                return true;
            }
        }
        return false;

    }











    //// https://github.com/AlexanderFabisch/distance3d
    //void gjk_distance_original(
    //    const std::vector<vec3>& shape1,
    //    const std::vector<vec3>& shape2
    //) {
    //    struct SimplexInfo {
    //        std::vector<vec3> pos;
    //    };

    //    SimplexInfo simplex;
    //    simplex.pos.push_back(shape1[0] - shape2[0]);

    //    semo::point4_t barycentric_coordinates;
    //    vec3 search_direction;
    //    double distance_squared;

    //    int iteration = 0;
    //    bool backup = false;
    //    while (true) {
    //        ++iteration;

    //        // distance_subalgorithm_with_backup_procedure

    //        if (backup) {

    //        }
    //        else {

    //            // distance_subalgorithm
    //            // Johnson's distance subalgorithm.
    //            if (simplex.pos.size() == 1) {
    //                barycentric_coordinates[0] = 1.0;
    //                search_direction = simplex.pos[0];
    //                distance_squared = simplex.pos[0].dot(simplex.pos[0]);
    //            }
    //            else if (simplex.pos.size() == 2) {
    //                // _distance_subalgorithm_line_segment
    //                d[1,2] = simplex.pos[0].dot(simplex.pos[0]);
    //                if (d[1, 2] <= 0.0) {
    //                    simplex.pos[0];
    //                }
    //                d[0,2] = simplex.pos[1].dot(simplex.pos[1]);
    //                if (!(d[0, 2] <= 0.0 || d[1, 2] <= 0.0)) {
    //                    coords_sum = a + b;
    //                    barycentric_coordinates[0] = a / coords_sum;
    //                    barycentric_coordinates[1] = 1.0 - barycentric_coordinates[0];
    //                    search_direction = simplex.pos[0];
    //                    distance_squared = search_direction.dot(search_direction);
    //                }
    //                else {

    //                }
    //            }
    //            else if (simplex.pos.size() == 3) {

    //                // _distance_subalgorithm_face
    //            }
    //            else {
    //                // _distance_subalgorithm_tetrahedron

    //            }
    //        }


    //    }

    //}



    void gjk_triangle_libccd(
        std::vector<vec3>& simplex,
        vec3& search_direction,
        int& gjk_state,
        double EPSILON = 1.e-12
    ) {
        // triangle
        vec3 A = simplex[2];
        vec3 B = simplex[1];
        vec3 C = simplex[0];

        semo::point_to_triangle({ 0.0,0.0,0.0 }, {
                A, B, C
            });
        bool touching_contact =
            semo::point_to_triangle({ 0.0,0.0,0.0 }, {
                A, B, C
                }).isZero(1.e-12);
            if (touching_contact) {
                gjk_state = 0;
            }
            else {
                bool degenerated_triangle =
                    (A - B).isZero(1.e-12) || (A - C).isZero(1.e-12);

                if (degenerated_triangle) {
                    gjk_state = 1;
                }
                else {
                    vec3 AO = -A;
                    vec3 AB = B - A;
                    vec3 AC = C - A;
                    vec3 ABC = AB.cross(AC);

                    if (ABC.cross(AC).dot(AO) > -EPSILON) {
                        if (AC.dot(AO) > -EPSILON) {
                            simplex[1] = A;
                            simplex.pop_back();
                            search_direction = AC.cross(AO).cross(AC);
                        }
                        else {
                            if (AB.dot(AO) > -EPSILON) {
                                simplex[0] = B;
                                simplex[1] = A;
                                simplex.pop_back();
                                search_direction = AB.cross(AO).cross(AB);
                            }
                            else {
                                simplex[0] = A;
                                simplex.resize(1);
                                search_direction = AO;
                            }
                        }
                    }
                    else {
                        if (AB.cross(ABC).dot(AO) > -EPSILON) {
                            if (AB.dot(AO) > -EPSILON) {
                                simplex[0] = B;
                                simplex[1] = A;
                                simplex.pop_back();
                                search_direction = AB.cross(AO).cross(AB);
                            }
                            else {
                                simplex[0] = A;
                                simplex.resize(1);
                                search_direction = AO;
                            }
                        }
                        else {
                            if (ABC.dot(AO) > -EPSILON) {
                                search_direction = ABC;
                            }
                            else {
                                simplex[0] = B;
                                simplex[1] = C;
                                search_direction = -ABC;
                            }
                        }

                    }

                    gjk_state = 5;

                }
            }

    }


    // https://github.com/AlexanderFabisch/distance3d
    bool gjk_intersection_libccd(
        const std::vector<vec3>& shape1,
        const std::vector<vec3>& shape2,
        std::vector<vec3>& simplex
    ) {
        int max_iterations = 64;
        double EPSILON = 1.e-12;

        int CONTACT = 0;
        int NO_CONTACT = 1;
        int CONTINUE = 5;

        vec3 search_direction;
        for (int i = 0; i < shape1.size(); ++i) {
            for (int j = i; j < shape2.size(); ++j) {
                search_direction = shape1[i] - shape2[j];
                if (!search_direction.isZero(1.e-12)) break;
            }
            if (!search_direction.isZero(1.e-12)) break;
        }
        simplex.push_back(search_direction);

        int iteration = 0;
        while (iteration < max_iterations) {
            ++iteration;

            vec3 support_point =
                semo::furthest_point(shape1, search_direction) -
                semo::furthest_point(shape2, -search_direction);
            bool support_point_is_origin =
                support_point.dot(support_point) < EPSILON;
            if (support_point_is_origin) {
                return true;
            }

            support_point_is_origin =
                support_point.dot(search_direction) < -EPSILON;
            if (support_point_is_origin) {
                return false;
            }

            simplex.push_back(support_point);

            int gjk_state;
            // refine simplex
            if (simplex.size() == 2) {
                // line_segment
                vec3 A = simplex[1];
                vec3 B = simplex[0];

                vec3 AB = B - A;
                vec3 AO = -A;
                double origin_on_AB = AB.dot(AO);

                vec3 tmp = AB.cross(AO);
                bool origin_on_AB_segment =
                    std::abs(tmp.dot(tmp)) < EPSILON && origin_on_AB > 0.0;
                if (origin_on_AB_segment) {
                    gjk_state = 0;
                }
                else {
                    if (origin_on_AB < EPSILON) {
                        search_direction = AO;
                        simplex[0] = A;
                        simplex.pop_back();
                    }
                    // origin is closer to line segment
                    else {
                        search_direction = AB.cross(AO).cross(AB);
                    }
                    gjk_state = 5;
                }


            }
            else if (simplex.size() == 3) {
                // triangle
                semo::gjk_triangle_libccd(simplex, search_direction, gjk_state);

            }
            else {
                // tetrahedron
                vec3 A = simplex[3];
                vec3 B = simplex[2];
                vec3 C = simplex[1];
                vec3 D = simplex[0];

                bool degenerated_tetrahedron =
                    semo::point_to_triangle(A, {
                        B,C,D
                        }).isZero(1.e-12);
                if (degenerated_tetrahedron) {
                    gjk_state = NO_CONTACT;
                }
                else {
                    vec3 origin = { 0.0,0.0,0.0 };
                    bool origin_lies_on_tetrahedrons_face =
                        semo::point_to_triangle(origin, { A,B,C }).isZero(1.e-12) ||
                        semo::point_to_triangle(origin, { A,C,D }).isZero(1.e-12) ||
                        semo::point_to_triangle(origin, { A,B,D }).isZero(1.e-12) ||
                        semo::point_to_triangle(origin, { B,C,D }).isZero(1.e-12);
                    if (origin_lies_on_tetrahedrons_face) {
                        gjk_state = CONTACT;
                        simplex.pop_back();
                    }
                    else {
                        // compute AO, AB, AC, AD segments and ABC, ACD, ADB normal vectors
                        vec3 AO = -A;
                        vec3 AB = B - A;
                        vec3 AC = C - A;
                        vec3 AD = D - A;
                        vec3 ABC = AB.cross(AC);
                        vec3 ACD = AC.cross(AD);
                        vec3 ADB = AD.cross(AB);

                        // side (positive or negative) of B, C, D relative to planes ACD, ADB
                        // and ABC respectively 
                        bool B_on_ACD = ACD.dot(AB) > 0.0;
                        bool C_on_ADB = ADB.dot(AC) > 0.0;
                        bool D_on_ABC = ABC.dot(AD) > 0.0;

                        // whether origin is on same side of ACD, ADB, ABC as B, C, D
                        // respectively 
                        bool AB_O = (ACD.dot(AO) > 0.0) == B_on_ACD;
                        bool AC_O = (ADB.dot(AO) > 0.0) == C_on_ADB;
                        bool AD_O = (ABC.dot(AO) > 0.0) == D_on_ABC;

                        bool origin_is_in_tetrahedron =
                            AB_O && AC_O && AD_O;
                        if (origin_is_in_tetrahedron) {
                            gjk_state = CONTACT;
                        }
                        else {

                            simplex.resize(3);
                            if (!AB_O) {
                                simplex[2] = A;
                            }
                            else if (!AC_O) {
                                simplex[1] = D;
                                simplex[0] = B;
                                simplex[2] = A;
                            }
                            else {
                                simplex[0] = C;
                                simplex[1] = B;
                                simplex[2] = A;
                            }
                            semo::gjk_triangle_libccd(
                                simplex, search_direction, gjk_state);
                        }
                    }
                }

            }

            if (gjk_state == CONTACT) {
                return true;
            }
            else if (gjk_state == NO_CONTACT) {
                return false;
            }
            else if (search_direction.dot(search_direction) < EPSILON) {
                return false;
            }
        }
        return false;

    }


}

};
