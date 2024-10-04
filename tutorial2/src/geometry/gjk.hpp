#pragma once

#include <vector>
#include <Eigen/Core>

namespace SimulMan {

    // https://github.com/kevinmoran/GJK
    class GJK {
    public:
        using vec3 = Eigen::Vector3d;

        static bool is_overlap(
            const std::vector<vec3>& shape1,
            const std::vector<vec3>& shape2
        ) {

            vec3 A, B, C, D, search_dir;

            for (int i = 0; i < shape1.size(); ++i) {
                for (int j = i; j < shape2.size(); ++j) {
                    search_dir = shape1[i] - shape2[j];
                    if (!search_dir.isZero(1.e-12)) break;
                }
                if (!search_dir.isZero(1.e-12)) break;
            }
            C = support(shape2, search_dir) - support(shape1, -search_dir);
            search_dir = -C;

            B = support(shape2, search_dir) - support(shape1, -search_dir);
            if (B.dot(search_dir) < 0.0) return false;

            search_dir = (C - B).cross(-B).cross(C - B);
            // BC와 BO가 평행하다면,
            if (search_dir.isZero(1.e-12)) {
                // BC와 X축에 수직인 것으로 잡음
                search_dir = (C - B).cross(vec3(1, 0, 0));
                // BC와 X축이 평행하다면, BC와 -Z에 수직인 것으로 잡음
                if (search_dir.isZero(1.e-12)) search_dir = (C - B).cross(vec3(0, 0, -1));
            }

            int simp_dim = 2;
            int GJK_MAX_NUM_ITERATIONS = 64;
            for (int iterations = 0; iterations < GJK_MAX_NUM_ITERATIONS; ++iterations) {
                A = support(shape2, search_dir) - support(shape1, -search_dir);
                if (A.dot(search_dir) < 0) return false;

                ++simp_dim;

                if (simp_dim == 3) {
                    update_simplex3(A, B, C, D, simp_dim, search_dir);
                }
                else if (update_simplex4(A, B, C, D, simp_dim, search_dir)) {
                    return true;
                }
            }
            return false;


        }

    private:

        static vec3 support(const std::vector<vec3>& shape, vec3 dir) {
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



        static void update_simplex3(
            vec3& A, vec3& B, vec3& C, vec3& D,
            int& simp_dim, vec3& search_dir
        ) {
            // 가정 : A, B, C, BC voronoi 영역에는 O가 존재하지 않음.
            // -> AB, AC, +ABC, -ABC voronoi 영역만 판단하면 됨.
            // A는 가장 최근에 생성된 민코프키차
            // D는 여기선 안쓰임. 저장할때만 쓰임

            // △ABC 의 수직벡터
            vec3 ABC = (B - A).cross(C - A);
            // AO 벡터
            vec3 AO = -A;

            // AB의 voronoi 영역에 O가 있으면 true
            if ((B - A).cross(ABC).dot(AO) > 0) {
                simp_dim = 2;
                // A,B,A,D 순서로 저장
                C = A;
                // O방향쪽으로 AB수직벡터를 방향벡터로 설정
                search_dir = (B - A).cross(AO).cross(B - A);
            }
            // AC의 voronoi 영역에 O가 있으면 true
            else if (ABC.cross(C - A).dot(AO) > 0) {
                simp_dim = 2;
                // A,A,C,D 순서로 저장
                B = A;
                // O방향쪽으로 AC수직벡터를 방향벡터로 설정
                search_dir = (C - A).cross(AO).cross(C - A);
            }
            // 그렇지 않으면 △ABC +-voronoi 영역에 있는지 판단
            else {
                simp_dim = 3;
                // △ABC +voronoi 영역에 있으면 true
                if (ABC.dot(AO) > 0) {
                    // A,A,B,C 순서로 저장
                    D = C;
                    C = B;
                    B = A;
                    search_dir = ABC;
                }
                // △ABC -voronoi 영역에 있으면
                else {
                    // A,A,C,B 순서로 저장
                    D = B;
                    B = A;
                    search_dir = -ABC;
                }
            }
        }

        static bool update_simplex4(
            vec3& A, vec3& B, vec3& C, vec3& D,
            int& simp_dim, vec3& search_dir
        ) {
            // 가정 : 사면체 ABCD에서 △ABC 수직벡터 위에 D가 있다.
            // -> △DCB 수직벡터 위에 A가 있다.

            // △ABC 의 수직벡터
            vec3 ABC = (B - A).cross(C - A);
            // △ACD 의 수직벡터
            vec3 ACD = (C - A).cross(D - A);
            // △ADB 의 수직벡터
            vec3 ADB = (D - A).cross(B - A);
            // AO 벡터
            vec3 AO = -A;
            simp_dim = 3;

            // △ABC +voronoi 영역에 있으면 true
            if (ABC.dot(AO) > 0) {
                // A,A,B,C 순서로 저장
                D = C;
                C = B;
                B = A;
                search_dir = ABC;
                return false;
            }
            // △ACD +voronoi 영역에 있으면 true
            if (ACD.dot(AO) > 0) {
                // A,A,C,D 순서로 저장
                B = A;
                search_dir = ACD;
                return false;
            }
            // △ADB +voronoi 영역에 있으면 true
            if (ADB.dot(AO) > 0) {
                // A,A,D,B 순서로 저장
                C = D;
                D = B;
                B = A;
                search_dir = ADB;
                return false;
            }
            return true;
        }

    };


}
