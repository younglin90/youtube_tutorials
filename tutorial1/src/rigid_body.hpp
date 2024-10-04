#pragma once

#include <iostream>
#include <array>
#include <vector>
#include <Eigen/Dense>

class RigidBody {
public:
    using vec3 = Eigen::Vector3d;
    using mat3 = Eigen::Matrix3d;
    using quat = Eigen::Quaterniond;

	std::vector<vec3> pos;
	std::vector<std::vector<size_t>> f2v;
	std::vector<std::vector<size_t>> c2v;

    vec3 xc{}, vc{}, ac{};
    vec3 omega{}, alpha{};
	std::vector<vec3> r{};

    double density = 1.0;
    double mass{};
    double volume{};
    mat3 inertia{};
    mat3 inertiaInverse{};
    //vec3 force{};
    //vec3 torque{};

    quat orient = quat::Identity();

	bool scale(double in) {

		volume *= (in * in * in);
		mass *= (in * in * in);
		inertia *= (in * in * in);
		inertiaInverse /= (in * in * in);
		for (size_t i = 0; i < r.size(); ++i) {
			r[i] *= in;
		}


		return true;
	}

	bool CGS_union(const RigidBody& rigid) {

		auto vstr = this->pos.size();
		this->pos.insert(this->pos.end(), rigid.pos.begin(), rigid.pos.end());

		{
			for (const auto& inner_vec : rigid.f2v) {
				std::vector<size_t> temp;
				temp.reserve(inner_vec.size());
				for (const auto& value : inner_vec) {
					temp.push_back(value + vstr);
				}
				this->f2v.push_back(std::move(temp));
			}
		}
		{
			for (const auto& inner_vec : rigid.c2v) {
				std::vector<size_t> temp;
				temp.reserve(inner_vec.size());
				for (const auto& value : inner_vec) {
					temp.push_back(value + vstr);
				}
				this->c2v.push_back(std::move(temp));
			}
		}

		return true;
	}

	bool set_density(double new_density) {
		mass *= new_density / density;
		volume *= new_density / density;
		density = new_density;

		return true;
	}

	bool initialization() {

		if (pos.empty()) return false;
		if (f2v.empty()) return false;
		if (c2v.empty()) return false;

		calc_mass_and_center_of_mass();
		calc_rvec();
		calc_inertia_tensor();

		return true;
	}


    bool calc_mass_and_center_of_mass() {

		//vec3 xc_old = xc;
        xc.setZero();
        mass = 0.0;
        volume = 0.0;

        for (const auto& ivs : c2v) {

            // Calculate tetrahedron volume
            vec3 v1 = pos[ivs[1]] - pos[ivs[0]];
            vec3 v2 = pos[ivs[2]] - pos[ivs[0]];
            vec3 v3 = pos[ivs[3]] - pos[ivs[0]];
            auto& p0 = pos[ivs[0]];
            auto& p1 = pos[ivs[1]];
            auto& p2 = pos[ivs[2]];
            auto& p3 = pos[ivs[3]];

            double detJ = std::abs(v1.dot(v2.cross(v3)));

            double tet_volume = detJ / 6.0;
            double tet_mass = density * tet_volume;
            vec3 mass_center = (p0 + p1 + p2 + p3) * 0.25;

            volume += tet_volume;
            xc += tet_mass * mass_center;
            mass += tet_mass;

        }

		xc /= mass;

		//// pos 질량중심으로 이동
		//for (auto& p : pos) {
		//	p -= xc;
		//}
		//xc.setZero();

		//xc += xc_old;

        return true;

    }


	bool calc_rvec() {

		r.clear();
		r.reserve(pos.size());
		for (const auto& p : pos) {
			r.push_back(p - xc);
		}

		return true;
	}


	// Tonon, Fulvio. "Explicit exact formulas for the 3-D tetrahedron inertia tensor 
	// in terms of its vertex coordinates." Journal of Mathematics and Statistics 1.1 (2005): 8-11.
	bool calc_inertia_tensor() {

		inertia.setZero();

		for (const auto& ivs : c2v) {

			// Calculate tetrahedron volume
			vec3 v1 = pos[ivs[1]] - pos[ivs[0]];
			vec3 v2 = pos[ivs[2]] - pos[ivs[0]];
			vec3 v3 = pos[ivs[3]] - pos[ivs[0]];
			auto& p0 = pos[ivs[0]];
			auto& p1 = pos[ivs[1]];
			auto& p2 = pos[ivs[2]];
			auto& p3 = pos[ivs[3]];

			double x1 = p0[0]; double y1 = p0[1]; double z1 = p0[2];
			double x2 = p1[0]; double y2 = p1[1]; double z2 = p1[2];
			double x3 = p2[0]; double y3 = p2[1]; double z3 = p2[2];
			double x4 = p3[0]; double y4 = p3[1]; double z4 = p3[2];

			double detJ = std::abs(v1.dot(v2.cross(v3)));

			double I11 = density * detJ * (
				y1 * y1 + y2 * y2 + y3 * y3 + y3 * y4 + y4 * y4 + y2 * (y3 + y4) +
				y1 * (y2 + y3 + y4) + z1 * z1 + z2 * z2 +
				z2 * z3 + z3 * z3 + (z2 + z3) * z4 + z4 * z4 + z1 * (z2 + z3 + z4)
				//y1 * y1 + y1 * y2 + y2 * y2 + y1 * y3 + y2 * y3 + y3 * y3 + y1 * y4 +
				//y2 * y4 + y3 * y4 + y4 * y4 + z1 * z1 + z1 * z2 + z2 * z2 + z1 * z3 +
				//z2 * z3 + z3 * z3 + z1 * z4 + z2 * z4 + z3 * z4 + z4 * z4
				) / 60.0;
			double I22 = density * detJ * (
				x1 * x1 + x2 * x2 + x3 * x3 + x3 * x4 + x4 * x4 + x2 * (x3 + x4) +
				x1 * (x2 + x3 + x4) + z1 * z1 + z2 * z2 +
				z2 * z3 + z3 * z3 + (z2 + z3) * z4 + z4 * z4 + z1 * (z2 + z3 + z4)
				//x1 * x1 + x1 * x2 + x2 * x2 + x1 * x3 + x2 * x3 + x3 * x3 + x1 * x4 +
				//x2 * x4 + x3 * x4 + x4 * x4 + z1 * z1 + z1 * z2 + z2 * z2 + z1 * z3 +
				//z2 * z3 + z3 * z3 + z1 * z4 + z2 * z4 + z3 * z4 + z4 * z4
				) / 60.0;
			double I33 = density * detJ * (
				x1 * x1 + x2 * x2 + x3 * x3 + x3 * x4 + x4 * x4 + x2 * (x3 + x4) +
				x1 * (x2 + x3 + x4) + y1 * y1 + y2 * y2 +
				y2 * y3 + y3 * y3 + (y2 + y3) * y4 + y4 * y4 + y1 * (y2 + y3 + y4)
				//x1 * x1 + x1 * x2 + x2 * x2 + x1 * x3 + x2 * x3 + x3 * x3 + x1 * x4 +
				//x2 * x4 + x3 * x4 + x4 * x4 + y1 * y1 + y1 * y2 + y2 * y2 + y1 * y3 +
				//y2 * y3 + y3 * y3 + y1 * y4 + y2 * y4 + y3 * y4 + y4 * y4
				) / 60.0;
			double I23 = density * detJ * (
				-y3 * z1 - y4 * z1 - y3 * z2 - y4 * z2 - 2.0 * y3 * z3 - y4 * z3 - y3 * z4 - 2.0 * y4 * z4 -
				y1 * (2.0 * z1 + z2 + z3 + z4) - y2 * (z1 + 2.0 * z2 + z3 + z4)
				//2.0 * y1 * z1 + y2 * z1 + y3 * z1 + y4 * z1 + y1 * z2 +
				//2.0 * y2 * z2 + y3 * z2 + y4 * z2 + y1 * z3 + y2 * z3 + 2.0 * y3 * z3 +
				//y4 * z3 + y1 * z4 + y2 * z4 + y3 * z4 + 2.0 * y4 * z4
				) / 120.0;
			double I12 = density * detJ * (
				-x3 * y1 - x4 * y1 - x3 * y2 - x4 * y2 - 2.0 * x3 * y3 - x4 * y3 - x3 * y4 - 2.0 * x4 * y4 -
				x1 * (2.0 * y1 + y2 + y3 + y4) - x2 * (y1 + 2.0 * y2 + y3 + y4)
				//2.0 * x1 * z1 + x2 * z1 + x3 * z1 + x4 * z1 + x1 * z2 +
				//2.0 * x2 * z2 + x3 * z2 + x4 * z2 + x1 * z3 + x2 * z3 + 2.0 * x3 * z3 +
				//x4 * z3 + x1 * z4 + x2 * z4 + x3 * z4 + 2.0 * x4 * z4
				) / 120.0;
			double I13 = density * detJ * (
				-x3 * z1 - x4 * z1 - x3 * z2 - x4 * z2 - 2.0 * x3 * z3 - x4 * z3 - x3 * z4 - 2.0 * x4 * z4 -
				x1 * (2.0 * z1 + z2 + z3 + z4) - x2 * (z1 + 2.0 * z2 + z3 + z4)
				//2.0 * x1 * y1 + x2 * y1 + x3 * y1 + x4 * y1 + x1 * y2 +
				//2.0 * x2 * y2 + x3 * y2 + x4 * y2 + x1 * y3 + x2 * y3 + 2.0 * x3 * y3 +
				//x4 * y3 + x1 * y4 + x2 * y4 + x3 * y4 + 2.0 * x4 * y4
				) / 120.0;

			mat3 Iab;
			Iab(0, 0) = I11;
			Iab(1, 1) = I22;
			Iab(2, 2) = I33;
			Iab(0, 1) = I12;
			Iab(0, 2) = I13;
			Iab(1, 2) = I23;
			Iab(1, 0) = Iab(0, 1);
			Iab(2, 0) = Iab(0, 2);
			Iab(2, 1) = Iab(1, 2);

			inertia += Iab;

		}

		vec3 r = xc;
		mat3 translated_inertia =
			mass * (r.squaredNorm() * Eigen::Matrix3d::Identity() - r * r.transpose());
		inertia -= translated_inertia;

		inertiaInverse = inertia.inverse();

		return true;
	}



	bool update_Crank_Nicolson(double dt, vec3 force, vec3 torque) {

		int nSub = 1;
		int niter = 1;
		double h = dt / static_cast<double>(nSub);
		for (int iSub = 0; iSub < nSub; ++iSub) {
			vec3 vc_old = vc;
			vec3 xc_old = xc;
			vec3 omega_old = omega;
			for (int iter = 0; iter < niter; ++iter) {
				// 선형 운동량 : 속도, 위치 업데이트
				//tex: $$ \Delta v = -(v^{k}-v^{n})+h F_{ext} / m $$
				vec3 dvel = -(vc - vc_old) + h * force / mass;
				vc += dvel;
				xc = xc_old + 0.5 * dt * (vc + vc_old);

				// 각 운동량 : 각속도, orient 업데이트
				//tex: $$ \Delta\omega_{W}=-(\omega^{k}-\omega^{n})+h I^{-1} T_{ext}$$
				vec3 domega1_w = -(omega - omega_old) + h * inertiaInverse * torque;
				omega += domega1_w;

				// 각 운동량 : 자이로스코픽 항
				vec3 omega_b = world2body(omega);
				vec3 domega2_exp_b = -h * inertiaInverse * (skew(omega_b) * inertia * omega_b);
				mat3 domega2_imp_c_b = inertia + 
					h * (skew(omega_b) * inertia - skew(inertia * omega_b));
				vec3 domega2_imp_b = -h * (
					domega2_imp_c_b.inverse() * 
					(skew(omega_b) * inertia * omega_b));
				omega_b += 0.5 * (domega2_exp_b + domega2_imp_b);
				omega = body2world(omega_b);

				vec3 axis = h * omega; //0.5 * h * omega;
				double angle = axis.norm();
				axis.normalize();
				Eigen::Quaterniond dq(Eigen::AngleAxisd(angle, axis));
				orient = dq * orient;
				orient.normalize();
			}

		}

		return true;
	}

	vec3 world2body(const vec3& p) {
		double s = orient.w();
		vec3 v = -orient.vec();
		return (s * s - v.dot(v)) * p + 2.0 * p.dot(v) * v + 2.0 * s * v.cross(p);

	}

	vec3 body2world(const vec3& p) {
		double s = orient.w();
		vec3 v = orient.vec();
		return (s * s - v.dot(v)) * p + 2.0 * p.dot(v) * v + 2.0 * s * v.cross(p);
	}


	mat3 skew(const vec3& v) {
		mat3 res;
		res <<
			0.0, -v[2], v[1],
			v[2], 0.0, -v[0],
			-v[1], v[0], 0.0;
		return res;
	}


};


class RigidBodyGenerator {
public:
    using vec3 = Eigen::Vector3d;

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
	RigidBody hexahedron(
		vec3 v0, vec3 v1, vec3 v2, vec3 v3, vec3 v4, vec3 v5, vec3 v6, vec3 v7
	) {
        RigidBody rigid;
        rigid.pos = { v0,v1,v2,v3,v4,v5,v6,v7 };
        rigid.c2v = {
            {0, 1, 2, 5},
            {0, 2, 3, 7},
            {0, 5, 4, 7},
            {2, 5, 6, 7},
            {0, 2, 5, 7}
        };
        rigid.f2v = {
            { 0, 1, 2 },
            { 0, 1, 5 },
            { 1, 2, 5 },
            { 0, 2, 3 },
            { 0, 3, 7 },
            { 2, 3, 7 },
            { 0, 5, 4 },
            { 0, 4, 7 },
            { 5, 4, 7 },
            { 2, 5, 6 },
            { 2, 6, 7 },
            { 5, 6, 7 }
        };
        return rigid;
	}



	//        5
	//       /|\
	//      / | \
	//     /  |  \
	//    3 --+-- 4
	//    |   |   |
	//    |   2   |
	//    |  / \  |
	//    | /   \ |
	//    |/     \|
	//    0 ----- 1
	RigidBody prism(
		vec3 v0, vec3 v1, vec3 v2, vec3 v3, vec3 v4, vec3 v5
	) {
		RigidBody rigid;
		rigid.pos = { v0,v1,v2,v3,v4,v5 };
		rigid.c2v = {
			{0, 1, 2, 3},
			{1, 2, 3, 4},
			{2, 3, 4, 5}
		};
		rigid.f2v = {
			{ 0, 1, 2 },
			{ 3, 4, 5 },
			{ 0, 1, 3 },
			{ 1, 3, 4 },
			{ 0, 2, 3 },
			{ 2, 3, 5 },
			{ 1, 2, 4 },
			{ 2, 4, 5 }
		};
		return rigid;
	}

};