#pragma once

#include <utility>
#include <algorithm>
#include "rigid_body.hpp"

namespace PBD {


	class PositionBasedConstraints {
	public:
		using vec3 = Eigen::Vector3d;
		using vec4 = Eigen::Vector4d;
		using mat3 = Eigen::Matrix3d;
		using mat4 = Eigen::Matrix4d;
		using quat = Eigen::Quaterniond;
		using pair_t = std::pair<RigidBody*, RigidBody*>;
		using funcs_t = std::function<void(RigidBody*, RigidBody*)>;
		using inform_t = Eigen::Matrix<double, 4, 12>;
		//std::vector<pair_t> rigids;
		//std::vector<funcs_t> funcs;
		//std::vector<inform_t> informs;

		//void add(RigidBody* rigid0, RigidBody* rigid1,
		//	funcs_t func = [](RigidBody*, RigidBody*) {}
		//) {
		//	rigids.push_back(std::make_pair(rigid0, rigid1));
		//	funcs.push_back(func);
		//	informs.push_back({});
		//}

		//void update() {

		//	for (int i = 0; i < rigids.size(); ++i) {
		//		funcs[i](rigids[i].first, rigids[i].second);
		//	}

		//}



		//===========================================
		// ball joint
		static void init_ball_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			vec3 x_joint,
			inform_t& inform
		) {
			Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();
			Eigen::Matrix3d R1T = q1.toRotationMatrix().transpose();

			// 0: r vector (=xc to xjoint) in body 0 (local)
			// 1: r vector (=xc to xjoint) in body 1 (local)
			// 2: r vector (=xc to xjoint) in body 0 (global)
			// 3: r vector (=xc to xjoint) in body 1 (global)

			inform.setZero();
			//tex: $$ \Delta x=Rr \rightarrow r=R^{T}\Delta x $$
			inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			inform.block<3, 1>(0, 1) = R1T * (x_joint - x1);
			inform.block<3, 1>(0, 2) = x_joint;
			inform.block<3, 1>(0, 3) = x_joint;

		}
		static void update_ball_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			inform_t& inform
		) {
			Eigen::Matrix3d R0 = q0.toRotationMatrix();
			Eigen::Matrix3d R1 = q1.toRotationMatrix();

			inform.block<3, 1>(0, 2) = R0 * inform.block<3, 1>(0, 0) + x0;
			inform.block<3, 1>(0, 3) = R1 * inform.block<3, 1>(0, 1) + x1;

		}
		static void solve_ball_joint(
			const vec3& x0, const quat& q0,
			const double& mass0,
			const mat3& inertiaInverse0,
			const vec3& x1, const quat& q1,
			const double& mass1,
			const mat3& inertiaInverse1,
			inform_t& inform,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
			const auto& x_joint_b0 = inform.block<3, 1>(0, 2);
			const auto& x_joint_b1 = inform.block<3, 1>(0, 3);

			mat3 K1 = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);
			mat3 K2 = calcK(x_joint_b1, 1.0 / mass1, x1, inertiaInverse1);

			vec3 C = x_joint_b0 - x_joint_b1;
			mat3 K = K1 + K2;
			vec3 dlambda = K.llt().solve(-C);

			dx0 = dlambda / mass0;
			vec3 r0 = x_joint_b0 - x0;
			//tex:$$ -\tilde{r}^{T} = \tilde{r}=r \times   $$
			vec3 dr0 = inertiaInverse0 * r0.cross(dlambda);
			dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
			dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();

			dx1 = -dlambda / mass1;
			vec3 r1 = x_joint_b1 - x1;
			vec3 dr1 = -inertiaInverse1 * r1.cross(dlambda);
			dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
			dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();

		}



		//===========================================
		// particle ball joint
		static void init_particleBall_joint(
			const vec3& x0,
			const quat& q0,
			const vec3& x_joint,
			inform_t& inform
		) {
			//auto& q0 = rigid0->orient;
			Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();
			//auto& x0 = rigid0->xc;

			// 0: r vector (=xc to xjoint) in body 0 (local)
			// 1: r vector (=xc to xjoint) in body 0 (global)
			// 2: 고정 joint 위치 (global)
			inform.setZero();
			inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			inform.block<3, 1>(0, 1) = x_joint;
			inform.block<3, 1>(0, 2) = x_joint;
		}
		static void update_particleBall_joint(
			const vec3& x0,
			const quat& q0,
			inform_t& inform
		) {
			//auto& q0 = rigid0->orient;
			Eigen::Matrix3d R0 = q0.toRotationMatrix();
			//auto& x0 = rigid0->xc;
			const auto& r0 = inform.block<3, 1>(0, 0);

			inform.block<3, 1>(0, 1) = x0 + R0 * r0;
		}
		static void solve_particleBall_joint(
			const vec3& x0,
			const quat& q0,
			const double& mass0,
			const mat3& inertiaInverse0,
			inform_t& inform,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
			//auto& q0 = rigid0->orient;
			const auto& x_joint_b0 = inform.block<3, 1>(0, 1);
			const auto& x_joint = inform.block<3, 1>(0, 2);

			mat3 K = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);
			//mat3 K2 = (1.0 / mass0) * mat3::Identity();
			//mat3 alpha = stiffness / (dt * dt) * mat3::Identity();
			//mat3 K = K1 + K2 + alpha;

			vec3 C = x_joint_b0 - x_joint;
			//vec3 dlambda = K.llt().solve(-C - alpha * lambda);
			vec3 dlambda = K.llt().solve(-C);
			//lambda += dlambda;

			dx0 = dlambda / mass0;
			vec3 r0 = x_joint_b0 - x0;
			vec3 dr0 = inertiaInverse0 * r0.cross(dlambda);
			dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
			dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();

			dx1.setZero();
			dq1.coeffs().setZero();
		}
		//===========================================



		////===========================================
		//// distance joint
		//void init_distance_joint(RigidBody* rigid0, RigidBody* rigid1,
		//	vec3 x_joint,
		//	inform_t& inform
		//) {
		//	auto& x0 = rigid0->xc;
		//	auto& q0 = rigid0->orient;
		//	Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();
		//	auto& x1 = rigid1->xc;
		//	auto& q1 = rigid1->orient;
		//	Eigen::Matrix3d R1T = q1.toRotationMatrix().transpose();

		//	// 0: r vector (=xc to xjoint) in body 0 (local)
		//	// 1: r vector (=xc to xjoint) in body 1 (local)
		//	// 2: r vector (=xc to xjoint) in body 0 (global)
		//	// 3: r vector (=xc to xjoint) in body 1 (global)
		//	inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
		//	inform.block<3, 1>(0, 1) = R1T * (x_joint - x1);
		//	inform.block<3, 1>(0, 2) = x_joint;
		//	inform.block<3, 1>(0, 3) = x_joint;

		//}
		//void update_distance_joint(RigidBody* rigid0, RigidBody* rigid1,
		//	inform_t& inform
		//) {
		//	auto& x0 = rigid0->xc;
		//	auto& q0 = rigid0->orient;
		//	Eigen::Matrix3d R0 = q0.toRotationMatrix();
		//	auto& x1 = rigid1->xc;
		//	auto& q1 = rigid1->orient;
		//	Eigen::Matrix3d R1 = q1.toRotationMatrix();

		//	inform.block<3, 1>(0, 2) = R0 * inform.block<3, 1>(0, 0) + x0;
		//	inform.block<3, 1>(0, 3) = R1 * inform.block<3, 1>(0, 1) + x1;

		//}
		//void solve_distance_joint(RigidBody* rigid0, RigidBody* rigid1,
		//	inform_t& inform,
		//	const double& targetLenth,
		//	const double& stiffness,
		//	const double& dt,
		//	double& lambda,
		//	vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		//) {
		//	auto& q0 = rigid0->orient;
		//	auto& q1 = rigid1->orient;
		//	const auto& x_joint_b0 = inform.block<3, 1>(0, 2);
		//	const auto& x_joint_b1 = inform.block<3, 1>(0, 3);

		//	mat3 K1 = calcK(x_joint_b0, 1.0 / rigid0->mass, rigid0->xc, rigid0->inertiaInverse);
		//	mat3 K2 = calcK(x_joint_b1, 1.0 / rigid1->mass, rigid1->xc, rigid1->inertiaInverse);

		//	double length = (x_joint_b0 - x_joint_b1).norm();
		//	vec3 dir = (x_joint_b0 - x_joint_b1).normalized();
		//	double C = length - targetLenth;

		//	double alpha = 1.0 / (stiffness * dt * dt);
		//	double K = (dir.transpose() * (K1 + K2)).dot(dir) + alpha;
		//	double Kinv = 1.0 / K;

		//	double dlambda_mag = Kinv * (-C - alpha * lambda);
		//	lambda += dlambda_mag;
		//	vec3 dlambda = dlambda_mag * dir;

		//	dx0 = dlambda / rigid0->mass;
		//	vec3 r0 = x_joint_b0 - rigid0->xc;
		//	vec3 dr0 = rigid0->inertiaInverse * r0.cross(dlambda);
		//	dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
		//	dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();


		//	dx1 = -dlambda / rigid1->mass;
		//	vec3 r1 = x_joint_b1 - rigid1->xc;
		//	vec3 dr1 = -rigid1->inertiaInverse * r1.cross(dlambda);
		//	dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
		//	dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();

		//}
		////===========================================


		//===========================================
		// universal joint
		static void init_universal_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			const vec3& x_joint,
			const vec3& axis_joint0,
			const vec3& axis_joint1,
			inform_t& inform
		) {
			Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();
			Eigen::Matrix3d R1T = q1.toRotationMatrix().transpose();

			// 0: r vector (=xc to xjoint) in body 0 (local)
			// 1: r vector (=xc to xjoint) in body 1 (local)
			// 2: r vector (=xc to xjoint) in body 0 (global)
			// 3: r vector (=xc to xjoint) in body 1 (global)
			// 4: constraint axis in body 0 (local)
			// 5: constraint axis in body 1 (local)
			// 6: constraint axis in body 0 (global)
			// 7: constraint axis in body 1 (global)
			inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			inform.block<3, 1>(0, 1) = R1T * (x_joint - x1);
			inform.block<3, 1>(0, 2) = x_joint;
			inform.block<3, 1>(0, 3) = x_joint;
			inform.block<3, 1>(0, 4) = R0T * axis_joint0.normalized();
			inform.block<3, 1>(0, 5) = R1T * axis_joint1.normalized();
			inform.block<3, 1>(0, 6) = axis_joint0.normalized();
			inform.block<3, 1>(0, 7) = axis_joint1.normalized();
		}
		static void update_universal_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			inform_t& inform
		) {
			Eigen::Matrix3d R0 = q0.toRotationMatrix();
			Eigen::Matrix3d R1 = q1.toRotationMatrix();

			inform.block<3, 1>(0, 2) = R0 * inform.block<3, 1>(0, 0) + x0;
			inform.block<3, 1>(0, 3) = R1 * inform.block<3, 1>(0, 1) + x1;
			inform.block<3, 1>(0, 6) = (R0 * inform.block<3, 1>(0, 4)).normalized();
			inform.block<3, 1>(0, 7) = (R1 * inform.block<3, 1>(0, 5)).normalized();
		}
		static void solve_universal_joint(
			const vec3& x0, const quat& q0,
			const double& mass0,
			const mat3& inertiaInverse0,
			const vec3& x1, const quat& q1,
			const double& mass1,
			const mat3& inertiaInverse1,
			inform_t& inform,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
			const auto& x_joint_b0 = inform.block<3, 1>(0, 2);
			const auto& x_joint_b1 = inform.block<3, 1>(0, 3);
			const auto& axis_joint_b0 = inform.block<3, 1>(0, 6);
			const auto& axis_joint_b1 = inform.block<3, 1>(0, 7);

			vec3 r_joint0 = x_joint_b0 - x0;
			vec3 r_joint1 = x_joint_b1 - x1;

			Eigen::Matrix<double, 4, 1> C;
			C.block<3, 1>(0, 0) = x_joint_b0 - x_joint_b1;
			C(3, 0) = axis_joint_b0.dot(axis_joint_b1);

			Eigen::Matrix<double, 4, 4> K = Eigen::Matrix<double, 4, 4>::Zero();
			mat3 K1 = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);
			mat3 K2 = calcK(x_joint_b1, 1.0 / mass1, x1, inertiaInverse1);

			vec3 u = axis_joint_b0.cross(axis_joint_b1);

			K.block<3, 3>(0, 0) = K1 + K2;
			K.block<3, 1>(0, 3) = -(
				skew(r_joint0) * inertiaInverse0 * u +
				skew(r_joint1) * inertiaInverse1 * u);
			K.block<1, 3>(3, 0) = K.block<3, 1>(0, 3).transpose();
			K(3, 3) = u.transpose() *
				(inertiaInverse0 + inertiaInverse1) * u;

			Eigen::Matrix<double, 4, 1> dlambda = K.llt().solve(-C);
			vec3 dlambda_X = dlambda.block<3, 1>(0, 0);
			vec3 u_dlambda_q = u * dlambda(3, 0);

			dx0 = dlambda_X / mass0;
			vec3 r0 = x_joint_b0 - x0;
			vec3 dr0 = inertiaInverse0 * (r0.cross(dlambda_X) + u_dlambda_q);
			dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
			dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();


			dx1 = -dlambda_X / mass1;
			vec3 r1 = x_joint_b1 - x1;
			vec3 dr1 = -inertiaInverse1 * (r1.cross(dlambda_X) + u_dlambda_q);
			dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
			dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();
		}
		//===========================================




		//===========================================
		// hinge joint
		static void init_hinge_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			const vec3& x_joint,
			const vec3& axis_joint,
			inform_t& inform
		) {
			Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();
			Eigen::Matrix3d R1T = q1.toRotationMatrix().transpose();

			mat3 R0 = orthogonal_basis_matrix(axis_joint, { 1.0,0.0,0.0 });
			quat qR(R0);
			quat qRq0 = qR.conjugate() * q0;
			quat q1qR = q1.conjugate() * qR;
			mat4 L_qRq0 = quat_to_left_matrix(qRq0);
			mat4 R_q1qR = quat_to_right_matrix(q1qR);
			Eigen::Matrix<double, 2, 4> mat_proj =
				(R_q1qR * L_qRq0).block<2, 4>(2, 0);

			//// 0: r vector (=xc to xjoint) in body 0 (local)
			//// 1: r vector (=xc to xjoint) in body 1 (local)
			//// 2: r vector (=xc to xjoint) in body 0 (global)
			//// 3: r vector (=xc to xjoint) in body 1 (global)
			//// 4: projection matrix for the rotational part
			//// 5: hinge axis in body 0 (local)
			//inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			//inform.block<3, 1>(0, 1) = R1T * (x_joint - x1);
			//inform.block<3, 1>(0, 2) = x_joint;
			//inform.block<3, 1>(0, 3) = x_joint;
			//inform.block<4, 2>(0, 4) = mat_proj.transpose();
			//inform.block<3, 1>(0, 6) = R0T * axis_joint.normalized();

			//====================================
			// 0: r vector (=xc to xjoint) in body 0 (local)
			// 1: r vector (=xc to xjoint) in body 1 (local)
			// 2: r vector (=xc to xjoint) in body 0 (global)
			// 3: r vector (=xc to xjoint) in body 1 (global)
			// 4: constraint axis s in body 0 (local)
			// 5: constraint axis t in body 0 (local)
			// 6: constraint axis n in body 1 (local)
			// 7: constraint axis s in body 0 (global)
			// 8: constraint axis t in body 0 (global)
			// 9: constraint axis n in body 1 (global)
			inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			inform.block<3, 1>(0, 1) = R1T * (x_joint - x1);
			inform.block<3, 1>(0, 2) = x_joint;
			inform.block<3, 1>(0, 3) = x_joint;
			inform.block<3, 1>(0, 4) = R0T * R0.col(1).normalized();
			inform.block<3, 1>(0, 5) = R0T * R0.col(2).normalized();
			inform.block<3, 1>(0, 6) = R1T * axis_joint.normalized();
			inform.block<3, 1>(0, 7) = R0.col(1).normalized();
			inform.block<3, 1>(0, 8) = R0.col(2).normalized();
			inform.block<3, 1>(0, 9) = axis_joint.normalized();
			//====================================

		}
		static void update_hinge_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			inform_t& inform
		) {
			Eigen::Matrix3d R0 = q0.toRotationMatrix();
			Eigen::Matrix3d R1 = q1.toRotationMatrix();

			inform.block<3, 1>(0, 2) = R0 * inform.block<3, 1>(0, 0) + x0;
			inform.block<3, 1>(0, 3) = R1 * inform.block<3, 1>(0, 1) + x1;

			//====================================
			inform.block<3, 1>(0, 7) = R0 * inform.block<3, 1>(0, 4);
			inform.block<3, 1>(0, 8) = R0 * inform.block<3, 1>(0, 5);
			inform.block<3, 1>(0, 9) = R1 * inform.block<3, 1>(0, 6);
			//====================================


		}
		static void solve_hinge_joint(
			const vec3& x0, const quat& q0,
			const double& mass0,
			const mat3& inertiaInverse0,
			const vec3& x1, const quat& q1,
			const double& mass1,
			const mat3& inertiaInverse1,
			inform_t& inform,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
			//====================================
			const auto& x_joint_b0 = inform.block<3, 1>(0, 2);
			const auto& x_joint_b1 = inform.block<3, 1>(0, 3);
			const auto& axis_s0 = inform.block<3, 1>(0, 7);
			const auto& axis_t0 = inform.block<3, 1>(0, 8);
			const auto& axis1 = inform.block<3, 1>(0, 9);

			vec3 r_joint0 = x_joint_b0 - x0;
			vec3 r_joint1 = x_joint_b1 - x1;

			Eigen::Matrix<double, 5, 1> C;
			C.block<3, 1>(0, 0) = x_joint_b0 - x_joint_b1;
			C(3, 0) = axis_s0.dot(axis1);
			C(4, 0) = axis_t0.dot(axis1);

			mat3 K1 = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);
			mat3 K2 = calcK(x_joint_b1, 1.0 / mass1, x1, inertiaInverse1);

			Eigen::Matrix<double, 3, 2> u;
			u.col(0) = axis_s0.cross(axis1);
			u.col(1) = axis_t0.cross(axis1);

			Eigen::Matrix<double, 5, 5> K = Eigen::Matrix<double, 5, 5>::Zero();
			K.block<3, 3>(0, 0) = K1 + K2;
			K.block<3, 2>(0, 3) = -(
				skew(r_joint0) * inertiaInverse0 * u +
				skew(r_joint1) * inertiaInverse1 * u);
			K.block<2, 3>(3, 0) = K.block<3, 2>(0, 3).transpose();
			K.block<2, 2>(3, 3) = u.transpose() *
				(inertiaInverse0 + inertiaInverse1) * u;

			Eigen::Matrix<double, 5, 1> dlambda = K.llt().solve(-C);
			vec3 dlambda_X = dlambda.block<3, 1>(0, 0);
			vec3 u_dlambda_q = u * dlambda.block<2, 1>(3, 0);

			dx0 = dlambda_X / mass0;
			vec3 r0 = x_joint_b0 - x0;
			vec3 dr0 = inertiaInverse0 * (r0.cross(dlambda_X) + u_dlambda_q);
			dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
			dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();


			dx1 = -dlambda_X / mass1;
			vec3 r1 = x_joint_b1 - x1;
			vec3 dr1 = -inertiaInverse1 * (r1.cross(dlambda_X) + u_dlambda_q);
			dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
			dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();
			//====================================


		}
		//===========================================


		//===========================================
		// particle hinge joint
		static void init_particleHinge_joint(
			const vec3& x0,
			const quat& q0,
			const vec3& x_joint,
			const vec3& axis_joint,
			inform_t& inform
		) {
			Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();

			mat3 R0 = orthogonal_basis_matrix(axis_joint, { 1.0,0.0,0.0 });

			//====================================
			// 0: r vector (=xc to xjoint) in body 0 (local)
			// 1: r vector (=xc to xjoint) in body 1 (local)
			// 2: r vector (=xc to xjoint) in body 0 (global)
			// 3: r vector (=xc to xjoint) in body 1 (global)
			// 4: constraint axis s in body 0 (local)
			// 5: constraint axis t in body 0 (local)
			// 6: constraint axis n in body 1 (local)
			// 7: constraint axis s in body 0 (global)
			// 8: constraint axis t in body 0 (global)
			// 9: constraint axis n in body 1 (global)
			inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			inform.block<3, 1>(0, 2) = x_joint;
			inform.block<3, 1>(0, 3) = x_joint;
			inform.block<3, 1>(0, 4) = R0T * R0.col(1).normalized();
			inform.block<3, 1>(0, 5) = R0T * R0.col(2).normalized();
			inform.block<3, 1>(0, 7) = R0.col(1).normalized();
			inform.block<3, 1>(0, 8) = R0.col(2).normalized();
			inform.block<3, 1>(0, 9) = axis_joint.normalized();
			//====================================


		}
		static void update_particleHinge_joint(
			const vec3& x0,
			const quat& q0,
			inform_t& inform
		) {
			Eigen::Matrix3d R0 = q0.toRotationMatrix();

			inform.block<3, 1>(0, 2) = R0 * inform.block<3, 1>(0, 0) + x0;

			//====================================
			inform.block<3, 1>(0, 7) = R0 * inform.block<3, 1>(0, 4);
			inform.block<3, 1>(0, 8) = R0 * inform.block<3, 1>(0, 5);
			//====================================
		}
		static void solve_particleHinge_joint(
			const vec3& x0,
			const quat& q0,
			const double& mass0,
			const mat3& inertiaInverse0,
			inform_t& inform,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
			const auto& x_joint_b0 = inform.block<3, 1>(0, 2);
			const auto& x_joint = inform.block<3, 1>(0, 3);
			const auto& axis_s0 = inform.block<3, 1>(0, 7);
			const auto& axis_t0 = inform.block<3, 1>(0, 8);
			const auto& axis1 = inform.block<3, 1>(0, 9);

			vec3 r_joint0 = x_joint_b0 - x0;

			Eigen::Matrix<double, 5, 1> C;
			C.block<3, 1>(0, 0) = x_joint_b0 - x_joint;
			C(3, 0) = axis_s0.dot(axis1);
			C(4, 0) = axis_t0.dot(axis1);

			mat3 K1 = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);

			Eigen::Matrix<double, 3, 2> u;
			u.col(0) = axis_s0.cross(axis1);
			u.col(1) = axis_t0.cross(axis1);

			Eigen::Matrix<double, 5, 5> K = Eigen::Matrix<double, 5, 5>::Zero();
			K.block<3, 3>(0, 0) = K1;
			K.block<3, 2>(0, 3) = -(
				skew(r_joint0) * inertiaInverse0 * u);
			K.block<2, 3>(3, 0) = K.block<3, 2>(0, 3).transpose();
			K.block<2, 2>(3, 3) = u.transpose() * (inertiaInverse0) * u;

			Eigen::Matrix<double, 5, 1> dlambda = K.llt().solve(-C);
			vec3 dlambda_X = dlambda.block<3, 1>(0, 0);
			vec3 u_dlambda_q = u * dlambda.block<2, 1>(3, 0);

			dx0 = dlambda_X / mass0;
			vec3 r0 = x_joint_b0 - x0;
			vec3 dr0 = inertiaInverse0 * (r0.cross(dlambda_X) + u_dlambda_q);
			dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
			dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();

			dx1.setZero();
			dq1.coeffs().setZero();


		}
		//===========================================




		//===========================================
		// cylindrical joint
		static void init_cylindrical_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			vec3 x_joint,
			vec3 axis_joint,
			inform_t& inform
		) {
			Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();
			Eigen::Matrix3d R1T = q1.toRotationMatrix().transpose();

			mat3 R0 = orthogonal_basis_matrix(axis_joint, { 1.0,0.0,0.0 });

			// 0: r vector (=xc to xjoint) in body 0 (local)
			// 1: r vector (=xc to xjoint) in body 1 (local)
			// 2: r vector (=xc to xjoint) in body 0 (global)
			// 3: r vector (=xc to xjoint) in body 1 (global)
			// 4: constraint axis s in body 0 (local)
			// 5: constraint axis t in body 0 (local)
			// 6: constraint axis n in body 1 (local)
			// 7: constraint axis s in body 0 (global)
			// 8: constraint axis t in body 0 (global)
			// 9: constraint axis n in body 1 (global)
			inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			inform.block<3, 1>(0, 1) = R1T * (x_joint - x1);
			inform.block<3, 1>(0, 2) = x_joint;
			inform.block<3, 1>(0, 3) = x_joint;
			inform.block<3, 1>(0, 4) = R0T * R0.col(1).normalized();
			inform.block<3, 1>(0, 5) = R0T * R0.col(2).normalized();
			inform.block<3, 1>(0, 6) = R1T * axis_joint.normalized();
			inform.block<3, 1>(0, 7) = R0.col(1).normalized();
			inform.block<3, 1>(0, 8) = R0.col(2).normalized();
			inform.block<3, 1>(0, 9) = axis_joint.normalized();

		}
		static void update_cylindrical_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			inform_t& inform
		) {
			Eigen::Matrix3d R0 = q0.toRotationMatrix();
			Eigen::Matrix3d R1 = q1.toRotationMatrix();

			inform.block<3, 1>(0, 2) = R0 * inform.block<3, 1>(0, 0) + x0;
			inform.block<3, 1>(0, 3) = R1 * inform.block<3, 1>(0, 1) + x1;

			inform.block<3, 1>(0, 7) = R0 * inform.block<3, 1>(0, 4);
			inform.block<3, 1>(0, 8) = R0 * inform.block<3, 1>(0, 5);
			inform.block<3, 1>(0, 9) = R1 * inform.block<3, 1>(0, 6);

		}
		static void solve_cylindrical_joint(
			const vec3& x0, const quat& q0,
			const double& mass0,
			const mat3& inertiaInverse0,
			const vec3& x1, const quat& q1,
			const double& mass1,
			const mat3& inertiaInverse1,
			inform_t& inform,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
			const auto& x_joint_b0 = inform.block<3, 1>(0, 2);
			const auto& x_joint_b1 = inform.block<3, 1>(0, 3);
			const auto& axis_s0 = inform.block<3, 1>(0, 7);
			const auto& axis_t0 = inform.block<3, 1>(0, 8);
			const auto& axis1 = inform.block<3, 1>(0, 9);

			vec3 r_joint0 = x_joint_b0 - x0;
			vec3 r_joint1 = x_joint_b1 - x1;

			Eigen::Matrix<double, 4, 1> C;
			C(0, 0) = axis_s0.dot(x_joint_b0 - x_joint_b1);
			C(1, 0) = axis_t0.dot(x_joint_b0 - x_joint_b1);
			C(2, 0) = axis_s0.dot(axis1);
			C(3, 0) = axis_t0.dot(axis1);

			mat3 K1 = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);
			mat3 K2 = calcK(x_joint_b1, 1.0 / mass1, x1, inertiaInverse1);

			Eigen::Matrix<double, 3, 2> u;
			u.col(0) = axis_s0.cross(axis1);
			u.col(1) = axis_t0.cross(axis1);

			Eigen::Matrix<double, 2, 3> st;
			st.block<1, 3>(0, 0) = axis_s0.transpose();
			st.block<1, 3>(1, 0) = axis_t0.transpose();
			Eigen::Matrix<double, 2, 2> K_M = st * (K1 + K2) * st.transpose();

			Eigen::Matrix<double, 4, 4> K = Eigen::Matrix<double, 4, 4>::Zero();
			K.block<2, 2>(0, 0) = K_M;
			K.block<2, 2>(0, 2) = -st * (
				skew(r_joint0) * inertiaInverse0 * u +
				skew(r_joint1) * inertiaInverse1 * u);
			K.block<2, 2>(2, 0) = K.block<2, 2>(0, 2).transpose();
			K.block<2, 2>(2, 2) = u.transpose() *
				(inertiaInverse0 + inertiaInverse1) * u;

			Eigen::Matrix<double, 4, 1> dlambda = K.llt().solve(-C);
			vec3 dlambda_X = st.transpose() * dlambda.block<2, 1>(0, 0);
			vec3 u_dlambda_q = u * dlambda.block<2, 1>(2, 0);

			dx0 = dlambda_X / mass0;
			vec3 r0 = x_joint_b0 - x0;
			vec3 dr0 = inertiaInverse0 * (r0.cross(dlambda_X) + u_dlambda_q);
			dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
			dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();


			dx1 = -dlambda_X / mass1;
			vec3 r1 = x_joint_b1 - x1;
			vec3 dr1 = -inertiaInverse1 * (r1.cross(dlambda_X) + u_dlambda_q);
			dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
			dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();

		}
		//===========================================



		//===========================================
		// prismatic joint
		static void init_prismatic_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			vec3 x_joint,
			vec3 axis_joint,
			inform_t& inform
		) {
			Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();
			Eigen::Matrix3d R1T = q1.toRotationMatrix().transpose();

			mat3 R0 = orthogonal_basis_matrix(axis_joint, { 1.0,0.0,0.0 });

			// 0: r vector (=xc to xjoint) in body 0 (local)
			// 1: r vector (=xc to xjoint) in body 1 (local)
			// 2: r vector (=xc to xjoint) in body 0 (global)
			// 3: r vector (=xc to xjoint) in body 1 (global)
			// 4: constraint axis s in body 0 (local)
			// 5: constraint axis t in body 0 (local)
			// 6: constraint axis n in body 1 (local)
			// 7: constraint axis s in body 0 (global)
			// 8: constraint axis t in body 0 (global)
			// 9: constraint axis n in body 1 (global)
			// 10: constraint axis s in body 1 (local)
			// 11: constraint axis s in body 1 (global)
			inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			inform.block<3, 1>(0, 1) = R1T * (x_joint - x1);
			inform.block<3, 1>(0, 2) = x_joint;
			inform.block<3, 1>(0, 3) = x_joint;
			inform.block<3, 1>(0, 4) = R0T * R0.col(1).normalized();
			inform.block<3, 1>(0, 5) = R0T * R0.col(2).normalized();
			inform.block<3, 1>(0, 6) = R1T * axis_joint.normalized();
			inform.block<3, 1>(0, 7) = R0.col(1).normalized();
			inform.block<3, 1>(0, 8) = R0.col(2).normalized();
			inform.block<3, 1>(0, 9) = axis_joint.normalized();
			inform.block<3, 1>(0, 10) = R1T * R0.col(1).normalized();
			inform.block<3, 1>(0, 11) = R0.col(1).normalized();

		}
		static void update_prismatic_joint(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			inform_t& inform
		) {
			Eigen::Matrix3d R0 = q0.toRotationMatrix();
			Eigen::Matrix3d R1 = q1.toRotationMatrix();

			inform.block<3, 1>(0, 2) = R0 * inform.block<3, 1>(0, 0) + x0;
			inform.block<3, 1>(0, 3) = R1 * inform.block<3, 1>(0, 1) + x1;

			inform.block<3, 1>(0, 7) = R0 * inform.block<3, 1>(0, 4);
			inform.block<3, 1>(0, 8) = R0 * inform.block<3, 1>(0, 5);
			inform.block<3, 1>(0, 9) = R1 * inform.block<3, 1>(0, 6);

			inform.block<3, 1>(0, 11) = R1 * inform.block<3, 1>(0, 10);

		}
		static void solve_prismatic_joint(
			const vec3& x0, const quat& q0,
			const double& mass0,
			const mat3& inertiaInverse0,
			const vec3& x1, const quat& q1,
			const double& mass1,
			const mat3& inertiaInverse1,
			inform_t& inform,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
			const auto& x_joint_b0 = inform.block<3, 1>(0, 2);
			const auto& x_joint_b1 = inform.block<3, 1>(0, 3);
			const auto& axis_s0 = inform.block<3, 1>(0, 7);
			const auto& axis_t0 = inform.block<3, 1>(0, 8);
			const auto& axis_n1 = inform.block<3, 1>(0, 9);
			const auto& axis_s1 = inform.block<3, 1>(0, 11);

			vec3 r_joint0 = x_joint_b0 - x0;
			vec3 r_joint1 = x_joint_b1 - x1;

			Eigen::Matrix<double, 5, 1> C;
			C(0, 0) = axis_s0.dot(x_joint_b0 - x_joint_b1);
			C(1, 0) = axis_t0.dot(x_joint_b0 - x_joint_b1);
			Eigen::AngleAxisd theta0_tmp(q0);
			Eigen::AngleAxisd theta1_tmp(q1);
			vec3 theta0 = theta0_tmp.angle() * theta0_tmp.axis();
			vec3 theta1 = theta1_tmp.angle() * theta1_tmp.axis();
			C.block<3, 1>(2, 0) = theta0 - theta1;

			mat3 K1 = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);
			mat3 K2 = calcK(x_joint_b1, 1.0 / mass1, x1, inertiaInverse1);

			//Eigen::Matrix<double, 3, 3> u;
			//u.col(0) = -axis_s0.cross(axis_n1);
			//u.col(1) = -axis_t0.cross(axis_n1);

			Eigen::Matrix<double, 2, 3> st;
			st.block<1, 3>(0, 0) = axis_s0.transpose();
			st.block<1, 3>(1, 0) = axis_t0.transpose();
			Eigen::Matrix<double, 2, 2> K_M = st * (K1 + K2) * st.transpose();

			Eigen::Matrix<double, 5, 5> K = Eigen::Matrix<double, 5, 5>::Zero();
			K.block<2, 2>(0, 0) = K_M;
			K.block<2, 3>(0, 2) = -st * (
				skew(r_joint0) * inertiaInverse0 +
				skew(r_joint1) * inertiaInverse1);
			K.block<3, 2>(2, 0) = K.block<2, 3>(0, 2).transpose();
			K.block<3, 3>(2, 2) = (inertiaInverse0 + inertiaInverse1);

			Eigen::Matrix<double, 5, 1> dlambda = K.llt().solve(-C);
			vec3 dlambda_X = st.transpose() * dlambda.block<2, 1>(0, 0);
			vec3 u_dlambda_q = dlambda.block<3, 1>(2, 0);

			dx0 = dlambda_X / mass0;
			vec3 r0 = x_joint_b0 - x0;
			vec3 dr0 = inertiaInverse0 * (r0.cross(dlambda_X) + u_dlambda_q);
			dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
			dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();


			dx1 = -dlambda_X / mass1;
			vec3 r1 = x_joint_b1 - x1;
			vec3 dr1 = -inertiaInverse1 * (r1.cross(dlambda_X) + u_dlambda_q);
			dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
			dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();

		}
		//===========================================




		//===========================================
		// plane 부등식
		static void init_plane_inequality(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			vec3 x_joint,
			vec3 normal,
			inform_t& inform
		) {
			Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();
			Eigen::Matrix3d R1T = q1.toRotationMatrix().transpose();

			// 0: r vector (=xc to xjoint) in body 0 (local)
			// 1: r vector (=xc to xjoint) in body 1 (local)
			// 2: r vector (=xc to xjoint) in body 0 (global)
			// 3: r vector (=xc to xjoint) in body 1 (global)
			inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			inform.block<3, 1>(0, 1) = R1T * (x_joint - x1);
			inform.block<3, 1>(0, 2) = x_joint;
			inform.block<3, 1>(0, 3) = x_joint;
			inform.block<3, 1>(0, 4) = normal;

		}
		static void update_plane_inequality(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			inform_t& inform
		) {
			Eigen::Matrix3d R0 = q0.toRotationMatrix();
			Eigen::Matrix3d R1 = q1.toRotationMatrix();

			inform.block<3, 1>(0, 2) = R0 * inform.block<3, 1>(0, 0) + x0;
			inform.block<3, 1>(0, 3) = R1 * inform.block<3, 1>(0, 1) + x1;

		}
		static void solve_plane_inequality(
			const vec3& x0, const quat& q0,
			const double& mass0,
			const mat3& inertiaInverse0,
			const vec3& x1, const quat& q1,
			const double& mass1,
			const mat3& inertiaInverse1,
			const std::vector<vec3>& r1s,
			inform_t& inform,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
			const vec3& x_joint_b0 = inform.block<3, 1>(0, 2);
			const vec3& x_joint_b1 = inform.block<3, 1>(0, 3);
			const vec3& normal = inform.block<3, 1>(0, 4);

			Eigen::Matrix3d R1 = q1.toRotationMatrix();

			double d_min = 1.e100;
			for (int i = 0; i < r1s.size(); ++i) {
				vec3 p = x1 + R1 * r1s[i];
				double d = (p - x_joint_b0).dot(normal);
				if (d < d_min) d_min = d;
			}

			double C = 0.0 - d_min;
			if (C < 0.0) {
				dx0 = vec3(0.0, 0.0, 0.0);
				dq0 = quat(0.0, 0.0, 0.0, 0.0);
				dx1 = vec3(0.0, 0.0, 0.0);
				dq1 = quat(0.0, 0.0, 0.0, 0.0);
				return;
			}

			mat3 K1 = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);
			mat3 K2 = calcK(x_joint_b1, 1.0 / mass1, x1, inertiaInverse1);

			double alpha = stiffness / (dt * dt);
			vec3 K = (K1 + K2 + alpha * mat3::Identity()) * normal;
			vec3 dlambda = { 0.0,0.0,0.0 };
			if (K[0] != 0.0) dlambda[0] = (-C - alpha * lambda[0]) / K[0];
			if (K[1] != 0.0) dlambda[1] = (-C - alpha * lambda[1]) / K[1];
			if (K[2] != 0.0) dlambda[2] = (-C - alpha * lambda[2]) / K[2];
			lambda += dlambda;

			//std::cout << dlambda.transpose() << std::endl;

			dx0 = dlambda / mass0;
			vec3 r0 = x_joint_b0 - x0;
			vec3 dr0 = inertiaInverse0 * r0.cross(dlambda);
			dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
			dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();


			dx1 = -dlambda / mass1;
			vec3 r1 = x_joint_b1 - x1;
			vec3 dr1 = -inertiaInverse1 * r1.cross(dlambda);
			dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
			dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();


		}
		//===========================================


		//===========================================
		// 힌지 부등식
		static void init_hinge_inequality(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			vec3 x_joint,
			vec3 axis_joint,
			double limit_angle,
			inform_t& inform
		) {
			Eigen::Matrix3d R0T = q0.toRotationMatrix().transpose();
			Eigen::Matrix3d R1T = q1.toRotationMatrix().transpose();

			mat3 R0 = orthogonal_basis_matrix(axis_joint, { 1.0,0.0,0.0 });
			quat qR(R0);
			quat qRq0 = qR.conjugate() * q0;
			quat q1qR = q1.conjugate() * qR;
			mat4 L_qRq0 = quat_to_left_matrix(qRq0);
			mat4 R_q1qR = quat_to_right_matrix(q1qR);
			Eigen::Matrix<double, 2, 4> mat_proj =
				(R_q1qR * L_qRq0).block<2, 4>(2, 0);

			// 0: r vector (=xc to xjoint) in body 0 (local)
			// 1: r vector (=xc to xjoint) in body 1 (local)
			// 2: r vector (=xc to xjoint) in body 0 (global)
			// 3: r vector (=xc to xjoint) in body 1 (global)
			// 4: constraint axis s in body 0 (local)
			// 5: constraint axis t in body 0 (local)
			// 6: constraint axis n in body 1 (local)
			// 7: constraint axis s in body 0 (global)
			// 8: constraint axis t in body 0 (global)
			// 9: constraint axis n in body 1 (global)
			inform.block<3, 1>(0, 0) = R0T * (x_joint - x0);
			inform.block<3, 1>(0, 1) = R1T * (x_joint - x1);
			inform.block<3, 1>(0, 2) = x_joint;
			inform.block<3, 1>(0, 3) = x_joint;
			inform.block<3, 1>(0, 4) = R0T * R0.col(1).normalized();
			inform.block<3, 1>(0, 5) = R0T * R0.col(2).normalized();
			inform.block<3, 1>(0, 6) = R1T * axis_joint.normalized();
			inform.block<3, 1>(0, 7) = R0.col(1).normalized();
			inform.block<3, 1>(0, 8) = R0.col(2).normalized();
			inform.block<3, 1>(0, 9) = axis_joint.normalized();
			inform.block<3, 1>(0, 10) = R1T * R0.col(1).normalized();

			inform(3, 0) = limit_angle;

		}
		static void update_hinge_inequality(
			const vec3& x0, const quat& q0,
			const vec3& x1, const quat& q1,
			inform_t& inform
		) {
			Eigen::Matrix3d R0 = q0.toRotationMatrix();
			Eigen::Matrix3d R1 = q1.toRotationMatrix();

			inform.block<3, 1>(0, 2) = R0 * inform.block<3, 1>(0, 0) + x0;
			inform.block<3, 1>(0, 3) = R1 * inform.block<3, 1>(0, 1) + x1;

			inform.block<3, 1>(0, 7) = R0 * inform.block<3, 1>(0, 4);
			inform.block<3, 1>(0, 8) = R0 * inform.block<3, 1>(0, 5);
			inform.block<3, 1>(0, 9) = R1 * inform.block<3, 1>(0, 6);
			inform.block<3, 1>(0, 10) = R1 * inform.block<3, 1>(0, 4);


		}
		static void solve_hinge_inequality(
			const vec3& x0, const quat& q0,
			const double& mass0,
			const mat3& inertiaInverse0,
			const vec3& x1, const quat& q1,
			const double& mass1,
			const mat3& inertiaInverse1,
			inform_t& inform,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
			const auto& x_joint_b0 = inform.block<3, 1>(0, 2);
			const auto& x_joint_b1 = inform.block<3, 1>(0, 3);
			const auto& axis_s0 = inform.block<3, 1>(0, 7);
			const auto& axis_t0 = inform.block<3, 1>(0, 8);
			const auto& axis1 = inform.block<3, 1>(0, 9);
			const auto& axis_s1 = inform.block<3, 1>(0, 10);

			vec3 r_joint0 = x_joint_b0 - x0;
			vec3 r_joint1 = x_joint_b1 - x1;

			double C5 = axis_s0.dot(axis_s1) - std::cos(inform(3, 0));


			if (C5 > 0.0) {

				Eigen::Matrix<double, 5, 1> C;
				C.block<3, 1>(0, 0) = x_joint_b0 - x_joint_b1;
				C(3, 0) = axis_s0.dot(axis1);
				C(4, 0) = axis_t0.dot(axis1);

				mat3 K1 = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);
				mat3 K2 = calcK(x_joint_b1, 1.0 / mass1, x1, inertiaInverse1);

				Eigen::Matrix<double, 3, 2> u;
				u.col(0) = axis_s0.cross(axis1);
				u.col(1) = axis_t0.cross(axis1);

				Eigen::Matrix<double, 5, 5> K = Eigen::Matrix<double, 5, 5>::Zero();
				K.block<3, 3>(0, 0) = K1 + K2;
				K.block<3, 2>(0, 3) = -(
					skew(r_joint0) * inertiaInverse0 * u +
					skew(r_joint1) * inertiaInverse1 * u);
				K.block<2, 3>(3, 0) = K.block<3, 2>(0, 3).transpose();
				K.block<2, 2>(3, 3) = u.transpose() *
					(inertiaInverse0 + inertiaInverse1) * u;

				Eigen::Matrix<double, 5, 1> dlambda = K.llt().solve(-C);
				vec3 dlambda_X = dlambda.block<3, 1>(0, 0);
				vec3 u_dlambda_q = u * dlambda.block<2, 1>(3, 0);

				dx0 = dlambda_X / mass0;
				vec3 r0 = x_joint_b0 - x0;
				vec3 dr0 = inertiaInverse0 * (r0.cross(dlambda_X) + u_dlambda_q);
				dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
				dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();


				dx1 = -dlambda_X / mass1;
				vec3 r1 = x_joint_b1 - x1;
				vec3 dr1 = -inertiaInverse1 * (r1.cross(dlambda_X) + u_dlambda_q);
				dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
				dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();

			}
			else {

				Eigen::Matrix<double, 6, 1> C;
				C.block<3, 1>(0, 0) = x_joint_b0 - x_joint_b1;
				C(3, 0) = axis_s0.dot(axis1);
				C(4, 0) = axis_t0.dot(axis1);
				C(5, 0) = C5;

				mat3 K1 = calcK(x_joint_b0, 1.0 / mass0, x0, inertiaInverse0);
				mat3 K2 = calcK(x_joint_b1, 1.0 / mass1, x1, inertiaInverse1);

				Eigen::Matrix<double, 3, 3> u;
				u.col(0) = axis_s0.cross(axis1);
				u.col(1) = axis_t0.cross(axis1);
				u.col(2) = axis_s0.cross(axis_s1);

				Eigen::Matrix<double, 6, 6> K = Eigen::Matrix<double, 6, 6>::Zero();
				K.block<3, 3>(0, 0) = K1 + K2;
				K.block<3, 3>(0, 3) = -(
					skew(r_joint0) * inertiaInverse0 * u +
					skew(r_joint1) * inertiaInverse1 * u);
				K.block<3, 3>(3, 0) = K.block<3, 3>(0, 3).transpose();
				K.block<3, 3>(3, 3) = u.transpose() *
					(inertiaInverse0 + inertiaInverse1) * u;

				Eigen::Matrix<double, 6, 1> dlambda = K.llt().solve(-C);
				vec3 dlambda_X = dlambda.block<3, 1>(0, 0);
				vec3 u_dlambda_q = u * dlambda.block<3, 1>(3, 0);

				dx0 = dlambda_X / mass0;
				vec3 r0 = x_joint_b0 - x0;
				vec3 dr0 = inertiaInverse0 * (r0.cross(dlambda_X) + u_dlambda_q);
				dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
				dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();


				dx1 = -dlambda_X / mass1;
				vec3 r1 = x_joint_b1 - x1;
				vec3 dr1 = -inertiaInverse1 * (r1.cross(dlambda_X) + u_dlambda_q);
				dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
				dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();


			}

		}
		//===========================================




		////===========================================
		//// target angle motor hinge joint
		//void init_targetAngleMotorHinge_joint(RigidBody* rigid0, RigidBody* rigid1,
		//	vec3 v0, vec3 v1,
		//	vec3 omega0, vec3 omega1,
		//	vec3 cp0, vec3 cp1,
		//	vec3 n,
		//	double coeffRestitution,
		//	inform_t& inform
		//) {

		//}
		//void update_targetAngleMotorHinge_joint(RigidBody* rigid0, RigidBody* rigid1,
		//	inform_t& inform
		//) {

		//}
		//void solve_targetAngleMotorHinge_joint(RigidBody* rigid0, RigidBody* rigid1,
		//	inform_t& inform,
		//	vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		//) {

		//}
		////===========================================




		////===========================================
		//// rigid body contact
		//void init_rigidBodyContact(RigidBody* rigid0, RigidBody* rigid1,
		//	vec3 cp0, vec3 cp1,
		//	vec3 n,
		//	double coeffRestitution,
		//	inform_t& inform
		//) {
		//	auto& x0 = rigid0->xc;
		//	auto& x1 = rigid1->xc;

		//	n.normalize();

		//	vec3 r0 = cp0 - x0;
		//	vec3 r1 = cp1 - x1;
		//	vec3 u0 = rigid0->vc + rigid0->omega.cross(r0);
		//	vec3 u1 = rigid1->vc + rigid1->omega.cross(r1);
		//	vec3 u_rel = u0 - u1;
		//	double u_rel_n = n.dot(u_rel);

		//	vec3 t = u_rel - u_rel_n * n;
		//	t.normalize();

		//	mat3 K1 = calcK(cp0, 1.0 / rigid0->mass, rigid0->xc, rigid0->inertiaInverse);
		//	mat3 K2 = calcK(cp1, 1.0 / rigid1->mass, rigid1->xc, rigid1->inertiaInverse);
		//	mat3 K = K1 + K2;

		//	// 0: contact point in body 0 (global)
		//	// 1: contact point in body 1 (global)
		//	// 2: contact normal in body 1 (global)
		//	// 3: contact tangent in body 1 (global)
		//	// 0,4: 1.0 / (n^T * K * n)
		//	// 1,4: maximal impulse in tangent direction
		//	// 2,4: goal velocity in normal direction after collision
		//	// 3,4: sum impulse
		//	inform.block<3, 1>(0, 0) = cp0;
		//	inform.block<3, 1>(0, 1) = cp1;
		//	inform.block<3, 1>(0, 2) = n;
		//	inform.block<3, 1>(0, 3) = t;
		//	inform(0, 4) = 1.0 / (n.dot(K * n));
		//	inform(1, 4) = 1.0 / (t.dot(K * t)) * u_rel.dot(t);
		//	inform(2, 4) = 0.0;
		//	if (u_rel_n < 0.0) inform(2, 4) = -coeffRestitution * u_rel_n;
		//	inform(3, 4) = 0.0;

		//}
		//void velocitySolve_rigidBodyContact(RigidBody* rigid0, RigidBody* rigid1,
		//	inform_t& inform,
		//	double coeffStiffness,
		//	double coeffFriction,
		//	vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		//) {
		//	auto& q0 = rigid0->orient;
		//	auto& q1 = rigid1->orient;

		//	const vec3& cp0 = inform.block<3, 1>(0, 0);
		//	const vec3& cp1 = inform.block<3, 1>(0, 1);
		//	const vec3& n = inform.block<3, 1>(0, 2);
		//	const vec3& t = inform.block<3, 1>(0, 3);

		//	double nKn_inv = inform(0, 4);
		//	double d = n.dot(cp0 - cp1);
		//	double j_max = inform(1, 4);
		//	double u_rel_n_goal = inform(2, 4);
		//	double& sum_j = inform(3, 4);

		//	vec3 r0 = cp0 - rigid0->xc;
		//	vec3 r1 = cp1 - rigid1->xc;

		//	vec3 u0 = rigid0->vc + rigid0->omega.cross(r0);
		//	vec3 u1 = rigid1->vc + rigid1->omega.cross(r1);

		//	vec3 u_rel = u0 - u1;
		//	double u_rel_n = u_rel.dot(n);
		//	double du_rel_n = u_rel_n_goal - u_rel_n;
		//	double j_corr_mag = nKn_inv * du_rel_n;

		//	if (j_corr_mag < -sum_j) j_corr_mag = -sum_j;
		//	if (d < 0.0) j_corr_mag -= coeffStiffness * nKn_inv * d;

		//	vec3 j(j_corr_mag * n);
		//	sum_j += j_corr_mag;

		//	double jn = j.dot(n);
		//	if (coeffFriction * jn > j_max) {
		//		j -= j_max * t;
		//	}
		//	else if (coeffFriction * jn < -j_max) {
		//		j += j_max * t;
		//	}
		//	else {
		//		j -= coeffFriction * jn * t;
		//	}

		//	dx0 = j / rigid0->mass;
		//	vec3 dr0 = rigid0->inertiaInverse * (r0.cross(j));
		//	dq0 = quat(0.0, dr0[0], dr0[1], dr0[2]);
		//	dq0.coeffs() = 0.5 * (dq0 * q0).coeffs();

		//	dx1 = -j / rigid1->mass;
		//	vec3 dr1 = -rigid1->inertiaInverse * (r1.cross(j));
		//	dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
		//	dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();

		//}
		////===========================================


		////===========================================
		//// particle contact
		//void init_particleContact(RigidBody* rigid0, RigidBody* rigid1,
		//	vec3 v0, double mass0,
		//	vec3 cp0, vec3 cp1,
		//	vec3 n,
		//	double coeffRestitution,
		//	inform_t& inform
		//) {
		//	auto& x1 = rigid1->xc;

		//	n.normalize();

		//	vec3 r1 = cp1 - x1;
		//	vec3 u1 = rigid1->vc + rigid1->omega.cross(r1);
		//	vec3 u_rel = v0 - u1;
		//	double u_rel_n = n.dot(u_rel);

		//	vec3 t = u_rel - u_rel_n * n;
		//	t.normalize();

		//	mat3 K1 = (1.0 / mass0) * mat3::Identity();
		//	mat3 K2 = calcK(cp1, 1.0 / rigid1->mass, rigid1->xc, rigid1->inertiaInverse);
		//	mat3 K = K1 + K2;

		//	// 0: contact point in body 0 (global)
		//	// 1: contact point in body 1 (global)
		//	// 2: contact normal in body 1 (global)
		//	// 3: contact tangent in body 1 (global)
		//	// 0,4: 1.0 / (n^T * K * n)
		//	// 1,4: maximal impulse in tangent direction
		//	// 2,4: goal velocity in normal direction after collision
		//	// 3,4: sum impulse
		//	inform.block<3, 1>(0, 0) = cp0;
		//	inform.block<3, 1>(0, 1) = cp1;
		//	inform.block<3, 1>(0, 2) = n;
		//	inform.block<3, 1>(0, 3) = t;
		//	inform(0, 4) = 1.0 / (n.dot(K * n));
		//	inform(1, 4) = 1.0 / (t.dot(K * t)) * u_rel.dot(t);
		//	inform(2, 4) = 0.0;
		//	if (u_rel_n < 0.0) inform(2, 4) = -coeffRestitution * u_rel_n;
		//	inform(3, 4) = 0.0;

		//}
		//void velocitySolve_particleContact(RigidBody* rigid0, RigidBody* rigid1,
		//	inform_t& inform,
		//	vec3 v0, double mass0,
		//	double coeffStiffness,
		//	double coeffFriction,
		//	vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		//) {
		//	auto& q1 = rigid1->orient;

		//	const vec3& cp0 = inform.block<3, 1>(0, 0);
		//	const vec3& cp1 = inform.block<3, 1>(0, 1);
		//	const vec3& n = inform.block<3, 1>(0, 2);
		//	const vec3& t = inform.block<3, 1>(0, 3);

		//	double nKn_inv = inform(0, 4);
		//	double d = n.dot(cp0 - cp1);
		//	double j_max = inform(1, 4);
		//	double u_rel_n_goal = inform(2, 4);
		//	double& sum_j = inform(3, 4);

		//	vec3 r1 = cp1 - rigid1->xc;

		//	vec3 u1 = rigid1->vc + rigid1->omega.cross(r1);

		//	vec3 u_rel = v0 - u1;
		//	double u_rel_n = u_rel.dot(n);
		//	double du_rel_n = u_rel_n_goal - u_rel_n;
		//	double j_corr_mag = nKn_inv * du_rel_n;

		//	if (j_corr_mag < -sum_j) j_corr_mag = -sum_j;
		//	if (d < 0.0) j_corr_mag -= coeffStiffness * nKn_inv * d;

		//	vec3 j(j_corr_mag * n);
		//	sum_j += j_corr_mag;

		//	double jn = j.dot(n);
		//	if (coeffFriction * jn > j_max) {
		//		j -= j_max * t;
		//	}
		//	else if (coeffFriction * jn < -j_max) {
		//		j += j_max * t;
		//	}
		//	else {
		//		j -= coeffFriction * jn * t;
		//	}

		//	dx0 = j / mass0;

		//	dx1 = -j / rigid1->mass;
		//	vec3 dr1 = -rigid1->inertiaInverse * (r1.cross(j));
		//	dq1 = quat(0.0, dr1[0], dr1[1], dr1[2]);
		//	dq1.coeffs() = 0.5 * (dq1 * q1).coeffs();

		//}
		////===========================================


	private:

		static mat3 calcK(
			const vec3& x_b, const double& inv_m,
			const vec3& xc, const mat3& inertiaInverse_w
		) {
			mat3 K;

			if (inv_m > 0.0) {

				vec3 r = x_b - xc;
				double a = r[0];
				double b = r[1];
				double c = r[2];

				double j11 = inertiaInverse_w(0, 0);
				double j12 = inertiaInverse_w(0, 1);
				double j13 = inertiaInverse_w(0, 2);
				double j22 = inertiaInverse_w(1, 1);
				double j23 = inertiaInverse_w(1, 2);
				double j33 = inertiaInverse_w(2, 2);

				K(0, 0) = c * c * j22 - b * c * (j23 + j23) + b * b * j33 + inv_m;
				K(0, 1) = -(c * c * j12) + a * c * j23 + b * c * j13 - a * b * j33;
				K(0, 2) = b * c * j12 - a * c * j22 - b * b * j13 + a * b * j23;
				K(1, 0) = K(0, 1);
				K(1, 1) = c * c * j11 - a * c * (j13 + j13) + a * a * j33 + inv_m;
				K(1, 2) = -(b * c * j11) + a * c * j12 + a * b * j13 - a * a * j23;
				K(2, 0) = K(0, 2);
				K(2, 1) = K(1, 2);
				K(2, 2) = b * b * j11 - a * b * (j12 + j12) + a * a * j22 + inv_m;
			}
			else {
				K.setZero();
			}

			return K;
		}

		static mat3 orthogonal_basis_matrix(
			const vec3& axis, vec3 temp_basis = vec3(1.0, 0.0, 0.0)
		) {
			mat3 res;
			vec3 basis0 = axis.normalized();
			if (abs(basis0.dot(temp_basis)) > 0.99) {
				std::rotate(
					temp_basis.data(),
					temp_basis.data() + temp_basis.size() - 1,
					temp_basis.data() + temp_basis.size());
			}
			vec3 basis1 = basis0.cross(temp_basis).normalized();
			vec3 basis2 = basis0.cross(basis1).normalized();

			res.col(0) = basis0;
			res.col(1) = basis1;
			res.col(2) = basis2;

			return res;

		}

		static mat4 quat_to_left_matrix(const quat& q) {
			mat4 res;

			res <<
				q.w(), -q.x(), -q.y(), -q.z(),
				q.x(), q.w(), -q.z(), q.y(),
				q.y(), q.z(), q.w(), -q.x(),
				q.z(), -q.y(), q.x(), q.w();

			return res;
		}

		static mat4 quat_to_right_matrix(const quat& q) {
			mat4 res;

			res <<
				q.w(), -q.x(), -q.y(), -q.z(),
				q.x(), q.w(), q.z(), -q.y(),
				q.y(), -q.z(), q.w(), q.x(),
				q.z(), q.y(), -q.x(), q.w();

			return res;
		}

		static Eigen::Matrix<double, 4, 3> quat_to_dot_matrix(const quat& q) {
			Eigen::Matrix<double, 4, 3> res;

			res <<
				-0.5 * q.x(), -0.5 * q.y(), -0.5 * q.z(),
				0.5 * q.w(), 0.5 * q.z(), -0.5 * q.y(),
				-0.5 * q.z(), 0.5 * q.w(), 0.5 * q.x(),
				0.5 * q.y(), -0.5 * q.x(), 0.5 * q.w();

			return res;
		}

		static mat3 skew(const vec3& v) {
			mat3 res;
			res <<
				0.0, -v[2], v[1],
				v[2], 0.0, -v[0],
				-v[1], v[0], 0.0;
			return res;
		}

	};



	class Constraint {
	public:
		using vec3 = Eigen::Vector3d;
		using vec4 = Eigen::Vector4d;
		using mat3 = Eigen::Matrix3d;
		using mat4 = Eigen::Matrix4d;
		using quat = Eigen::Quaterniond;
		using inform_t = Eigen::Matrix<double, 4, 12>;

		virtual ~Constraint() = default;  // 가상 소멸자 추가


		virtual void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) {
		}

		//virtual void update() {

		//}
		//virtual void solve() {

		//}
	};


	class BallJoint : public PBD::Constraint {
	public:
		BallJoint(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const vec3& x_joint
		) {
			PBD::PositionBasedConstraints::init_ball_joint(
				rigid0.xc, rigid0.orient, 
				rigid1.xc, rigid1.orient, 
				x_joint, this->inform
			);
		}

		void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) override {
			PBD::PositionBasedConstraints::update_ball_joint(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				this->inform
			);
			PBD::PositionBasedConstraints::solve_ball_joint(
				rigid0.xc, rigid0.orient, rigid0.mass, rigid0.inertiaInverse,
				rigid1.xc, rigid1.orient, rigid1.mass, rigid1.inertiaInverse,
				this->inform,
				//stiffness, dt,
				//lambda,
				dx0, dq0, dx1, dq1
			);
		}

	private:
		inform_t inform;
	};

	class ParticleBallJoint : public PBD::Constraint {
	public:
		ParticleBallJoint(
			const RigidBody& rigid0,
			const vec3& x_joint
		) {
			PBD::PositionBasedConstraints::init_particleBall_joint(
				rigid0.xc, rigid0.orient, x_joint, this->inform
			);
		}

		void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) override {
			PBD::PositionBasedConstraints::update_particleBall_joint(
				rigid0.xc, rigid0.orient, this->inform
			);
			PBD::PositionBasedConstraints::solve_particleBall_joint(
				rigid0.xc, rigid0.orient, rigid0.mass, rigid0.inertiaInverse,
				this->inform,
				stiffness, dt,
				lambda,
				dx0, dq0, dx1, dq1
			);
		}
	private:
		inform_t inform;
	};

	class UniversalJoint : public PBD::Constraint {
	public:
		UniversalJoint(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const vec3& x_joint,
			const vec3& axis0,
			const vec3& axis1
		) {
			PBD::PositionBasedConstraints::init_universal_joint(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				x_joint, axis0, axis1, this->inform
			);
		}

		void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) override {
			PBD::PositionBasedConstraints::update_universal_joint(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				this->inform
			);
			PBD::PositionBasedConstraints::solve_universal_joint(
				rigid0.xc, rigid0.orient, rigid0.mass, rigid0.inertiaInverse,
				rigid1.xc, rigid1.orient, rigid1.mass, rigid1.inertiaInverse,
				this->inform,
				//stiffness, dt,
				//lambda,
				dx0, dq0, dx1, dq1
			);
		}
	private:
		inform_t inform;
	};

	class HingeJoint : public PBD::Constraint {
	public:
		HingeJoint(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const vec3& x_joint,
			const vec3& axis
		) {
			PBD::PositionBasedConstraints::init_hinge_joint(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				x_joint, axis, this->inform
			);
		}

		void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) override {
			PBD::PositionBasedConstraints::update_hinge_joint(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				this->inform
			);
			PBD::PositionBasedConstraints::solve_hinge_joint(
				rigid0.xc, rigid0.orient, rigid0.mass, rigid0.inertiaInverse,
				rigid1.xc, rigid1.orient, rigid1.mass, rigid1.inertiaInverse,
				this->inform,
				//stiffness, dt,
				//lambda,
				dx0, dq0, dx1, dq1
			);
		}
	private:
		inform_t inform;
	};


	class ParticleHingeJoint : public PBD::Constraint {
	public:
		ParticleHingeJoint(
			const RigidBody& rigid0,
			const vec3& x_joint,
			const vec3& axis
		) {
			PBD::PositionBasedConstraints::init_particleHinge_joint(
				rigid0.xc, rigid0.orient, x_joint, axis, this->inform
			);
		}

		void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) override {
			PBD::PositionBasedConstraints::update_particleHinge_joint(
				rigid0.xc, rigid0.orient, this->inform
			);
			PBD::PositionBasedConstraints::solve_particleHinge_joint(
				rigid0.xc, rigid0.orient, rigid0.mass, rigid0.inertiaInverse,
				this->inform,
				stiffness, dt,
				lambda,
				dx0, dq0, dx1, dq1
			);
		}
	private:
		inform_t inform;
	};


	class CylindricalJoint : public PBD::Constraint {
	public:
		CylindricalJoint(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const vec3& x_joint,
			const vec3& axis
		) {
			PBD::PositionBasedConstraints::init_cylindrical_joint(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				x_joint, axis, this->inform
			);
		}

		void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) override {
			PBD::PositionBasedConstraints::update_cylindrical_joint(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				this->inform
			);
			PBD::PositionBasedConstraints::solve_cylindrical_joint(
				rigid0.xc, rigid0.orient, rigid0.mass, rigid0.inertiaInverse,
				rigid1.xc, rigid1.orient, rigid1.mass, rigid1.inertiaInverse,
				this->inform,
				//stiffness, dt,
				//lambda,
				dx0, dq0, dx1, dq1
			);
		}
	private:
		inform_t inform;
	};



	class PrismaticJoint : public PBD::Constraint {
	public:
		PrismaticJoint(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const vec3& x_joint,
			const vec3& axis
		) {
			PBD::PositionBasedConstraints::init_prismatic_joint(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				x_joint, axis, this->inform
			);
		}

		void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) override {
			PBD::PositionBasedConstraints::update_prismatic_joint(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				this->inform
			);
			PBD::PositionBasedConstraints::solve_prismatic_joint(
				rigid0.xc, rigid0.orient, rigid0.mass, rigid0.inertiaInverse,
				rigid1.xc, rigid1.orient, rigid1.mass, rigid1.inertiaInverse,
				this->inform,
				//stiffness, dt,
				//lambda,
				dx0, dq0, dx1, dq1
			);
		}
	private:
		inform_t inform;
	};



	class PlaneInequality : public PBD::Constraint {
	public:
		PlaneInequality(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const vec3& x_joint,
			const vec3& normal
		) {
			PBD::PositionBasedConstraints::init_plane_inequality(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				x_joint, normal, this->inform
			);
		}

		void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) override {
			PBD::PositionBasedConstraints::update_plane_inequality(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				this->inform
			);
			PBD::PositionBasedConstraints::solve_plane_inequality(
				rigid0.xc, rigid0.orient, rigid0.mass, rigid0.inertiaInverse,
				rigid1.xc, rigid1.orient, rigid1.mass, rigid1.inertiaInverse,
				rigid1.r,
				this->inform,
				stiffness, dt,
				lambda,
				dx0, dq0, dx1, dq1
			);
		}
	private:
		inform_t inform;
	};


	class HingeInequality : public PBD::Constraint {
	public:
		HingeInequality(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const vec3& x_joint,
			const vec3& axis,
			const double& limit_angle
		) {
			PBD::PositionBasedConstraints::init_hinge_inequality(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				x_joint, axis, limit_angle, this->inform
			);
		}

		void update_and_solve(
			const RigidBody& rigid0,
			const RigidBody& rigid1,
			const double& stiffness, const double& dt,
			vec3& lambda,
			vec3& dx0, quat& dq0, vec3& dx1, quat& dq1
		) override {
			PBD::PositionBasedConstraints::update_hinge_inequality(
				rigid0.xc, rigid0.orient,
				rigid1.xc, rigid1.orient,
				this->inform
			);
			PBD::PositionBasedConstraints::solve_hinge_inequality(
				rigid0.xc, rigid0.orient, rigid0.mass, rigid0.inertiaInverse,
				rigid1.xc, rigid1.orient, rigid1.mass, rigid1.inertiaInverse,
				this->inform,
				//stiffness, dt,
				//lambda,
				dx0, dq0, dx1, dq1
			);
		}
	private:
		inform_t inform;
	};

}
