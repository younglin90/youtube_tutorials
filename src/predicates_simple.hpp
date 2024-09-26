#pragma once

#include <Eigen/Core>

class PredicatesSimple {
public:
	int orient2d(
		const Eigen::Vector2d& a,
		const Eigen::Vector2d& b,
		const Eigen::Vector2d& c
	) {
		Eigen::Matrix3d mat;
		mat.block<1, 2>(0, 0) = a.transpose();
		mat.block<1, 2>(1, 0) = b.transpose();
		mat.block<1, 2>(2, 0) = c.transpose();
		mat.col(2) = Eigen::Vector3d(1.0, 1.0, 1.0);
		double det = mat.determinant();
		if (det > 0.0) return -1; // left of line AB
		else if (det < 0.0) return 1; // right of line AB
		return 0; // 3 points are collinear
	}
	int orient3d(
		const Eigen::Vector3d& a,
		const Eigen::Vector3d& b,
		const Eigen::Vector3d& c,
		const Eigen::Vector3d& d
	) {
		Eigen::Matrix4d mat;
		mat.block<1, 3>(0, 0) = a.transpose();
		mat.block<1, 3>(1, 0) = b.transpose();
		mat.block<1, 3>(2, 0) = c.transpose();
		mat.block<1, 3>(3, 0) = d.transpose();
		mat.col(3) = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
		double det = mat.determinant();
		if (det > 0.0) return -1; // below plane of ABC
		else if (det < 0.0) return 1; // above plane of ABC
		return 0; // 4 points are coplanar
	}
	int incircle(
		const Eigen::Vector2d& a,
		const Eigen::Vector2d& b,
		const Eigen::Vector2d& c,
		const Eigen::Vector2d& d
	) {
		Eigen::Matrix4d mat;
		mat.block<1, 2>(0, 0) = a.transpose();
		mat.block<1, 2>(1, 0) = b.transpose();
		mat.block<1, 2>(2, 0) = c.transpose();
		mat.block<1, 2>(3, 0) = d.transpose();
		mat.col(2) = Eigen::Vector4d(
			a.squaredNorm(),
			b.squaredNorm(),
			c.squaredNorm(),
			d.squaredNorm()
		);
		mat.col(3) = Eigen::Vector4d(1.0, 1.0, 1.0, 1.0);
		double det = mat.determinant();
		if (det > 0.0) return -1; // inside circle
		else if (det < 0.0) return 1; // outside circle
		return 0; // 4 points are coplanar

	}
	int insphere(
		const Eigen::Vector3d& a,
		const Eigen::Vector3d& b,
		const Eigen::Vector3d& c,
		const Eigen::Vector3d& d,
		const Eigen::Vector3d& e
	) {
		Eigen::Matrix<double, 5, 5> mat;
		mat.block<1, 3>(0, 0) = a.transpose();
		mat.block<1, 3>(1, 0) = b.transpose();
		mat.block<1, 3>(2, 0) = c.transpose();
		mat.block<1, 3>(3, 0) = d.transpose();
		mat.block<1, 3>(4, 0) = e.transpose();
		mat.col(3) = Eigen::Vector<double, 5>(
			a.squaredNorm(),
			b.squaredNorm(),
			c.squaredNorm(),
			d.squaredNorm(),
			e.squaredNorm()
		);
		mat.col(4) = Eigen::Vector<double, 5>(1.0, 1.0, 1.0, 1.0, 1.0);
		double det = mat.determinant();
		if (det > 0.0) return -1; // E inside ABCD sphere
		else if (det < 0.0) return 1; // E outside ABCD sphere
		return 0; // 5 points are cospherical

	}

};