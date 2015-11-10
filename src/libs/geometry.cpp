#include "libs/geometry.h"

Point::Point(const Vector &p) {
    x = p.x; y = p.y; z = p.z;
}

Vector operator+(const Vector &v1, const Vector &v2){
    Vector vec;
    vec.x = v1.x + v2.x;
    vec.y = v1.y + v2.y;
    vec.z = v1.z + v2.z;
    return vec;
}

Vector operator-(const Vector &v1, const Vector &v2){
    Vector temp_vec = v2;
    temp_vec.scale(-1);
    return v1+temp_vec;
}

Orientation operator-(const Orientation &o){
    Orientation n;
    n.r = -o.r;
    n.p = -o.p;
    n.y = -o.y;
    return n;
}

void RotationMatrix::initRx(double r){
	elem[0][0] = 1;
	elem[0][1] = 0;
	elem[0][2] = 0;
	elem[1][0] = 0;
	elem[1][1] = cos(r);
	elem[1][2] = -sin(r);
	elem[2][0] = 0;
	elem[2][1] = sin(r);
	elem[2][2] = cos(r);
}

void RotationMatrix::initRy(double p) {
	elem[0][0] = cos(p);
	elem[0][1] = 0;
	elem[0][2] = sin(p);
	elem[1][0] = 0;
	elem[1][1] = 1;
	elem[1][2] = 0;
	elem[2][0] = -sin(p);
	elem[2][1] = 0;
	elem[2][2] = cos(p);
}

void RotationMatrix::initRz(double y) {
	elem[0][0] = cos(y);
	elem[0][1] = -sin(y);
	elem[0][2] = 0;
	elem[1][0] = sin(y);
	elem[1][1] = cos(y);
	elem[1][2] = 0;
	elem[2][0] = 0;
	elem[2][1] = 0;
	elem[2][2] = 1;
}

//TODO: use matrix-vector multiplication operator to perform this rotation
Point RotationMatrix::rotatePoint(Point point) {
	Point out;
	out.x = elem[0][0] * point.x + elem[0][1] * point.y + elem[0][2] * point.z;
	out.y = elem[1][0] * point.x + elem[1][1] * point.y + elem[1][2] * point.z;
	out.z = elem[2][0] * point.x + elem[2][1] * point.y + elem[2][2] * point.z;
	
	return out;
}

//TODO: use matrix-vector multiplication operator to perform this rotation
Velocity RotationMatrix::rotateVelocity(Velocity vel) {
	Velocity out;
	out.vx = elem[0][0] * vel.vx + elem[0][1] * vel.vy + elem[0][2] * vel.vz;
	out.vy = elem[1][0] * vel.vx + elem[1][1] * vel.vy + elem[1][2] * vel.vz;
	out.vz = elem[2][0] * vel.vx + elem[2][1] * vel.vy + elem[2][2] * vel.vz;

	return out;
}

RotationMatrix RotationMatrix::transpose() {
	RotationMatrix out;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out.elem[i][j] = elem[j][i];
		}
	}
	return out;
}

RotationMatrix operator*(const RotationMatrix &rm1, const RotationMatrix &rm2) {
	RotationMatrix out;

	for (int row = 0; row < 3; row++) {
		for (int col = 0; col < 3; col++) {
			for (int i = 0; i < 3; i++) {
				out.elem[row][col] += rm1.elem[row][i] * rm2.elem[i][col];
			}
		}
	}

	return out;
}

RotationMatrix operator-(const RotationMatrix &rm) {
	RotationMatrix out;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out.elem[i][j] = -rm.elem[i][j];
		}
	}
	
	return out;
}

//TODO: temporary
RotationMatrix getTaitBryanMatrix(Orientation o){
    double y = o.y, p = o.p, r = o.r;
    RotationMatrix rot;
    rot.elem[0][0] = cos(y)*cos(p);
    rot.elem[0][1] = cos(y)*sin(p)*sin(r)-cos(r)*sin(y);
    rot.elem[0][2] = sin(y)*sin(r)+cos(y)*cos(r)*sin(p);
    rot.elem[1][0] = cos(p)*sin(y);
    rot.elem[1][1] = cos(y)*cos(r)+sin(y)*sin(p)*sin(r);
    rot.elem[1][2] = cos(r)*sin(y)*sin(p)-cos(y)*sin(r);
    rot.elem[2][0] = -sin(p);
    rot.elem[2][1] = cos(p)*sin(r);
    rot.elem[3][2] = cos(p)*cos(r);
    return rot;
}

TransformationMatrix::TransformationMatrix(RotationMatrix R, Vector v) {
	_R = R;
	_v = v;

	constructMatrix(_R, _v);
}

TransformationMatrix::TransformationMatrix(double m[4][4]) {
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			elem[i][j] = m[i][j];
		}
	}
}

void TransformationMatrix::constructMatrix(RotationMatrix R, Vector v) {
	for (int i = 0; i < 3; i++) {
		elem[i][1] = R.elem[i][1];
		elem[i][2] = R.elem[i][2];
		elem[i][3] = R.elem[i][3];
		elem[i][4] = v[i];

		elem[4][i] = 0;
	}
	elem[4][4] = 1;
}

TransformationMatrix TransformationMatrix::inverse() {
	RotationMatrix R_inv = _R.transpose();
	Vector v_temp = (-R_inv*_v);

	constructMatrix(R_inv, v_temp);

	return TransformationMatrix(elem);
}

/* matrix-vector multiplications */
Vector operator*(const RotationMatrix &rm, Vector &v) {
	Vector out;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out[i] += rm.elem[i][j] * v[j];
		}
	}
	
	return out;
}

// very nasty non-mathimatical 3x1 = 4x4 * 3x1 matrix multiplication
Vector operator*(const TransformationMatrix &tm, Vector &v) {
	Vector out;

	double v_hg[4] = { v[1], v[2], v[3], 1 };
	double o_hg[4] = { 0, 0, 0, 0 };

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			o_hg[i] += tm.elem[i][j] * v_hg[j];
		}
	}

	out[1] = o_hg[1];
	out[2] = o_hg[2];
	out[3] = o_hg[3];

	return out;
}

//TODO: use matrix-vector multiplication operator to perform this rotation
Point TransformationMatrix::transformPoint(Point point) {
	Point out;

	double p_hg[4] = { point.x, point.y, point.z, 1 };
	double o_hg[4] = { 0, 0, 0, 0 };

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			o_hg[i] += elem[i][j] * p_hg[j];
		}
	}

	out.x = o_hg[1];
	out.y = o_hg[2];
	out.z = o_hg[3];

	return out;
}

//TODO: use matrix-vector multiplication operator to perform this rotation
Velocity TransformationMatrix::transformVelocity(Velocity vel) {
	Velocity out;

	double v_hg[4] = { vel.vx, vel.vy, vel.vz, 1 };
	double o_hg[4] = { 0, 0, 0, 0 };

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			o_hg[i] += elem[i][j] * v_hg[j];
		}
	}

	out.vx = o_hg[1];
	out.vy = o_hg[2];
	out.vz = o_hg[3];

	return out;
}
