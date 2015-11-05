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

Point RotationMatrix::rotatePoint(Point point) {
	Point out;
	out.x = elem[0][0] * point.x + elem[0][1] * point.y + elem[0][2] * point.z;
	out.y = elem[1][0] * point.x + elem[1][1] * point.y + elem[1][2] * point.z;
	out.z = elem[2][0] * point.x + elem[2][1] * point.y + elem[2][2] * point.z;
	
	return out;
}

Velocity RotationMatrix::rotateVelocity(Velocity vel) {
	Velocity out;
	out.vx = elem[0][0] * vel.vx + elem[0][1] * vel.vy + elem[0][2] * vel.vz;
	out.vy = elem[1][0] * vel.vx + elem[1][1] * vel.vy + elem[1][2] * vel.vz;
	out.vz = elem[2][0] * vel.vx + elem[2][1] * vel.vy + elem[2][2] * vel.vz;

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