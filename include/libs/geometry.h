#ifndef _BLUEJAY_GEOMETRY_H_
#define _BLUEJAY_GEOMETRY_H_

#include <cmath>
#include <limits>

#include <cstdlib>
#include <cstring>

#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

/* 
 * Provides geometry interfaces (needs Eigen3)
 * 
 * WARNING: will be phased out (should be discussed)
 */

#define M_PI 3.14159265358979323846
#define M_EPS 1e-7

/*
 * Point/Vector in a frame
 * x - forward/back (0)
 * y - left/right   (1)
 * z - up/down      (2)
 */
typedef Eigen::Vector3d Vector;
typedef Eigen::Vector3d Point;

/*
 * RotationVector in a frame
 * r - roll         (0)
 * p - pitch        (1)
 * y - yaw          (2)
 */
class RotationVector: public Vector{
public:
    RotationVector() {}
    RotationVector(const Vector vector): Vector(vector) {}
    /* convenience methods just like x - y - z for normal vectors */
    double &r(){
        return (*this)[0];
    }
    double &p(){
        return (*this)[1];
    }
    double &y(){
        return (*this)[2];
    }
};

/* 
 * Position orientation in a frame 
 */
class Pose{
public:
    Vector position;
    RotationVector orientation;
};

/* 
 * Velocitity lineair and angular in a frame
 */
class Twist{
    Vector lineair;
    RotationVector angular;
};

/* Rotation matrix */
class RotationMatrix{
    friend RotationMatrix operator*(const RotationMatrix &rm1, const RotationMatrix &rm2);
public:
    RotationMatrix() {}
    RotationMatrix(Eigen::Affine3d matrix): _matrix(matrix) {}
    RotationMatrix(double angle, char type) {
        switch (type) {
            case 'x':
                _matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX());
                break;
            case 'y':
                _matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
                break;
            case 'z':
                _matrix = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ());
                break;
            default:
                break;
        }
    }
    
    Vector operator*(Vector vec){
        return _matrix*vec;
    }
    RotationMatrix transpose() {
        return RotationMatrix(_matrix.inverse());
    }
    RotationMatrix operator*(const RotationMatrix &rm2) {
        RotationMatrix rot;
        rot._matrix = _matrix*rm2._matrix;
        return rot;
    }
    
    static RotationMatrix getTaitBryan(Vector vec){
        Eigen::Affine3d rot;
        rot = Eigen::AngleAxisd(vec[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(vec[1], Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(vec[0], Eigen::Vector3d::UnitX());
        return RotationMatrix(rot);
    }
private:
    Eigen::Affine3d _matrix;
};



#endif
