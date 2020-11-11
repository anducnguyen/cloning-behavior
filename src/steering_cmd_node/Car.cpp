/**
 * Created by: TRAN Anh Tuan 2019/09/06
 *
*/

#include "Car.h"
#include <iostream>
#include <iomanip>
/* =====================================================================
 * Class Car
 * =====================================================================
 */

// Constructor
Car::Car(double _x, double _y, double _z, double _yaw, double _vx, double _vy, double _vz) : x(_x), y(_y), z(_z), yaw(_yaw), vx(_vx), vy(_vy), vz(_vz) {};

// Getter
double Car::getX() const {
    return x;
}
double Car::getY() const {
    return y;
}
double Car::getZ() const {
    return z;
}
double Car::getYaw() const {
    return yaw;
}

double Car::getVx() const {
    return vx;
}
double Car::getVy() const {
    return vy;
}
double Car::getVz() const {
    return vz;
}
//Setter
void Car::setX(double _x) {
    this->x = _x;
}
void Car::setY(double _y) {
    this->y = _y;
}
void Car::setZ(double _z) {
    this->z = _z;
}
void Car::setYaw(double _yaw) {
    this->yaw = _yaw;
}

void Car::setVx(double _vx) {
    this->vx = _vx;
}
void Car::setVy(double _vy) {
    this->vy = _vy;
}
void Car::setVz(double _vz) {
    this->vz = _vz;
}
// Print
void Car::print() const {
    std::cout << "Car @ (x,y,z,yaw,vx,vy,vz) = ("<<std::fixed<<std::setprecision(2)<<x<<","<<y<<","<<z<<","<<yaw<<", "<<vx<<","<<vy<<","<<vz<<")"<<std::endl;
}
