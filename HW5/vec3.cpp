//
//  Vec3.cpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/11/21.
//

#include "vec3.hpp"
#include <cmath>

vec3::vec3() {
    x = 0;
    y = 0;
    z = 0;
}

vec3::vec3(double x) {
    this->x = x;
    this->y = x;
    this->z = x;
}

vec3::vec3(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

vec3::vec3(double vec[]) {
    this->x = vec[0];
    this->y = vec[1];
    this->z = vec[2];
}

vec3 vec3::operator+(const vec3& operand) const {
    vec3 result;
    result.x = x + operand.x;
    result.y = y + operand.y;
    result.z = z + operand.z;

    return result;
}

vec3 vec3::operator-(const vec3& operand) const {
    vec3 result;
    result.x = x - operand.x;
    result.y = y - operand.y;
    result.z = z - operand.z;

    return result;
}

vec3 vec3::operator-() const {
    return vec3(-x, -y, -z);
}

vec3 vec3::operator*(const double& operand) const {
    vec3 result;
    result.x = x * operand;
    result.y = y * operand;
    result.z = z * operand;

    return result;
}

vec3 vec3::operator*(const vec3& operand) const {
    vec3 result;
    result.x = x * operand.x;
    result.y = y * operand.y;
    result.z = z * operand.z;

    return result;
}

vec3 vec3::operator/(const double& operand) const {
    vec3 result;
    result.x = x / operand;
    result.y = y / operand;
    result.z = z / operand;

    return result;
}

bool vec3::operator==(const vec3& operand) const {
    return operand.x == x && operand.y == y && operand.z == z;
}

double vec3::length() const {
    return sqrt((x * x) + (y * y) + (z * z));
}

void vec3::normalize() {
    double length = this->length();
    if (length == 0) return;
    x /= length; y /= length; z /= length;
}

vec3 vec3::normalized() const {
    double length = this->length();
    if (length == 0) {
        return vec3();
    }
    return vec3(x/length, y/length, z/length);
}

double vec3::dotProduct(vec3 a) const {
    double result = (this->x * a.x) + (this->y * a.y) + (this->z * a.z);
    return result;
}

vec3 vec3::crossProduct(vec3 a) const {
    vec3 result = vec3((this->y * a.z) - (this->z * a.y), (this->z * a.x) - (this->x * a.z), (this->x * a.y) - (this->y * a.x));
    return result;
}
