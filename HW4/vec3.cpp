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

vec3::vec3(float x, float y, float z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

vec3::vec3(float vec[]) {
    this->x = vec[0];
    this->y = vec[1];
    this->z = vec[2];
}

vec3 vec3::operator+(const vec3& operand) {
    vec3 result;
    result.x = x + operand.x;
    result.y = y + operand.y;
    result.z = z + operand.z;

    return result;
}

vec3 vec3::operator-(const vec3& operand) {
    vec3 result;
    result.x = x - operand.x;
    result.y = y - operand.y;
    result.z = z - operand.z;

    return result;
}

vec3 vec3::operator*(const float& operand) {
    vec3 result;
    result.x = x * operand;
    result.y = y * operand;
    result.z = z * operand;

    return result;
}
vec3 vec3::operator/(const float& operand) {
    vec3 result;
    result.x = x / operand;
    result.y = y / operand;
    result.z = z / operand;

    return result;
}

float vec3::length() {
    return sqrt((x * x) + (y * y) + (z * z));
}

void vec3::normalize() {
    float length = this->length();
    if (length == 0) return;
    x /= length; y /= length; z /= length;
}

vec3 vec3::normalized() {
    float length = this->length();
    if (length == 0) {
        return vec3();
    }
    return vec3(x/length, y/length, z/length);
}

float vec3::dotProduct(vec3 a) {
    float result = (this->x * a.x) + (this->y * a.y) + (this->z * a.z);
    return result;
}

vec3 vec3::crossProduct(vec3 a) {
    vec3 result = vec3((this->y * a.z) - (this->z * a.y), (this->z * a.x) - (this->x * a.z), (this->x * a.y) - (this->y * a.x));
    return result;
}
