//
//  Vec3.hpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/11/21.
//

#ifndef vec3_hpp
#define vec3_hpp

class vec3 {
public:
    double x, y, z;
    
    vec3();
    vec3(double x);
    vec3(double x, double y, double z);
    vec3(double vec[]);
    
    vec3 operator+(const vec3& operand) const;
    vec3 operator-(const vec3& operand) const;
    vec3 operator-() const;
    vec3 operator*(const double& operand) const;
    vec3 operator*(const vec3& operand) const;
    vec3 operator/(const double& operand) const;
    
    bool operator==(const vec3& operand) const;
    
    double length() const;
    
    void normalize();
    vec3 normalized() const;
    
    double dotProduct(vec3 a) const;
    vec3 crossProduct(vec3 a) const;
};

#endif /* vec3_hpp */
