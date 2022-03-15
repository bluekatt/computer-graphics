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
    float x, y, z;
    
    vec3();
    vec3(float x, float y, float z);
    vec3(float vec[]);
    
    vec3 operator+(const vec3& operand);
    vec3 operator-(const vec3& operand);
    vec3 operator*(const float& operand);
    vec3 operator/(const float& operand);
    
    float length();
    
    void normalize();
    vec3 normalized();
    
    float dotProduct(vec3 a);
    vec3 crossProduct(vec3 a);
};

#endif /* vec3_hpp */
