//
//  shape.cpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/12/07.
//

#include "shape.hpp"
#include <cmath>
#include <iostream>

sphere::sphere(const vec3& c,
               const double& r,
               const vec3& sc,
               const double& reflection,
               const double& transparency,
               const double& refIdx,
               const double& ka,
               const double& kd,
               const double& ks,
               const double& shininess
               ) {
    this->center = c;
    this->radius = r;
    this->color = sc;
    this->reflection = reflection;
    this->transparency = transparency;
    this->refIdx = refIdx;
    this->ka = ka;
    this->kd = kd;
    this->ks = ks;
    this->shininess = shininess;
}

bool sphere::intersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) {
    vec3 deltaP = center - rOrig;
    double b = deltaP.dotProduct(rDir);
    if(b < 0) return false;
    double ac = deltaP.dotProduct(deltaP) - radius * radius;
    double d = b * b - ac;
    if(d < 0) return false;
    
    s = b - sqrt(d);
    if(s<0) s = b + sqrt(d);
    
    normal = rOrig + rDir * s - center;
    normal.normalize();
    
    return true;
}

polygonMesh::polygonMesh(const vec3& c,
                         const vector<vector<vec3> >& polygons,
                         const vector<vector<vec3> >& normals,
                         const vec3& sc,
                         const double& reflection,
                         const double& transparency,
                         const double& refIdx,
                         const double& ka,
                         const double& kd,
                         const double& ks,
                         const double& shininess
                         ) {
    this->center = c;
    this->polygons = polygons;
    this->normals = normals;
    this->color = sc;
    this->reflection = reflection;
    this->transparency = transparency;
    this->refIdx = refIdx;
    this->ka = ka;
    this->kd = kd;
    this->ks = ks;
    this->shininess = shininess;
    
    for(int i=0;i<this->polygons.size();i++) {
        for(int j=0;j<this->polygons[i].size();j++) {
            this->polygons[i][j] = this->polygons[i][j] + center;
        }
    }
    
    bspTree.construct(this->polygons);
}

bool polygonMesh::intersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) {
    return bspTree.findIntersect(rOrig, rDir, normal, s);
}
