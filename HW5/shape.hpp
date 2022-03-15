//
//  shape.hpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/12/07.
//

#ifndef shape_hpp
#define shape_hpp

#include "vec3.hpp"
#include "BSPTree.hpp"
#include <vector>

using namespace std;

class shape {
public:
    shape() : color(1), emission(false), transparency(0.0), reflection(0.1), refIdx(1) {}
    virtual ~shape() {}
    virtual bool intersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) = 0;
    vec3 center;
    vec3 color;
    bool emission;
    double transparency, reflection;
    double refIdx;
    
    double ka, ks, kd;
    double shininess;
};

class sphere: public shape {
public:
    sphere(
        const vec3& c,
        const double& r,
        const vec3& sc,
        const double& reflection = 1,
        const double& transparency = 0,
        const double& refIdx = 1,
        const double& ka = 0,
        const double& kd = 0,
        const double& ks = 0,
        const double& shininess = 0
    );
    
    virtual bool intersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) override;
    double radius;
};

class polygonMesh: public shape {
public:
    polygonMesh(
        const vec3& c,
        const vector<vector<vec3> >& polygons,
        const vector<vector<vec3> >& normals,
        const vec3& sc,
        const double& reflection = 1,
        const double& transparency = 0,
        const double& refIdx = 1,
        const double& ka = 0,
        const double& kd = 0,
        const double& ks = 0,
        const double& shininess = 0
    );
    
    virtual bool intersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) override;
    vector<vector<vec3> > polygons;
    vector<vector<vec3> > normals;
    BSPTree bspTree;
    
};

#endif /* shape_hpp */
