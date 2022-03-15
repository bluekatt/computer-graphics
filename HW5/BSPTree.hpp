//
//  BSPTree.hpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/11/21.
//

#ifndef BSPTree_hpp
#define BSPTree_hpp

#include <cstdio>
#include <vector>
#include "vec3.hpp"

using namespace std;

class BSPNode {
public:
    BSPNode(const vector<vec3>& polygon);
    
    int compare(const vector<vec3>& poly);
    void dividePolygon(const vector<vec3>& poly, vector<vector<vec3> >& polyFront, vector<vector<vec3> >& polyBehind);
    void traverse(const vec3& rOrig, const vec3& rDir, vector<vector<vec3> >& order);
    bool findIntersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s);
    
    static BSPNode* construct(const vector<vector<vec3> >& polygons);
    
    BSPNode* left;
    BSPNode* right;
private:
    // Plane
    vec3 normal;
    vec3 point;
    double d;
    
    vector<vec3> polygon;
    
    bool intersectPolygon(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) const;
};

class BSPTree{
public:
    void construct(vector<vector<vec3> > polygons);
    void traverse(const vec3& rOrig, const vec3& rDir, vector<vector<vec3> >& order);
    bool findIntersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s);
    
private:
    BSPNode* root;
};

#endif /* BSPTree_hpp */

