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
    BSPNode(vector<vec3> polygon);
    
    int compare(vector<vec3> poly);
    void dividePolygon(const vector<vec3>& poly, vector<vector<vec3> >& polyFront, vector<vector<vec3> >& polyBehind);
    void traverse(vec3 viewPoint, vector<vector<float> >& order);
    
    static BSPNode* construct(vector<vector<vec3> > polygons);
    
    BSPNode* left;
    BSPNode* right;
private:
    // Plane
    vec3 normal;
    vec3 point;
    
    vector<vec3> polygon;
};

class BSPTree{
public:
    void construct(vector<vector<vec3> > polygons);
    void traverse(vec3 viewPoint, vector<vector<float> >& order);
    
private:
    BSPNode* root;
};

#endif /* BSPTree_hpp */

