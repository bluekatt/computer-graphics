//
//  BSPTree.cpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/11/21.
//

#include "BSPTree.hpp"
#include <iostream>

#define EPSILON 0.001

BSPNode::BSPNode(vector<vec3> polygon) {
    this->polygon = polygon;
    if(polygon.size()<2) return;
    vec3 normal = (polygon[1]-polygon[2]).crossProduct((polygon[0]-polygon[1]));
    normal.normalize();
    this->normal = normal;
    this->point = polygon[0];
}

int BSPNode::compare(vector<vec3> poly) {
    bool front = false;
    bool intersect = false;
    
    vec3 vertex = poly[0];
    
    front = (vertex-point).dotProduct(normal) > EPSILON;
    for(auto it=poly.begin()+1;it!=poly.end();it++) {
        vertex = *it;
        bool vFront = (vertex-point).dotProduct(normal) > 0 && abs((vertex-point).dotProduct(normal)) > EPSILON;
        if(vFront != front) {
            intersect = true;
            break;
        }
    }
    if (intersect) return 0;
    if (front) {
        return 1;
    } else {
        return -1;
    }
}

void BSPNode::dividePolygon(const vector<vec3>& poly, vector<vector<vec3> >& polyFront, vector<vector<vec3> >& polyBehind) {
    vector<vector<vec3> > polygonStack;
    vector<vec3> tPoly = poly;
    
    vector<vec3> init;
    
    bool front = false;
    
    tPoly.push_back(poly[0]);
    vec3 vertex = poly[0];
    float d = point.dotProduct(normal);
    
    front = (vertex-point).dotProduct(normal) > EPSILON;
    init.push_back(vertex);
    
    polygonStack.push_back(init);
    
    int switchCnt = 0;
    for(int i=1;i<tPoly.size();i++) {
        vertex = tPoly[i];
        bool vFront = (vertex-point).dotProduct(normal) > EPSILON;
        if(vFront != front) {
            switchCnt++;
            vec3 prevVertex = tPoly[i-1];
            
            vec3 lineDirection = vertex-prevVertex;
            
            float x = (d - normal.dotProduct(prevVertex)) / normal.dotProduct(lineDirection);
            
            vec3 intersection = prevVertex + lineDirection * x;
            
            polygonStack.back().push_back(intersection);
            
            if(switchCnt > 0 && switchCnt%2==0) {
                if(front) {
                    polyFront.push_back(polygonStack.back());
                } else {
                    polyBehind.push_back(polygonStack.back());
                }
                
                polygonStack.pop_back();
                polygonStack.back().push_back(intersection);
                if(i<tPoly.size()-1) polygonStack.back().push_back(vertex);
            } else {
                vector<vec3> newPoly;
                newPoly.push_back(intersection);
                if(i<tPoly.size()-1) newPoly.push_back(vertex);
                
                polygonStack.push_back(newPoly);
            }
            
            front = vFront;
            continue;
        }
        if(i<tPoly.size()-1) polygonStack.back().push_back(vertex);
    }
    if(front) {
        polyFront.push_back(polygonStack[0]);
    } else {
        polyBehind.push_back(polygonStack[0]);
    }
}

BSPNode* BSPNode::construct(vector<vector<vec3> > polygons) {
    if (polygons.empty()) return NULL;
    
    BSPNode* root = new BSPNode(polygons[0]);
    vector<vector<vec3> > polyBehind;
    vector<vector<vec3> > polyFront;
    for(int i=1;i<polygons.size();i++) {
        vector<vec3> poly = polygons[i];
        int compare = root->compare(poly);
        if(compare>0){
            polyFront.push_back(poly);
        } else if (compare<0) {
            polyBehind.push_back(poly);
        } else {
            root->dividePolygon(poly, polyFront, polyBehind);
        }
    }
    
    BSPNode* left = construct(polyBehind);
    BSPNode* right = construct(polyFront);
    root->left = left; root->right = right;
    
    return root;
}

void BSPNode::traverse(vec3 viewPoint, vector<vector<float> >& order) {
    bool front = (viewPoint-point).dotProduct(normal) > EPSILON;
    if(front) {
        if(left) left->traverse(viewPoint, order);
        vector<float> poly;
        for(vec3 v: polygon) {
            poly.push_back(v.x);
            poly.push_back(v.y);
            poly.push_back(v.z);
        }
        order.push_back(poly);
        if(right) right->traverse(viewPoint, order);
    } else {
        if(right) right->traverse(viewPoint, order);
        vector<float> poly;
        for(vec3 v: polygon) {
            poly.push_back(v.x);
            poly.push_back(v.y);
            poly.push_back(v.z);
        }
        order.push_back(poly);
        if(left) left->traverse(viewPoint, order);
    }
}

struct PolygonCompare {
    bool operator()(vector<vec3>& a, vector<vec3>& b) {
        float sizeA = (a[0]-a[1]).crossProduct((a[1]-a[2])).length();
        float sizeB = (b[0]-b[1]).crossProduct((b[1]-b[2])).length();
        return sizeA > sizeB;
    }
};

void BSPTree::construct(vector<vector<vec3> > polygons) {
    sort(polygons.begin(), polygons.end(), PolygonCompare());
    root = BSPNode::construct(polygons);
}

void BSPTree::traverse(vec3 viewPoint, vector<vector<float> >& order) {
    if(root) root->traverse(viewPoint, order);
}
