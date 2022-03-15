//
//  BSPTree.cpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/11/21.
//

#include "BSPTree.hpp"
#include <iostream>

#define EPSILON 0.001

BSPNode::BSPNode(const vector<vec3>& polygon) {
    this->polygon = polygon;
    if(polygon.size()<2) return;
    vec3 normal = (polygon[1]-polygon[0]).crossProduct((polygon[2]-polygon[1]));
    normal.normalize();
    this->normal = normal;
    this->point = polygon[0];
    this->d = -point.dotProduct(normal);
}

int BSPNode::compare(const vector<vec3>& poly) {
    bool front = false;
    bool intersect = false;
    
    int chosen = 0;
    
    vec3 vertex = poly[chosen];
    while(abs(vertex.dotProduct(normal) + d) < EPSILON && chosen < poly.size()-1) {
        vertex = poly[++chosen];
    }
    front = vertex.dotProduct(normal) + d > 0;
    for(int i=0;i<=poly.size();i++) {
        if(i==chosen) continue;
        vertex = poly[i];
        bool vFront = vertex.dotProduct(normal) + d > 0;
        if(abs(vertex.dotProduct(normal) + d) > EPSILON && vFront != front) {
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

bool isPolygonSmall(const vector<vec3>& poly) {
    for(int i=0;i<poly.size()-2;i++) {
        if((poly[i]-poly[i+1]).crossProduct(poly[i+1]-poly[i+2]).length() < EPSILON
           || (poly[i]-poly[i+1]).normalized().crossProduct((poly[i+1]-poly[i+2]).normalized()).length() < EPSILON) {
            return true;
        }
    }
    return false;
}

void BSPNode::dividePolygon(const vector<vec3>& poly, vector<vector<vec3> >& polyFront, vector<vector<vec3> >& polyBehind) {
    vector<vector<vec3> > polygonStack;
    vector<vec3> tPoly = poly;
    
    vector<vec3> init;
    
    bool front = false;
    
    int chosen = 0;
    
    vec3 vertex = poly[chosen];
    while(abs(vertex.dotProduct(normal) + d) < EPSILON && chosen < poly.size()-1) {
        vertex = poly[++chosen];
    }
    
    front = vertex.dotProduct(normal) + d > 0 && abs(vertex.dotProduct(normal) + d) > EPSILON;
    init.push_back(vertex);
    
    polygonStack.push_back(init);
    
    int switchCnt = 0;
    for(int i=0;i<tPoly.size();i++) {
        vertex = tPoly[(i+1+chosen)%tPoly.size()];
        bool vFront = vertex.dotProduct(normal) + d > 0 && abs(vertex.dotProduct(normal) + d) > EPSILON;
        if(vFront != front) {
            vec3 prevVertex = tPoly[(i+chosen)%tPoly.size()];
            
            vec3 lineDirection = vertex-prevVertex;
            
            double x = -(d + normal.dotProduct(prevVertex)) / normal.dotProduct(lineDirection);
            
            switchCnt++;
            
            vec3 intersection = prevVertex + lineDirection * x;
            polygonStack.back().push_back(intersection);
            
            if(switchCnt > 0 && switchCnt%2==0) {
                if(!isPolygonSmall(polygonStack.back())) {
                    if(front) {
                        polyFront.push_back(polygonStack.back());
                    } else {
                        polyBehind.push_back(polygonStack.back());
                    }
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
    if(!isPolygonSmall(polygonStack.back())) {
        if(front) {
            polyFront.push_back(polygonStack.back());
        } else {
            polyBehind.push_back(polygonStack.back());
        }
    }
}

BSPNode* BSPNode::construct(const vector<vector<vec3> >& polygons) {
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

void BSPNode::traverse(const vec3& rOrig, const vec3& rDir, vector<vector<vec3> >& order) {
    bool front = rOrig.dotProduct(normal) + d > 0 && abs(rOrig.dotProduct(normal) + d) > EPSILON;
    if(front) {
        if(right) right->traverse(rOrig, rDir, order);
        vector<vec3> poly;
        for(vec3 v: polygon) {
            poly.push_back(vec3(v));
        }
        order.push_back(poly);
        if(left) left->traverse(rOrig, rDir, order);
    } else {
        if(left) left->traverse(rOrig, rDir, order);
        vector<vec3> poly;
        for(vec3 v: polygon) {
            poly.push_back(vec3(v));
        }
        order.push_back(poly);
        if(right) right->traverse(rOrig, rDir, order);
    }
}

bool BSPNode::intersectPolygon(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) const {
    double sCand = -(d+this->normal.dotProduct(rOrig)) / this->normal.dotProduct(rDir);
    if (sCand < 0) return false;
    
    vec3 p = rOrig + rDir * sCand;
    
    vec3 t = (polygon[1]-polygon[0]).crossProduct((p-polygon[0]));
    
    bool inside = true;
    for(int i=1;i<polygon.size();i++) {
        int next = (i+1) % polygon.size();
        
        if(t.dotProduct((polygon[next]-polygon[i]).crossProduct((p-polygon[i]))) < 0) {
            inside = false;
            break;
        }
    }
    
    if(inside) {
        s = sCand;
        normal = this->normal;
        return true;
    } else {
        return false;
    }
}

//bool BSPNode::findIntersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) {
//    bool front = (rOrig-point).dotProduct(this->normal) > EPSILON;
//    bool intersect = false;
//    double sMin = 1e8;
//    vec3 n;
//    if(front) {
//        if(right && right->findIntersect(rOrig, rDir, n, s)) {
//            intersect = true;
//            if (s < sMin) {
//                sMin = s;
//                normal = n;
//            }
//        }
//        if(intersectPolygon(rOrig, rDir, n, s)){
//            intersect = true;
//            if (s < sMin) {
//                sMin = s;
//                normal = n;
//            }
//        }
//        if(left && left->findIntersect(rOrig, rDir, n, s)){
//            intersect = true;
//            if (s < sMin) {
//                sMin = s;
//                normal = n;
//            }
//        }
//    } else {
//        if(left && left->findIntersect(rOrig, rDir, n, s)) {
//            intersect = true;
//            if (s < sMin) {
//                sMin = s;
//                normal = n;
//            }
//        }
//        if(intersectPolygon(rOrig, rDir, n, s)) {
//            intersect = true;
//            if (s < sMin) {
//                sMin = s;
//                normal = n;
//            }
//        }
//        if(right && right->findIntersect(rOrig, rDir, n, s)) {
//            intersect = true;
//            if (s < sMin) {
//                sMin = s;
//                normal = n;
//            }
//        }
//    }
//    s = sMin;
//    return intersect;
//}

bool BSPNode::findIntersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) {
    bool front = rOrig.dotProduct(this->normal) + d > 0;
    if(front) {
        if(right && right->findIntersect(rOrig, rDir, normal, s)) return true;
        if(this->normal.dotProduct(rDir) > 0) return false;
        if(intersectPolygon(rOrig, rDir, normal, s)) return true;
        if(left && left->findIntersect(rOrig, rDir, normal, s)) return true;
    } else {
        if(left && left->findIntersect(rOrig, rDir, normal, s)) return true;
        if(this->normal.dotProduct(rDir) < 0) return false;
        if(intersectPolygon(rOrig, rDir, normal, s)) return true;
        if(right && right->findIntersect(rOrig, rDir, normal, s)) return true;
    }
    return false;
}

struct PolygonCompare {
    bool operator()(vector<vec3>& a, vector<vec3>& b) {
        double sizeA = (a[0]-a[1]).crossProduct((a[1]-a[2])).length();
        double sizeB = (b[0]-b[1]).crossProduct((b[1]-b[2])).length();
        return sizeA > sizeB;
    }
};

void BSPTree::construct(vector<vector<vec3> > polygons) {
    sort(polygons.begin(), polygons.end(), PolygonCompare());
    root = BSPNode::construct(polygons);
}

void BSPTree::traverse(const vec3& rOrig, const vec3& rDir, vector<vector<vec3> >& order) {
    if(root) root->traverse(rOrig, rDir, order);
}

bool BSPTree::findIntersect(const vec3& rOrig, const vec3& rDir, vec3& normal, double& s) {
    if(root) return root->findIntersect(rOrig, rDir, normal, s);
    else return false;
}
