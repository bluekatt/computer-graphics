//
//  main.cpp
//  Computer Graphics
//
//  Created by 박종석 on 2021/09/01.
//

#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>
#include "shape.hpp"
#include "vec3.hpp"
#include "BSPTree.hpp"

#define INF 1e8
#define MAX_DEPTH 5

using namespace std;

vec3 reflect(const vec3& I, const vec3& N) {
    return I - N * 2 * I.dotProduct(N);
}

vec3 refract(const vec3& I, const vec3& N, const double nt) {
    double cosi = I.dotProduct(N);
    double etai = 1, etat = nt;
    vec3 n = N;
    if (cosi < 0) {
        cosi = -cosi;
    } else {
        swap(etai, etat); n= -N;
    }
    double eta = etai / etat;
    double k = 1 - eta * eta * (1 - cosi * cosi);
    if (k < 0) {
        return vec3();
    } else {
        return I * eta + n *(eta * cosi - sqrtf(k));
    }
}

double fresnel(const vec3& I, const vec3& N, const double& nt) {
    double cosi = I.dotProduct(N);
    double etai = 1, etat = nt;
    if(cosi > 0) {
        swap(etai, etat);
    }
    
    double sini = sqrt(1 - cosi * cosi);
    
    double sint = etai / etat * sini;
    double fresneleffect = 1;
    if(sint >= 1) {
        fresneleffect = 1;
    } else {
        double cost = sqrt(1 - sint * sint);
        cosi = abs(cosi);
        double rs = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        double rp = ((etai * cost) - (etat * cosi)) / ((etai * cost) + (etat * cosi));
        fresneleffect = (rs * rs + rp * rp) / 2;
    }
    return fresneleffect;
}

bool checkIntersect(const vec3& rOrig, const vec3& rDir, const vector<shape*>& shapes, vec3& normal, double& s, shape** hit) {
    double sMin = INF;
    vec3 n;
    for(int i=0;i<shapes.size();i++) {
        if(shapes[i]->intersect(rOrig, rDir, n, s)) {
            if(s<sMin) {
                sMin = s;
                normal = n;
                *hit = shapes[i];
            }
        }
    }
    s = sMin;
    if(sMin == INF) return false;
    else return true;
}

vec3 trace(const vec3& rOrig, const vec3& rDir, const vector<shape*>& shapes, const vector<shape*>& lights, int depth) {
    double s = INF;
    shape* hit = NULL;
    
    vec3 surfaceColor = vec3();
    vec3 hitNormal;
    
    if(checkIntersect(rOrig, rDir, lights, hitNormal, s, &hit)) return hit->color;
    if(!checkIntersect(rOrig, rDir, shapes, hitNormal, s, &hit)) return vec3(1);
    
    vec3 hitPoint = rOrig + rDir * s;
    
    double bias = 1e-2;
    if ((hit->transparency > 0 || hit->reflection > 0) && depth < MAX_DEPTH) {
        double nt = hit->refIdx;
        double fresneleffect = fresnel(rDir, hitNormal, nt);
        
        vec3 reflDir = reflect(rDir, hitNormal);
        reflDir.normalize();
        vec3 reflOrig = (reflDir.dotProduct(hitNormal)>0) ? hitPoint + hitNormal * bias : hitPoint - hitNormal * bias;
        vec3 reflection = trace(reflOrig, reflDir, shapes, lights, depth + 1);
        
        vec3 refraction = vec3();
        if (hit->transparency) {
            vec3 refrDir = refract(rDir, hitNormal, nt);
            refrDir.normalize();
            vec3 refrOrig = (refrDir.dotProduct(hitNormal)>0) ? hitPoint + hitNormal * bias : hitPoint - hitNormal * bias;
            refraction = trace(refrOrig, refrDir, shapes, lights, depth + 1);
        }
        double reflectCoefficient = (hit->reflection + (1-hit->reflection) * fresneleffect);
        surfaceColor = (
            reflection * reflectCoefficient +
                        refraction * (1 - reflectCoefficient) * hit->transparency) * hit->color;
    } else {
        vec3 amb = vec3(0.1, 0.1, 0.1);
        for(int i=0;i<lights.size();i++) {
            double transmission = 1;
            vec3 lightDir = lights[i]->center - hitPoint;
            lightDir.normalize();
            for(int j=0;j<shapes.size();j++) {
                vec3 n;
                double s;
                vec3 lightOrig = (lightDir.dotProduct(hitNormal) > 0) ? hitPoint + hitNormal * bias : hitPoint - hitNormal * bias;
                if (shapes[j]->intersect(lightOrig, lightDir, n, s)) {
                    if (shapes[j]->transparency) {
                        transmission = transmission * pow(shapes[j]->transparency, 2); // ray pass through translucent object twice.
                    } else {
                        transmission = 0;
                    }
                    break;
                }
            }
            vec3 reflDir = reflect(-lightDir, hitNormal);
            vec3 specular = lights[i]->color * pow(max(0.0, -reflDir.dotProduct(rDir)), hit->shininess) * hit->ks;
            vec3 ambient = amb * hit->ka;
            vec3 diffuse = lights[i]->color * max(0.0, hitNormal.dotProduct(lightDir)) * hit->kd;
            surfaceColor = surfaceColor + hit->color * transmission * (specular + ambient + diffuse);
            if(surfaceColor.length() > sqrt(3)) surfaceColor = surfaceColor.normalized() * sqrt(3);
        }
    }
    
    return surfaceColor;
}

vector<string> split(const string& s, const string& delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

void readObjFile(ifstream& dataFile, double scale, vector<vector<vec3> >& polygons, vector<vector<vec3> >& normals) {
    string line;
    stringstream ss;
    string word;
    
    vector<vec3> vertices;
    vector<vec3> fnormals;
    vector<double> realNormals = vector<double>();
    
    while(!dataFile.eof()) {
        getline(dataFile, line);
        while(line.empty() || line.at(0) == 13 || line.at(0) == '#') {
            if (dataFile.eof()) break;
            getline(dataFile, line);
        }
        ss.clear();
        ss.str(line);
        ss >> word;
        if(word == "v") {
            double x, y, z;
            ss >> x >> y >> z;
            x *= scale; y *= scale; z *= scale;
            vertices.push_back(vec3(x, y, z));
        } else if(word == "vn") {
            double x, y, z;
            ss >> x >> y >> z;
            fnormals.push_back(vec3(x, y, z));
        } else if(word == "f") {
            vector<string> faces = vector<string>();
            ss >> word;
            while(ss) {
                faces.push_back(word);
                ss >> word;
            }
            
            vector<int> faceVIndices = vector<int>();
            vector<int> faceNIndices = vector<int>();
            
            for(int i=0;i<faces.size();i++) {
                vector<string> splitted = split(faces[i], "/");
                int vIndex = atoi(splitted[0].c_str()) - 1;
                faceVIndices.push_back(vIndex);
                if(splitted.size()>=3) {
                    int nIndex = atoi(splitted[2].c_str()) - 1;
                    faceNIndices.push_back(nIndex);
                }
            }
            
            vector<vec3> newFace;
            vector<vec3> newNormal;
            
            for(int i=0;i<faceVIndices.size();i++) {
                int vIndex = faceVIndices[i];
                newFace.push_back(vertices[vIndex]);
                
                if(i<faceNIndices.size()) {
                    int nIndex = faceNIndices[i];
                    newNormal.push_back(fnormals[nIndex]);
                }
            }
            
            polygons.push_back(newFace);
            normals.push_back(newNormal);
        }
    }
}

void render(const vector<shape*>& shapes, const vector<shape*>& lights) {
    int width = 1000, height = 1000;
    vec3 *image = new vec3[width * height], *pixel = image;
    double invWidth = 1 / double(width), invHeight = 1 / double(height);
    double fov = 60, aspectratio = width / double(height);
    double angle = tan(M_PI * 0.5 * fov / 180);
    for (int j=0;j<height;j++) {
        for (int i=0;i<width;i++) {
            double x = (2 * ((i + 0.5) * invWidth) - 1) * angle * aspectratio;
            double y = (1 - 2 * ((j + 0.5) * invHeight)) * angle;
            vec3 rDir(x, y, -1);
            rDir.normalize();
            *pixel = trace(vec3(), rDir, shapes, lights, 0);
            pixel++;
        }
    }
    ofstream ofs("./result.ppm", ios::out | ios::binary);
    ofs << "P6\n" << width << " " << height << "\n255\n";
    for (unsigned i = 0; i < width * height; ++i) {
        ofs << (unsigned char)(min(double(1), image[i].x) * 255) <<
               (unsigned char)(min(double(1), image[i].y) * 255) <<
               (unsigned char)(min(double(1), image[i].z) * 255);
    }
    ofs.close();
    delete [] image;
}


int main(int argc, char * argv[]) {
    vector<shape*> shapes;
    vector<shape*> lights;
    
//    shapes.push_back(new sphere(vec3(0.0, 0, -20), 4, vec3(1.0, 1.0, 1.0), 0.3, 0.9, 1.03));
    shapes.push_back(new sphere(vec3(10, -6, -27), 3, vec3(0.90, 0.76, 0.46), 0.5, 0.0, 1.1));
    shapes.push_back(new sphere(vec3(7, -5, -32), 3, vec3(0.65, 0.77, 0.97), 0.5, 0.0, 1.1));
    shapes.push_back(new sphere(vec3(11, -1, -37), 3, vec3(0.90, 0.27, 0.20), 0.5, 0.0, 1.1));
    shapes.push_back(new sphere(vec3(14, -4, -32), 3, vec3(0.90, 0.76, 0.46), 0.3, 0.0, 1.1));
    shapes.push_back(new sphere(vec3(-10, -5, -25), 4, vec3(1.0, 1.0, 1.0), 0.1, 0.9, 1.5));
    
    vector<vector<vec3> > polygons1;
    vector<vector<vec3> > normals1;
    vector<vec3> polygon1;
    vector<vec3> normal1;
    polygon1.push_back(vec3(-20, 0, 20));
    polygon1.push_back(vec3(20, 0, 20));
    polygon1.push_back(vec3(20, 0, -20));
    polygon1.push_back(vec3(-20, 0, -20));
    normal1.push_back(vec3(0, 1, 0));
    normal1.push_back(vec3(0, 1, 0));
    normal1.push_back(vec3(0, 1, 0));
    normal1.push_back(vec3(0, 1, 0));
    polygons1.push_back(polygon1);
    normals1.push_back(normal1);
//    shapes.push_back(new polygonMesh(vec3(0, -4, -20), polygons1, normals1, vec3(0.35, 0.35, 0.35), 0.0, 0.0, 1.0));
    
    vector<vector<vec3> > polygons2;
    vector<vector<vec3> > normals2;
    string fname = "can.obj";
    ifstream dataFile(fname);
    if (!dataFile.is_open()) return 0;
    readObjFile(dataFile, 120, polygons2, normals2);
    dataFile.close();
    shapes.push_back(new polygonMesh(vec3(0, -8, -43), polygons2, normals2, vec3(0.7, 0.7, 0.7), 0.0, 0.0, 1, 0.1, 0.3, 0.2, 30));
    shapes.push_back(new polygonMesh(vec3(-9, -8, -43), polygons2, normals2, vec3(0.9, 0.9, 0.9), 0.8, 0.0, 1));
    
    vector<vector<vec3> > polygons3;
    vector<vector<vec3> > normals3;
    fname = "ico_sphere.obj";
    dataFile.open(fname);
    if (!dataFile.is_open()) return 0;
    readObjFile(dataFile, 3, polygons3, normals3);
    dataFile.close();
    shapes.push_back(new polygonMesh(vec3(-1, -9.5, -24), polygons3, normals3, vec3(0.75, 0.95, 0.95), 0.3, 0.9, 1.1));
    shapes.push_back(new polygonMesh(vec3(-5, -6.5, -28), polygons3, normals3, vec3(0.75, 0.95, 0.95), 0.3, 0.9, 1.4));
    
    vector<vector<vec3> > polygons6;
    vector<vector<vec3> > normals6;
    fname = "table.obj";
    dataFile.open(fname);
    if (!dataFile.is_open()) return 0;
    readObjFile(dataFile, 1.2, polygons6, normals6);
    dataFile.close();
    shapes.push_back(new polygonMesh(vec3(-7, -102.5, -77), polygons6, normals6, vec3(0.43, 0.32, 0.15), 0.0, 0.0, 1, 0.1, 0.5, 0.2, 8));
    
    // light
    lights.push_back(new sphere(vec3(0.0, 30, 10), 3, vec3(3.00, 3.00, 3.00)));
    render(shapes, lights);
    
    return 0;
}
