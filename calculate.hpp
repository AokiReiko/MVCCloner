//
//  calculate.hpp
//  Cloner
//
//  Created by aokireiko on 18/11/26.
//  Copyright © 2018年 aokireiko. All rights reserved.
//

#ifndef calculate_hpp
#define calculate_hpp

#include <vector>
#include <set>
#include <map>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <stdio.h>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_conformer_2.h>
#include <CGAL/Triangulation_vertex_base_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel     K;
typedef CGAL::Triangulation_vertex_base_2<K>                    Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K>                      Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb>            Tds;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds>      CDT;
typedef CDT::Point                                              cPoint;
typedef CDT::Vertex_handle                                      Vertex_handle;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT>                Criteria;
typedef CGAL::Delaunay_mesher_2<CDT, Criteria>                  Mesher;
typedef CDT::Triangle                                           Triangle;
typedef CDT::Face                                               Face ;
typedef Face::Face_handle                                       Face_handle ;
typedef CDT::Face_circulator                                    Face_circulator ;


using namespace std;
using namespace cv;

// A patch denotes a selected area of the src image.
class Patch {
public:
    Patch(){}
    Patch(vector<Point> vs, Mat* src);
    ~Patch();
    
    void preprocess();
    void move_horizontal(int d);
    void move_vertical(int d);
    void reset_pos();
    void paint_mesh(Mat*, const string&);
    void paint_patch(Mat*, const string&);
    void merge(Mat*, const string&);

private:
    void refresh_rx(Mat*);
    void build_mesh();
    void compute_MVC();
    double* MVC(double x, double y);
    Vec3f rx(int, int);
    double dist(int x1, int y1, int x2, int y2) {
        return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
    }
    void clear_all();
private:
    Mat srcMat;
    vector<cPoint> boundary;
    Mat* mask;
    
    // Position from the original place
    int posx, posy;
    
    // To build mesh
    Mesher* mesher;
    CDT *cdt;
    
    // Triangles in mesh
    vector<Triangle> triangles;
    // Vertices(at boundary or inner points) in mesh
    vector<cPoint> vertices;
    // Map vertices to indices
    map<cPoint, int> vertice_map;
    
    // lambda array at every vertice
    vector<double*> mvc;
    //rx at every vertices
    vector<Vec3f> rx_at_vertice;
    vector<double> min_dist_from_boundary;
};

#endif /* calculate_hpp */
