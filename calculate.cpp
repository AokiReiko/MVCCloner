//
//  calculate.cpp
//  Cloner
//
//  Created by aokireiko on 18/11/26.
//  Copyright © 2018年 aokireiko. All rights reserved.
//

#include "calculate.hpp"

Patch::Patch(vector<Point> vs, Mat* src) {
    if (src == NULL) {
        cout << "Patch()-1 srcMat null"<<endl;
        return;
    }
    src->convertTo(srcMat, CV_32FC3);
    this->mesher = NULL;
    this->cdt = NULL;
    for (Point p : vs) {
        boundary.push_back(cPoint(p.x, p.y));
    }
    
    mask = new Mat(src->rows, src->cols, CV_8UC3, Scalar(0,0,0));
        
    int len = (int) vs.size();
    Point rookPoints[1][8194];
    int npt[] = { len };
    for (int i = 0; i < len; i++) {
        rookPoints[0][i] = vs[i];
    }
    const Point* pts[1] = { rookPoints[0] };
    fillPoly(*mask, pts, npt, 1, Scalar(255,255,255));
    
    imshow("mask", *mask);
    posx = posy = 0;
    
}

Patch::~Patch() {
    clear_all();
    delete mask;
    
}

void Patch::preprocess() {
    clear_all();
    build_mesh();
    compute_MVC();
}

void Patch::move_horizontal(int d) {
    posx += d;
    
}
void Patch::move_vertical(int d) {
    posy += d;
}

void Patch::reset_pos() {
    posx = posy = 0;
}



// x, y is the coordinates in src image.
Vec3f Patch::rx(int x, int y) {
    
    if (vertice_map.find(cPoint(x, y)) != vertice_map.end()) {
        return rx_at_vertice[vertice_map[cPoint(x, y)]];
    }
    Face_handle f = cdt->locate(cPoint(x,y));
    
    if (cdt->is_infinite(f)) {
        //cout << "error infinite : " << x << " " << y <<endl;
        return Vec3f(0,0,0);
    }
    Triangle triangle = cdt->triangle(f);
    int ind[3];
    for (int i = 0; i < 3; i++)
        ind[i] = vertice_map[triangle[i]];
    
    //p = (1-u-v)*p1 + u*p2 + v*p3;
    double dx = 0.0 + x - triangle[0].x(), dy = 0.0 +  y - triangle[0].y();
    double dx2 = triangle[1].x() - triangle[0].x(), dy2 = triangle[1].y() - triangle[0].y();
    double dx3 = triangle[2].x() - triangle[0].x(), dy3 = triangle[2].y() - triangle[0].y();
    if (dx3*dy2 == dx2*dy3) {
        cout << "error k = 0: " << x << " " << y <<endl;
        return Vec3f(0,0,0);
    }
    double denom = dx3*dy2 - dx2*dy3;
    
    double u = (0.0 + dx3*dy - dy3*dx) / denom;
    double v = (0.0 + dy2*dx - dx2*dy) / denom;
    double a = 1.0 - u - v;
    if (u < 0 || v < 0 || u > 1 || v > 1) {
        cout << "error" << x << " " << y <<":";
        cout <<dx << "," <<dy <<"/ ";
        cout <<dx2 << "," <<dy2 << "/ ";
        cout <<dx3 << "," <<dy3 <<endl;
        cPoint p;
    }
    //return a * rx_at_vertice[ind[0]] + u * rx_at_vertice[ind[1]] + v * rx_at_vertice[ind[2]];
    return a * rx_at_vertice[ind[0]]*min_dist_from_boundary[ind[0]] + u * rx_at_vertice[ind[1]]*min_dist_from_boundary[ind[1]] + v * rx_at_vertice[ind[2]]*min_dist_from_boundary[ind[2]];
    
}

// Refresh the interpolation values at every vertices.
void Patch::refresh_rx(Mat* tarMat) {
    rx_at_vertice.clear();
    vector<Vec3f> diffs;
    int m = (int) boundary.size();

    int max_r = tarMat->rows, max_c = tarMat->cols;
    
    // First we compute the difference at every vertice
    for (cPoint cp : boundary) {
        int x = cp.x(), y = cp.y();
        int nx = x+posx, ny = y+posy;
        if (nx < 0 || ny < 0 || ny >= max_r || nx >= max_c) diffs.push_back(srcMat.at<Vec3f>(y,x));
        else diffs.push_back(tarMat->at<Vec3f>(ny,nx) - srcMat.at<Vec3f>(y,x));
    }
    // Then we apply interpolation to every vertice
    for (int i = 0; i < vertices.size(); i++) {
        double* lambdas = mvc[i];
        Vec3f v(0,0,0);
        for (int j = 0; j < m; j++) {
            v += lambdas[j]*diffs[j];
        }
        rx_at_vertice.push_back(v);
    }
    
}

// Apply MVC cloning to target Mat.
void Patch::merge(Mat* tarMat, const string& name) {
    Mat tar_f;
    tarMat->convertTo(tar_f, CV_32FC3);
    refresh_rx(&tar_f);
    // This should be called after refresh_rx
    int max_r = tarMat->rows, max_c = tarMat->cols;
   
    for (int i = 0; i < srcMat.rows; i++)
        for (int j = 0; j < srcMat.cols; j++) {
            if (mask->at<Vec3b>(i, j)[0] == 0) continue;
            int ny = i+posy, nx = j+posx;
            if (nx < 0 || ny < 0 || ny >= max_r || nx >= max_c) continue;
            //cout << dd[0] << " " << dd[1] << endl;
            // r = f'- g'
            Vec3f r = rx(j, i);
            if (r == Vec3f(0,0,0)) continue;
            tar_f.at<Vec3f>(ny, nx) = srcMat.at<Vec3f>(i,j) + r;
        }
    tar_f.convertTo(*tarMat, CV_8UC3);
    imshow(name, *tarMat);
}
//Paint the unprocessed patch on target Mat
void Patch::paint_patch(Mat* mat, const string& name) {
    int max_r = mat->rows, max_c = mat->cols;
   
    for (int i = 0; i < srcMat.rows; i++)
        for (int j = 0; j < srcMat.cols; j++) {
            if (mask->at<Vec3b>(i, j)[0] == 0) continue;
            int ny = i+posy, nx = j+posx;
            if (nx < 0 || ny < 0 || ny >= max_r || nx >= max_c) continue;
            mat->at<Vec3b>(ny, nx) = srcMat.at<Vec3f>(i,j);
        }
    imshow(name, *mat);
}
// Paint the trangularized mesh on target Mat
void Patch::paint_mesh(Mat* mat, const string& name) {
    int max_x = mat->cols, max_y = mat->rows;
    if (vertices.size() == 0) return;
    Scalar s(255, 255, 255);
    for (int i = 0; i < triangles.size(); i++) {
        Point p1, p2, p3;
        p1.x = triangles[i].vertex(0).x() + posx;
        p2.x = triangles[i].vertex(1).x() + posx;
        p3.x = triangles[i].vertex(2).x() + posx;
        
        p1.y = triangles[i].vertex(0).y() + posy;
        p2.y = triangles[i].vertex(1).y() + posy;
        p3.y = triangles[i].vertex(2).y() + posy;
        
        
        line(*mat, p1, p2, s);
        line(*mat, p1, p3, s);
        line(*mat, p2, p3, s);
    }
    imshow(name, *mat);
}
// Compute the mvc array for a node(x,y)
double* Patch::MVC(double x, double y) {
    int size = (int) boundary.size();
    if (size == 0) return NULL;
    double *res = new double[size];
    double tan_ad2[size];
    double lens[size];
    double min_dist = INT32_MAX;
    for (int i = 0; i < size; i++) {
        double dxi = boundary[i].x() - x, dyi = boundary[i].y() - y;
        lens[i] = sqrt(dxi*dxi+dyi*dyi);
        min_dist = min(lens[i], min_dist);
        // Too close to a boundary point
        if (lens[i] < 1) {
            min_dist_from_boundary.push_back(1);
            for (int k = 0; k < size; k++)
                res[k] = 0;
            res[i] = 1;
            return res;
        }
    }
    min_dist_from_boundary.push_back(exp(-1.0/min_dist));
    
    cout <<min_dist << " "<<1.0/log10(min_dist) << " " << exp(-1.0/min_dist) << endl;
    
    for (int i = 0; i < size; i++) {
        double dxi = boundary[i].x() - x, dyi = boundary[i].y() - y;
        double dxi1 = boundary[(i+1)%size].x() - x, dyi1 = boundary[(i+1)%size].y() - y;
        double cosx = (dxi*dxi1 + dyi*dyi1) / (lens[i] * lens[(i+1)%size]);
        if (cosx > 1) cosx = 1;
        if (cosx < -1) cosx = -1;
        double alpha = acos(cosx)/2.0;
        tan_ad2[i] = tan(alpha);
    }
    double sum = 0;
    for (int i = 0; i < size; i++) {
        res[i] = (tan_ad2[i] + tan_ad2[(i-1)%size]) / lens[i];
        sum += res[i];
    }
    double test = 0;
    for (int i = 0; i < size; i++) {
        res[i] /= sum;
        test += res[i];
    }
    return res;
    
}

void Patch:: compute_MVC() {
    for (int i = 0; i < vertices.size(); i++) {
        mvc.push_back(MVC(vertices[i].x(), vertices[i].y()));
    }
}

void Patch::build_mesh() {
    int point_num = (int) boundary.size();
    cout << " The number of initial boundary points is " << point_num << endl;
    if (boundary.size() < 3) return;
    
    cdt = new CDT();
    cdt->insert(boundary.begin(), boundary.end());
    for (int i=0 ; i < boundary.size() ; i++) {
        cPoint curr = boundary[i];
        cPoint next = boundary[(i+1) % boundary.size()];
        cdt->insert_constraint(curr,next);
    }
    mesher = new Mesher(*cdt);
    // 0.125 is the default shape bound. It corresponds to abound 20.6 degree.
    // 0.5 is the upper bound on the length of the longuest edge.
    // See reference manual for Delaunay_mesh_size_traits_2<K>.
    mesher->set_criteria(Criteria(0.125, 0));
    mesher->refine_mesh();

    cout << " After delaunay triangulation we get " << vertices.size() << " points and " << triangles.size() << " faces." << endl;
    int i = 0;
    for (CDT::Finite_vertices_iterator iter = cdt->finite_vertices_begin(); iter != cdt->finite_vertices_end(); iter++, i++) {
        vertices.push_back(iter->point());
        vertice_map[iter->point()] = i;
    }
    i = 0;
    for (CDT::Finite_faces_iterator iter = cdt->finite_faces_begin(); iter != cdt->finite_faces_end(); iter++, i++) {
        /*
        if (iter->is_in_domain() == false) {
            numTriangles--;
            continue;
        }*/
        triangles.push_back(cdt->triangle(iter));
    }
    cout << " After missing outer faces we get " << vertices.size() << " points and " << triangles.size() << " faces." << endl;
}

void Patch::clear_all() {
    if (mesher != NULL) delete mesher;
    if (cdt != NULL) delete cdt;
    
    triangles.clear();
    vertices.clear();
    min_dist_from_boundary.clear();
    
    for (double* lambda_arr: mvc) {
        delete[] lambda_arr;
    }
    mvc.clear();
    vertice_map.clear();
    rx_at_vertice.clear();
}