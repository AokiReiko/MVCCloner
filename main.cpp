//
//  main.cpp
//  Cloner
//
//  Created by aokireiko on 18/11/25.
//  Copyright © 2018年 aokireiko. All rights reserved.
//

#define windowL 0
#define windowB 0
#define GAP 10
#define MOVE 5

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cvaux.hpp>
#include <fstream>
#include "calculate.hpp"
using namespace cv;
using namespace std;

void help() {
    cout<<"Thanks for experiencing...draw the points in their order please~ "<<endl;
    printf("Operations are as follows:\n");
    printf("n: start a new poly\n");
    printf("e: complete\n");
    printf("o: apply\n");
    printf("q: quit\n");
    printf("direction: move the image\n");
    printf("s: save as jpg(fixed filename, be carefully!\n)");
}
int radius = 0;
Vec3b brush = (255,0,0);
const char* windowName = "polyfill";

Mat *srcMat = NULL, *tgtMat = NULL;
Mat *showMat = NULL, *showTar = NULL;
bool startDraw = false;
Patch* patch = NULL;
vector<Point> pointList;
/**
 * @function main
 * @brief Main function
 */


void key_event()
{
    
    bool showMesh = false;
    while(1) {
        int key = waitKey();
        switch (key) {
            case 'o':
                //Ok, merge
                tgtMat->copyTo(*showTar);
                //patch->paint_mesh(showTar, "target");
                patch->merge(showTar, "target");
                break;
            case 'n':
                pointList.clear();
                srcMat->copyTo(*showMat);
                //imshow(windowName, *showMat);
                startDraw = true;
                cout<<"restart!"<<endl;
                break;
            case 'q':
                cout<<"quit!"<<endl;
                return;
                break;
                
            case 's':
            {
                char* name="result.jpg";
                IplImage img = IplImage(*showTar);
                cvSaveImage(name, &img);
                break;
            }
            case 'e':
            {
                cout<<"end."<<endl;
                startDraw = false;
                
                int len = (int) pointList.size();
                Point rookPoints[1][8194];
                int npt[] = { len };
                for (int i = 0; i < len; i++) {
                    rookPoints[0][i] = pointList[i];
                }
                const Point* pts[1] = { rookPoints[0] };
                fillPoly(*showMat, pts, npt, 1, Scalar(brush));
                
                // Save and build mesh
                imshow(windowName, *showMat);
                delete patch;
                patch = new Patch(pointList, srcMat);
                patch->preprocess();
                patch->paint_mesh(showMat, windowName);
                tgtMat->copyTo(*showTar);
                patch->paint_patch(showTar, "target");
                break;
            }
            // up
            case 63232:
            // down
            case 63233:
            // left
            case 63234:
            // right
            case 63235:
                if (patch == NULL) break;
                if (key == 63232)
                    patch->move_vertical(-MOVE);
                else if(key == 63233)
                    patch->move_vertical(MOVE);
                else if(key == 63234)
                    patch->move_horizontal(-MOVE);
                else
                    patch->move_horizontal(MOVE);
                //
                tgtMat->copyTo(*showTar);
                //patch->paint_mesh(showTar, "target");
                patch->paint_patch(showTar, "target");
                break;
            default:
                break;
        }
     
    }
    
}

void draw(Mat &m) {
    long len = pointList.size();
    if (len <= 1) return;
    Point p1 = pointList[len-2], p2 = pointList[len-1];
    for (int i = -radius; i <= radius; i++) {
        Point tp1 = p1, tp2 = p2;
        tp1.x += i, tp1.y += i;
        tp2.x += i, tp2.y += i;
        line(m, tp1, tp2, Scalar(brush));
    }
    
}

void on_mouse(int event, int x, int y, int, void*) {
    if (startDraw == false) {
        return;
    }
    //CV_EVENT_LBUTTONDOWN
    if (event == CV_EVENT_MOUSEMOVE) {
        
        Point v;
        v.x = x;
        v.y = y;
        if (pointList.size() == 0) {
            pointList.push_back(v);
            return;
        }
        // TODO: avoid too dense points, use GAP
        Point last = pointList.back();
        int gap = abs(x - last.x) + abs(y - last.y);
        if (gap < GAP) return;
        if (pointList.size() < 8192) {
            pointList.push_back(v);
        }
        draw(*showMat);
        imshow(windowName, *showMat);
        cout << x<<"," << y <<endl;
    }
}
int main( void ){
    
    Mat source, target;
    string tar_path, src_path;
    
    cout << "Input target image:";
    cin >> tar_path;
    printf("target image:%s\n", tar_path.c_str());
    target = imread(tar_path, CV_LOAD_IMAGE_COLOR);
    if(! target.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    cout << "Input source image:";
    cin >> src_path;
    printf("source image:%s\n", src_path.c_str());
    source = imread(src_path, CV_LOAD_IMAGE_COLOR);   // Read the file
    if(! source.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
    
    help();
    
    srcMat = &source;
    tgtMat = &target;
    showMat = new Mat(srcMat->cols, srcMat->rows, CV_32FC3);
    showTar = new Mat(tgtMat->cols, tgtMat->rows, CV_32FC3);
    srcMat->copyTo(*showMat);
    
    cout << "row:" << source.rows << endl;
    cout << "col:" << source.cols << endl;
    
    imshow(windowName, source);
    setMouseCallback(windowName, on_mouse, 0);
    key_event();
    return 0;
}

