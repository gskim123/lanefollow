#include "opencv2/opencv.hpp"
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include "dxl.hpp"
using namespace cv;
using namespace std;
bool ctrl_c_pressed = false;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}
int main(void)
{
    /*string src = "nvarguscamerasrc sensor-id=0 ! \
            video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
            format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
            width=(int)640, height=(int)360, format=(string)BGRx ! \
            videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    
    VideoCapture source(src, CAP_GSTREAMER);
    if (!source.isOpened()){ cout << "Camera error" << endl; return -1; }*/
    
    VideoCapture source("lanefollow_100rpm_ccw.mp4");
    if (!source.isOpened()) { cerr << "video error" << endl; return -1; }
    string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
                    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
                    h264parse ! rtph264pay pt=96 ! \
                    udpsink host=203.234.58.158 port=8554 sync=false";
    
    VideoWriter writer(dst, 0, (double)30, Size(640, 360-250), true);
    if (!writer.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}
    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
                    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
                    h264parse ! rtph264pay pt=96 ! \
                    udpsink host=203.234.58.158 port=8555 sync=false";
    
    VideoWriter writer1(dst2, 0, (double)30, Size(640, 360), true);
    if (!writer.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}
    Dxl mx;
    struct timeval start,end1;
    double diff1;
    int vel1=0,vel2=0, spd = 100, max = 200, min = 10;
    bool mode = false, f = false;

    signal(SIGINT, ctrlc);
    if(!mx.open()) { cout << "dxl open error"<<endl; return -1; }
    
    Mat frame, gray, bin, dst1, label, stat, centroid;
    Mat roi;
    double d = source.get(CAP_PROP_FRAME_WIDTH) / 2, err = 0;
    double lx, ly, lxl, lyl, rx, ry, rxl, ryl, dl, dr;
    dl = d - d/2;
    dr = d + d/2;
    lx = ly = lxl = lyl = rx = ry = rxl = ryl = 0;
    Point r, l;
    while(true)
    {
        gettimeofday(&start,NULL);
        source >> frame;
        if (frame.empty()){ cerr << "frame empty!" << endl; break; }
        if (mx.kbhit())
        {
            char c = mx.getch();
            if(c == 'q') break;
            else if(c == 's') mode = !mode;
        }
        roi = frame(Rect(0,250,frame.cols,frame.rows-250));
        cvtColor(roi,gray,COLOR_BGR2GRAY);
        gray += Scalar::all(100) - mean(gray);
        adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 141, -21);
        
        morphologyEx(bin, bin, MORPH_OPEN, Mat(), Point(-1, -1));
        morphologyEx(bin, bin, MORPH_CLOSE, Mat(), Point(-1, -1));
        int cnt = connectedComponentsWithStats(bin, label, stat, centroid);
        cvtColor(bin, dst1, COLOR_GRAY2BGR);
        double dx1 = 100, dy1 = 50, dx2 = 100, dy2 = 50;
        for (int i = 1; i < cnt; i++) {
	        int* b = stat.ptr<int>(i);
            double* c = centroid.ptr<double>(i);
            if (b[4] < 200) continue;
            rectangle(dst1, Rect(b[0], b[1], b[2], b[3]), Scalar(255, 0, 0), 2);
            
            circle(dst1, Point(c[0], c[1]), 2, Scalar(255, 0, 0), 2, FILLED);
            
            if(f){//f가 true일때 실행
                if (abs(l.x - c[0]) <= dx1 && abs(l.y - c[1]) <= dy1) {
                    
                    dx1 = abs(l.x - c[0]);
                    dy1 = abs(l.y - c[1]);
                    l = Point(c[0], c[1]);
                    lx = b[0];
                    ly = b[1];
                    lxl = b[2];
                    lyl = b[3];
                }
                if (abs(r.x - c[0]) <= dx2 && abs(r.y - c[1]) <= dy2) {
                    
                    dx2 = abs(r.x - c[0]);
                    dy2 = abs(r.y - c[1]);
                    r = Point(c[0], c[1]);
                    rx = b[0];
                    ry = b[1];
                    rxl = b[2];
                    ryl = b[3];
                }
            } else {
                if (abs(dl - c[0]) < dx1) {
                    
                    dx1 = abs(dl - c[0]);
                    l = Point(c[0], c[1]);
                    lx = b[0];
                    ly = b[1];
                    lxl = b[2];
                    lyl = b[3];
                }
                if (abs(dr - c[0]) < dx2) {
                    
                    dx2 = abs(dr - c[0]);
                    r = Point(c[0], c[1]);
                    rx = b[0];
                    ry = b[1];
                    rxl = b[2];
                    ryl = b[3];
                }
            }
        }
        rectangle(dst1, Rect(lx, ly, lxl, lyl), Scalar(0, 0, 255), 2);
        
        circle(dst1, Point(l.x, l.y), 2, Scalar(0, 0, 255), 2, FILLED);
        
        rectangle(dst1, Rect(rx, ry, rxl, ryl), Scalar(0, 0, 255), 2);
        
        circle(dst1, Point(r.x, r.y), 2, Scalar(0, 0, 255), 2, FILLED);
        
        err = d - (l.x + r.x)/2;
        
        f = true;
        if(mode){
            if(err){
                vel1 = spd - err * 0.4;
                vel2 = -spd - err * 0.4;
            }
            else{
                vel1 = spd;
                vel2 = -spd;
            }
            if(vel1 > max) vel1 = max;
            if(vel1 < min) vel1 = min;
            if(vel2 < -max) vel2 = -max;
            if(vel2 > -min) vel2 = -min;
            if(!mx.setVelocity(vel1,vel2)){ cout << "setVelocity error"<<endl; return -1;}
            
        }
        writer << dst1;
        writer1 << frame;
        usleep(10*1000);
        if (ctrl_c_pressed) break;
        gettimeofday(&end1,NULL);
        diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
        
        cout <<"err: "<<err<<", vel1: " << vel1 <<", vel2: "<< vel2 << ", time: " << diff1*1000.<<"ms" << endl;
        
    }
    mx.close();
    return 0;
}
