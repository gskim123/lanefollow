#include "opencv2/opencv.hpp"
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include "dxl.hpp"
using namespace cv;
using namespace std;
bool ctrl_c_pressed = false;//bool 변수 선언
void ctrlc(int)//컨트롤C가 눌리면 실행
{
    ctrl_c_pressed = true;//bool 변수 ctrl_c_pressed ture로 바꿈
}
int main(void)
{
    /*string src = "nvarguscamerasrc sensor-id=0 ! \
            video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
            format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
            width=(int)640, height=(int)360, format=(string)BGRx ! \
            videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    //카메라 영상을 입력 받기위한 설정
    VideoCapture source(src, CAP_GSTREAMER);//gstreamer를 이용한 카메라 영상 받기
    if (!source.isOpened()){ cout << "Camera error" << endl; return -1; }*/
    //카메라 영상 에러 처리
    VideoCapture source("lanefollow_100rpm_ccw.mp4");//비디오 입력받음
    if (!source.isOpened()) { cerr << "video error" << endl; return -1; }//비디오 영상 에러처리
    string dst = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
                    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
                    h264parse ! rtph264pay pt=96 ! \
                    udpsink host=203.234.58.158 port=8554 sync=false";
    //영상을 gstreamer로 컴퓨터에 송출 하기 위한 설정
    VideoWriter writer(dst, 0, (double)30, Size(640, 360-250), true);//VideoWriter 선언
    if (!writer.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}//에러 처리
    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
                    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
                    h264parse ! rtph264pay pt=96 ! \
                    udpsink host=203.234.58.158 port=8555 sync=false";
    //영상을 gstreamer로 컴퓨터에 송출 하기 위한 설정
    VideoWriter writer1(dst2, 0, (double)30, Size(640, 360), true);//VideoWriter 선언
    if (!writer.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}//에러 처리
    Dxl mx;//Dxl 선언
    struct timeval start,end1;//struct 선언
    double diff1;//double 변수 선언
    int vel1=0,vel2=0, spd = 100, max = 200, min = 10;//int 변수 선언
    bool mode = false, f = false;//bool 변수 선언

    signal(SIGINT, ctrlc);//시그널 핸들러 지정
    if(!mx.open()) { cout << "dxl open error"<<endl; return -1; }//장치열기
    
    Mat frame, gray, bin, dst1, label, stat, centroid;//Mat 변수 선언
    Mat roi;//Mat 변수 선언
    double d = source.get(CAP_PROP_FRAME_WIDTH) / 2, err = 0;//double 변수 선언
    double lx, ly, lxl, lyl, rx, ry, rxl, ryl, dl, dr;//double 변수 선언
    dl = d - d/2;//dl 초기화 영상의 x좌표 1/4
    dr = d + d/2;//dr 초기화 영상의 x좌표 3/4
    lx = ly = lxl = lyl = rx = ry = rxl = ryl = 0;//초기화
    Point r, l;//Point 변수 선언
    while(true)//무한 반복문
    {
        gettimeofday(&start,NULL);//반복문 실행 시작 시간 측정
        source >> frame;//영상을 프레임별로 사진으로 변환
        if (frame.empty()){ cerr << "frame empty!" << endl; break; }//에러 처리
        if (mx.kbhit())//키보드 입력이 있으면 실행
        {
            char c = mx.getch();//키보드로 입력된 키를 저장
            if(c == 'q') break;//키보드로 입력된 키가 q면 탈출
            else if(c == 's') mode = !mode;//키보드로 입력된 키가 s면 mode 바꿈
        }
        roi = frame(Rect(0,250,frame.cols,frame.rows-250));//화면 자르기
        cvtColor(roi,gray,COLOR_BGR2GRAY);//그레이로 변환
        gray += Scalar::all(100) - mean(gray);//영상 밝기 평탄화
        adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 141, -21);
        //적응형 이진화
        morphologyEx(bin, bin, MORPH_OPEN, Mat(), Point(-1, -1));//모폴로지 open 연산 침식 -> 확장
        morphologyEx(bin, bin, MORPH_CLOSE, Mat(), Point(-1, -1));//모폴로지 닫기 연산 확장 -> 침식
        int cnt = connectedComponentsWithStats(bin, label, stat, centroid);//레이블링 검출
        cvtColor(bin, dst1, COLOR_GRAY2BGR);//결과 영상을 컬러로 변환
        double dx1 = 100, dy1 = 50, dx2 = 100, dy2 = 50;//double 변수 선언 및 초기화
        for (int i = 1; i < cnt; i++) {//1부터 레이블링 최대치까지 반복
	        int* b = stat.ptr<int>(i);//바운딩 박스 주소값 저장
            double* c = centroid.ptr<double>(i);//객체 중심점 저장
            if (b[4] < 200) continue;//객체 크기 200미만일 경우 넘어감
            rectangle(dst1, Rect(b[0], b[1], b[2], b[3]), Scalar(255, 0, 0), 2);
            //객체에 파란색 바운딩 박스 그림
            circle(dst1, Point(c[0], c[1]), 2, Scalar(255, 0, 0), 2, FILLED);
            //객체의 중심에 파란색 점 그림
            if(f){//f가 true일때 실행
                if (abs(l.x - c[0]) <= dx1 && abs(l.y - c[1]) <= dy1) {
                    //이전 객체의 x와 이번 객체의 x의 차이의 최소값과 y의 최소값을 구함
                    dx1 = abs(l.x - c[0]);//비교값에 현재 최소값 저장
                    dy1 = abs(l.y - c[1]);//비교값에 현재 최소값 저장
                    l = Point(c[0], c[1]);//현재 객체의 중심점 저장
                    lx = b[0];//바운딩 박스 x좌표 저장
                    ly = b[1];//바운딩 박스 y좌표 저장
                    lxl = b[2];//바운딩 박스 x의 길이 저장
                    lyl = b[3];//바운딩 박스 y의 길이 저장
                }
                if (abs(r.x - c[0]) <= dx2 && abs(r.y - c[1]) <= dy2) {
                    //이전 객체의 x와 이번 객체의 x의 차이의 최소값과 y의 최소값을 구함
                    dx2 = abs(r.x - c[0]);//비교값에 현재 최소값 저장
                    dy2 = abs(r.y - c[1]);//비교값에 현재 최소값 저장
                    r = Point(c[0], c[1]);//현재 객체의 중심점 저장
                    rx = b[0];//바운딩 박스 x좌표 저장
                    ry = b[1];//바운딩 박스 y좌표 저장
                    rxl = b[2];//바운딩 박스 x의 길이 저장
                    ryl = b[3];//바운딩 박스 y의 길이 저장
                }
            } else {
                if (abs(dl - c[0]) < dx1) {
                    //영상의 1/4 지점에서 가장 가까운 객체
                    dx1 = abs(dl - c[0]);//비교값에 현재 최소값 저장
                    l = Point(c[0], c[1]);//현재 객체의 중심점 저장
                    lx = b[0];//바운딩 박스 x좌표 저장
                    ly = b[1];//바운딩 박스 y좌표 저장
                    lxl = b[2];//바운딩 박스 x의 길이 저장
                    lyl = b[3];//바운딩 박스 y의 길이 저장
                }
                if (abs(dr - c[0]) < dx2) {
                    //영상의 3/4 지점에서 가장 가까운 객체
                    dx2 = abs(dr - c[0]);//비교값에 현재 최소값 저장
                    r = Point(c[0], c[1]);//현재 객체의 중심점 저장
                    rx = b[0];//바운딩 박스 x좌표 저장
                    ry = b[1];//바운딩 박스 y좌표 저장
                    rxl = b[2];//바운딩 박스 x의 길이 저장
                    ryl = b[3];//바운딩 박스 y의 길이 저장
                }
            }
        }
        rectangle(dst1, Rect(lx, ly, lxl, lyl), Scalar(0, 0, 255), 2);
        //이전 객체와 현재 객체의 중심의 차이가 최소인 객체에 빨간색 바운딩 박스 그림
        circle(dst1, Point(l.x, l.y), 2, Scalar(0, 0, 255), 2, FILLED);
        //이전 객체와 현재 객체의 중심의 차이가 최소인 객체에 빨간색 중심점 그림
        rectangle(dst1, Rect(rx, ry, rxl, ryl), Scalar(0, 0, 255), 2);
        //이전 객체와 현재 객체의 중심의 차이가 최소인 객체에 빨간색 바운딩 박스 그림
        circle(dst1, Point(r.x, r.y), 2, Scalar(0, 0, 255), 2, FILLED);
        //이전 객체와 현재 객체의 중심의 차이가 최소인 객체에 빨간색 중심점 그림
        err = d - (l.x + r.x)/2;
        //왼쪽과 오른쪽의 객체의 x좌표의 합과 영상의 중심을 비교해서 에러값 구함
        f = true;//영상의 첫프레임을 구별하기 위한 f변수에 true 저장
        if(mode){//mode가 true일때 실행
            if(err){//err값이 0이 아니면 실행
                vel1 = spd - err * 0.4;//왼쪽 바퀴 속력
                vel2 = -spd - err * 0.4;//오른쪽 바퀴 속력
            }
            else{//err값이 0일때
                vel1 = spd;//왼쪽 바퀴 속력
                vel2 = -spd;//오른쪽 바퀴 속력
            }
            if(vel1 > max) vel1 = max;//왼쪽 바퀴 속력 최대치 제한
            if(vel1 < min) vel1 = min;//왼쪽 바퀴 속력 최소치 제한
            if(vel2 < -max) vel2 = -max;//오른쪽 바퀴 속력 최대치 제한
            if(vel2 > -min) vel2 = -min;//오른쪽 바퀴 속력 최소치 제한
            if(!mx.setVelocity(vel1,vel2)){ cout << "setVelocity error"<<endl; return -1;}
            //모터 실행과 에러 처리
        }
        writer << dst1;//결과 영상 보냄
        writer1 << frame;//원본 영상 보냄
        usleep(10*1000);//1ms 쉼
        if (ctrl_c_pressed) break;//ctrl_c_pressed가 true라면 탈출
        gettimeofday(&end1,NULL);//반복문 실행 마지막 시간 측정
        diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
        //시간 계산
        cout <<"err: "<<err<<", vel1: " << vel1 <<", vel2: "<< vel2 << ", time: " << diff1*1000.<<"ms" << endl;
        //에러값, 왼쪽 바퀴 속력, 오른쪽 바퀴 속력, 반복문 실행시간 출력
    }
    mx.close();//모터 끔
    return 0;//0 반환
}