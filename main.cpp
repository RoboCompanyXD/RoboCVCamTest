/* 
 * File:   cameradet2.cpp
 * Author: ehu
 *
 * Created on 2 de marzo de 2020, 11:03
 */

#include <cstdlib>
#include <stdlib.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
//#include "ControlRobot.h"
#include <wiringPi.h> 
#include <vector>

#include <sys/stat.h>
#include <time.h>

//#include <cv.h> 
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

int x,xp,y,yp,area,frame_width,frame_height,foto=0;
/*
 * 
 */
int main(int argc, char** argv) {
    
    cout << "OpenCV version: "<< CV_VERSION << endl;
    xp=30;
    yp=45;
    
    clock_t start_t, end_t;
    start_t = clock();
      
    vector<int> compression_params;
    vector<Rect> bodies;
    
    compression_params.push_back(IMWRITE_JPEG_QUALITY);
    compression_params.push_back(90);
    
    Mat frame,hsv_frame,mask,frame_gray,mask_grey,final_mask;
    int frames_nodetect=0;
    
    Moments mu;
    Point2f mc;
    frame_width = cap.get(CAP_PROP_FRAME_WIDTH);
    frame_height = cap.get(CAP_PROP_FRAME_HEIGHT);
    
    //Mat mask_grey(frame_height,frame_width);
    mask_grey = Mat::zeros(Size(frame_width,frame_height), CV_8UC1);
        
    /*
     *      Directorio en el que se guarda la captura
     */
    struct stat sb;
    if (!(stat("/tmp/robonitor", &sb) == 0 && S_ISDIR(sb.st_mode))){
        const int dir_err = system("mkdir -p /tmp/robonitor");
        if (dir_err == -1)
            {
                printf("Error creating directory!n");
                exit(-1);
            }
        }

    /*
     *      Rango del color a detectar
     */
    Scalar lower_yellow = Scalar(24, 135, 93, 0);
    Scalar upper_yellow= Scalar(42, 252, 255, 0);
    Scalar lower_green = Scalar(54, 145, 16, 0);
    Scalar upper_green= Scalar(100, 255, 128, 0);
    
    
    VideoCapture cap(0);
    
    /*
     *      Haar-cascade Detection 
     */
    CascadeClassifier body_classifier;
    body_classifier = CascadeClassifier("/home/pi/pedestrian-detection/haarcascade_upperbody.xml");
    
    while(1){
    	cap >> frame;
        
        /*
         * Detección del color
         * 
         */
        
    	cvtColor(frame, hsv_frame, COLOR_BGR2HSV);         //Cambio del espacio de color
    	inRange(hsv_frame,lower_green,upper_green,mask);    //Filtrado
        
        erode(mask,mask,getStructuringElement(MORPH_ELLIPSE, Size(2,2)));  //Erosionar
                
        mu = moments(mask);  // Calculo momentos y posicion
        if (mu.m00>30000){
    	   mc = Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
           x = int(mc.x);
           y = int(mc.y);
           area = int(mu.m00);
        }else{
            mc = Point2f(0,0);
            x=0;
            y=0;
            area=0;
        }
        
        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);  // cambio a grises
        resize(frame_gray, frame_gray,Size(),0.25,0.25,INTER_LINEAR); 

        /*
         * Deteccion de 
         * upperbodies
         * 
         */
        
        body_classifier.detectMultiScale( frame_gray, bodies, 1.05, 3 ); 
        
        
        /*
         * Prueba para utilizar las 9 ultimos frames en el que se ha detectado persona
         */
        if (frames_nodetect>9)mask_grey = Mat::zeros(Size(frame_width,frame_height), CV_8UC1); 
        if (bodies.size()>0){
            frames_nodetect=0;
            cout << "Vector size: "<< bodies.size() <<endl;
            for (int i=0;i<bodies.size();i++){
                //cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
                rectangle(mask_grey,Point(bodies[i].x,bodies[i].y),Point(bodies[i].x+bodies[i].width,bodies[i].y+bodies[i].height),Scalar(255, 255, 255), FILLED);
                //rectangle(frame,Point(bodies[i].x,bodies[i].y),Point(bodies[i].x+bodies[i].width,bodies[i].y+bodies[i].height),Scalar(0, 255, 255), 2);
                //rectangle(mask,Point(xp,yp),Point(xp+50,yp+100),Scalar(255, 255, 255), 2);
            }
        }else{frames_nodetect++;}
        
        /*
         * Interseccion de deteccion por color y reconocimiento
         */
        bitwise_and(mask, mask_grey, final_mask); //interseccion de deteccion por color y reconocimiento
        
//        imshow( "Mask", mask_grey );
//        if( waitKey(10) == 27 )
//        {
//            break; // escape
//        }
//        imshow( "Detect", frame );
//        if( waitKey(10) == 27 )
//        {
//            break; // escape
//        }
//        imshow( "Origi", final_mask );
//        if( waitKey(10) == 27 )
//        {
//            break; // escape
//        }
        
        end_t = clock();
        
        /*
         * Guardar captura cada segundo
         */
//        if ((double)(end_t - start_t) / CLOCKS_PER_SEC > 1.0){  
//            start_t = end_t;
//            system("touch /tmp/robonitor/.lock");
//            imwrite("/tmp/robonitor/cap.jpg", frame, compression_params);
//            system("unlink /tmp/robonitor/.lock");
//        }

    }
    return 0;
}

