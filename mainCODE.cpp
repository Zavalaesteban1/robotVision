#include <unordered_map>
#include "opencv2/opencv.hpp"
#include <wiringPi.h>
#include <vector>
#include <iostream>
//#include <pthread.h>
#include "RTIMULib.h"

using namespace std;
using namespace cv;

void createMap();
void robotKilla(unordered_map<char, array<int, 6>> &map);
void setup();
void mForward();
void mBackward();
void mRight();
void mLeft();
void rLeft();
void rRight();
void stop();
float ultrasonicSensor(int sensNum);
void spinMe();
double BNO055();
bool findColorCenter(Mat& image, const Scalar& lowerBound, const Scalar& upperBound, Point& center);
bool alignRobotWithTarget(const Point& frameCenter, const Point& targetCenter);
void travelToTarget();


RTIMUSettings *settings = new RTIMUSettings("RTIMULib");

RTIMU *imu = RTIMU::createIMU(settings);
    

//Declaration of GPIO pins
const int PWMf1 = 0;  // GPIO pin 12 mapped to WiringPi pin 1
const int f1B = 1;  // GPIO pin 18 mapped to WiringPi pin 5
const int f1A = 2;  // GPIO pin 16 mapped to WiringPi pin 4
// B1
const int b1A = 4;  // GPIO pin 15 mapped to WiringPi pin 3
const int b1B = 5;  // GPIO pin 13 mapped to WiringPi pin 2
const int PWMb1 = 3;  // GPIO pin 11 mapped to WiringPi pin 0

// f2
const int PWMf2 = 12;  // GPIO pin 12 mapped to WiringPi pin 1
const int f2B = 13;  // GPIO pin 18 mapped to WiringPi pin 5
const int f2A = 14;  // GPIO pin 16 mapped to WiringPi pin 4
// B2
const int b2A = 11;  // GPIO pin 15 mapped to WiringPi pin 3
const int b2B = 21;  // GPIO pin 13 mapped to WiringPi pin 2
const int PWMb2 = 22;  // GPIO pin 11 mapped to WiringPi pin 0


//US1
const int TRIG_1 = 15;
const int ECHO_1 = 16;
//US2
const int TRIG_2 = 6;
const int ECHO_2 = 10;
//US3
const int TRIG_3 = 27;
const int ECHO_3 = 28;

uint64_t displayTimer;
uint64_t now;



//pthread_mutex_t mutex1 = PTHREAD_MUTEX_INITIALIZER;

//pthread_t thread1, thread2;

int main()
{
    
	
    setup();
    
    //if (test1 = pthread_create(&thread1, NULL, &createMap, NULL))
    //{
			//cout << "thread shat itself, fuck (1)" << endl;
	//}
    // createMap takes care of the whole program
    
    createMap();
	
	//pthread_join(thread1, NULL);
	return 0;
}


void setup()
{
	wiringPiSetup();
    pinMode(PWMf1, OUTPUT);
    pinMode(f1A, OUTPUT);  
    
    pinMode(f1B, OUTPUT);
    pinMode(b1A, OUTPUT);
    pinMode(b1B, OUTPUT);
    pinMode(PWMb1, OUTPUT);
    pinMode(PWMf2, OUTPUT);
    pinMode(f2A, OUTPUT);
    pinMode(f2B, OUTPUT);
    pinMode(b2A, OUTPUT);
    pinMode(b2B, OUTPUT);
    pinMode(PWMb2, OUTPUT);
    digitalWrite(PWMf1, HIGH);
    digitalWrite(PWMf2, HIGH);
    digitalWrite(PWMb1, HIGH);
    digitalWrite(PWMb2, HIGH);
    pinMode(TRIG_1, OUTPUT);
    pinMode(TRIG_2, OUTPUT);
    pinMode(TRIG_3, OUTPUT);
    pinMode(ECHO_1, INPUT);
    pinMode(ECHO_2, INPUT);
    pinMode(ECHO_3, INPUT);
    
    if((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL))
    {
        printf("No IMU found\n");
        exit(1);
	}
    
    imu->IMUInit();
    
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(true);
    imu->setCompassEnable(true);
  
}
// in this function it creates the map with the keys being the colors and the values being the hsv values of the color
// it also calles showHsvValues fucntions that passes it the first letter of the color and it shows in the camera that following color based on the hsv value
// killing two birds with one stone

void createMap()
{


    // declare the map
    unordered_map<char, array<int, 6>> hsvMap;
    // fill the map with the values
    
	hsvMap.insert({'A', {0,219,65,3, 255, 225}});
	// insert Orange
	hsvMap.insert({'B', {0, 230, 47, 5, 255, 255}});
	// insert Yellow
	hsvMap.insert({'C', {20,226,161,25,255,255}});
	// insert Light green  
 	hsvMap.insert({'D', {56,169,29,93,255,97}});
 	// insert brown
 	hsvMap.insert({'N', {14,122,158,19,164,194}});
	// insert black
	hsvMap.insert({'F', {83, 52, 17, 109, 148, 245}});
	// insert purple
	hsvMap.insert({'Z', {149,55,60,177,116,105}});
	// insert blue
	hsvMap.insert({'H', {93,255,29,177,255,255}});

    // call the following function to run the program that allows the robot to dectect color, moving to each color 
    robotKilla(hsvMap);

}


void robotKilla(unordered_map<char, array<int, 6>> &hsvMap)
{
	//int test2;
    vector<char> colors;
    char x;
    // color array
    // colors = {'A', 'D', 'H', 'F', 'B', 'G', 'E', 'C', 'A'};
    
   /* 
   cout << "Enter the letter to start with: ";
    cin >> x;

    // switch case to get the pattern based on the letter we chose 
    
    switch(x)
    {
        case 'A':
            colors = {'A','D', 'H','F', 'B', 'Z', 'N', 'C', 'A'};
            break;
        case 'B':
            colors = {'B', 'Z', 'N', 'C', 'A','D', 'H', 'F', 'B'};
            break;
        case 'C':
            colors = {'C', 'A', 'D', 'H', 'F', 'B', 'Z', 'N', 'C'};
            break;
        case 'D':
            colors = {'D', 'H', 'F', 'B', 'Z', 'N', 'C', 'A', 'D'};
            break;
        case 'E':
            colors = {'N', 'C', 'A', 'D', 'H', 'F', 'B', 'Z', 'N'};
            break;
        case 'F':
            colors = {'F', 'B', 'Z', 'N', 'C', 'A', 'D', 'H', 'F'};
            break;
        case 'G':
            colors = {'Z', 'N', 'C', 'A', 'D', 'H', 'F', 'B', 'Z'};
            break;
        case 'H':
            colors = {'H', 'F', 'B', 'Z', 'N', 'C', 'A', 'D', 'H'};
            break;
        default:
            cout << "Pick the right letter next time dumbass" << endl;
    }
    */
    
    
    //
  // colors = {'A','D', 'H','F', 'B', 'Z', 'N', 'C', 'A'};
    colors = {'A','D', 'H','A'};
   // colors = {'A'};
    Mat image, imgHSV, mask;
    Point center;

    VideoCapture cap(0);
   

    if (!cap.isOpened())
    {
        cout << "Cannot open camera" << endl;
    }
    
    
	bool aligned = false;
	// this loop takes care of running through each color in the course
    for(int i = 0; i <= colors.size(); i++)
    {
		
        delay(1000);
        aligned = false;
        while(true)
        {
			
            if(hsvMap.find(colors[i]) != hsvMap.end())
            {
				
				cout << "looking for color:"<<  colors[i] << endl;
                Scalar lower(hsvMap.at(colors[i])[0], hsvMap.at(colors[i])[1], hsvMap.at(colors[i])[2]);
                Scalar upper(hsvMap.at(colors[i])[3], hsvMap.at(colors[i])[4], hsvMap.at(colors[i])[5]);
                // 1. we find the color in the map
				// if it finds the color, the corresponding letter is passed into the following algorthim 
				cap >> image;
				cvtColor(image, imgHSV, COLOR_BGR2HSV);
               
                //cap >> image;
				//cvtColor(image, imgHSV, COLOR_BGR2HSV);
                // rotate the robot to the right until the color we're looking for is found
                if(!findColorCenter(image,lower,upper, center))
                {
					cout << "Else" << endl;
					rRight();
					cout << "Right" << endl;
			3	}
				else
				{	
					
					cout << "before alignment" << endl;
                    // 3. stop the robot
					// once it finds the color stop
                    stop();
                    // 4. Start the alignment of the robot 
					// Find the center of the color-detected area
                    aligned = alignRobotWithTarget(Point(image.cols / 2, image.rows / 2), center);
                    cout << "after aligment" << endl;
				}

                
                        
                  // Apply the mask to the original image
                inRange(imgHSV, lower, upper, mask);
                // show another way of picking up the color
                Mat result;
                bitwise_and(image, image, result, mask);
                // Display the result
          
                imshow("Result", result);

                
				imshow("Display window", image);
           
            }
           
			imshow("Mask", mask);
			
            if(waitKey(30) == 27 || aligned)
            {
                break;
            }  

        }
        

        // 5. travel to target
        // once its align go to the target 
        //if (test2 = pthread_create(&thread2, NULL, &travelToTarget, NULL))
		//{
			//cout << "thread shat itself, fuck (2)" << endl;
		//}
		
		//pthread_join(thread2, NULL);
        travelToTarget();
        
        // 6. Here will align ourselves with the wall using the bno or possibly a function that will help us do that 
        
    }
    
    cout << "loop ends" << endl;
}


float ultrasonicSensor(int sensNum)
{
    int TRIG;
    int ECHO;
    string side;
    if(sensNum == 1)
    {
        TRIG = 15;ECHO = 16;
        side = "left";
    }
    else if(sensNum == 2)
    {
        TRIG = 6; ECHO = 10;
        side = "center";
    }
    else
    {
        TRIG = 27;ECHO = 28;
        side = "Right";
    }
    long startTime, endTime;
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    while(digitalRead(ECHO) == LOW){}
    startTime = micros();
    while(digitalRead(ECHO) == HIGH){}
    endTime = micros();
    long duration = endTime - startTime;
    float distance = duration * 0.0343 / 2;
    cout << "Current Distance Reading from" << side << "sensor: " << distance << endl;
    return distance;
}
void travelToTarget()
{
    stop();
    delay(500);
    double refAngle = BNO055();
	double bnoAngle = BNO055();
    while(ultrasonicSensor(2) >= 10)
    {
			
		while (bnoAngle < refAngle - 2 )
		{
			rRight();
			//stop();
			cout << "turning right" << endl;
			bnoAngle = BNO055();
		}
		while (bnoAngle > refAngle + 2)
		{
			rLeft();
			//stop();
			bnoAngle = BNO055();
		}
        mForward();
      }
        stop();
       
      spinMe();
      
      //code for making the robot move forward for 4 seconds
      displayTimer = now = RTMath::currentUSecsSinceEpoch();
      while ((now - displayTimer) < 4000000)
      {
		   mForward;
		   cout << "moving forward" << endl;
		   now = RTMath::currentUSecsSinceEpoch();
		}
		
    stop();
       /*
    if(ultrasonicSensor(2) <= 10){
    spinMe();
    cout << " align logic next (if necessary)" << endl;
	}
	*/
}

//}

double BNO055()
{
	usleep(imu->IMUGetPollInterval() * 1000);
	double angle;
     if (imu->IMURead())
     {
		RTIMU_DATA imuData = imu->getIMUData();
            
		angle = imuData.fusionPose.z();
		angle *= RTMATH_RAD_TO_DEGREE;
		cout << "Angle: " << angle<< "degree" << endl;
	}
	return angle;
}

void spinMe()
{
	double refAngle = BNO055();
	double bnoAngle = BNO055();
	if (refAngle < 180)
	{
		while (bnoAngle < refAngle + 180)
		{
			rRight();
			//stop();
			cout << "turning right (1)" << endl;
			bnoAngle = BNO055();
		}
	}
	else if (refAngle >= 180.0)
	{
		while (bnoAngle > (refAngle - (180.0)) )
		{
			rLeft();
			//stop();
			cout << "turning left (2)" << endl;
			bnoAngle = BNO055();
		}
	}
stop();
}

void mForward()
{
    digitalWrite(f1A, HIGH);
    digitalWrite(f1B, LOW);
    digitalWrite(f2A, HIGH);
    digitalWrite(f2B, LOW);
    digitalWrite(b1A, HIGH);
    digitalWrite(b1B, LOW);
    digitalWrite(b2A, HIGH);
    digitalWrite(b2B, LOW);
}

void mBackward()
{
    //top right
    digitalWrite(f1A, LOW);
    digitalWrite(f1B, HIGH);
    //top left
    digitalWrite(f2A, LOW);
    digitalWrite(f2B, HIGH);
    //bottom right
    digitalWrite(b1A, LOW);
    digitalWrite(b1B, HIGH);
    //bottom left
    digitalWrite(b2A, LOW);
    digitalWrite(b2B, HIGH);
}
void mRight()
{
    //f2 back, f1 forward
    digitalWrite(f2A, LOW);
    digitalWrite(f2B, HIGH);
    digitalWrite(f1A, HIGH);
    digitalWrite(f1B, LOW);
    //b2 forward, b1, backward
    digitalWrite(b2A, HIGH);
    digitalWrite(b2B, LOW);
    digitalWrite(b1A, LOW);
    digitalWrite(b1B, HIGH);
}

void mLeft()
{
    //f2 forwards, f1 back
    digitalWrite(f2A, HIGH);
    digitalWrite(f2B, LOW);
    digitalWrite(f1A, LOW);
    digitalWrite(f1B, HIGH);
    //b2 backwards, b1 forward
    digitalWrite(b2A, LOW);
    digitalWrite(b2B, HIGH);
    digitalWrite(b1A, HIGH);
    digitalWrite(b1B, LOW);
}
void rLeft()
{
    digitalWrite(f2A, HIGH);
    digitalWrite(f2B, LOW);
    digitalWrite(b2A, HIGH);
    digitalWrite(b2B, LOW);
    digitalWrite(f1A, LOW);
    digitalWrite(f1B, HIGH);
    digitalWrite(b1A, LOW);
    digitalWrite(b1B, HIGH);
}
void rRight()
{
//Right Side Back
    digitalWrite(f1A, HIGH);
    digitalWrite(f1B, LOW);
    digitalWrite(b1A, HIGH);
    digitalWrite(b1B, LOW);
    //Left Side Forward
    digitalWrite(f2A, LOW);
    digitalWrite(f2B, HIGH);
    digitalWrite(b2A, LOW);
    digitalWrite(b2B, HIGH);
}
void stop()
{
    digitalWrite(f1A, HIGH);
    digitalWrite(f1B, HIGH);
    digitalWrite(f2A, HIGH);
    digitalWrite(f2B, HIGH);
    digitalWrite(b1A, HIGH);
    digitalWrite(b1B, HIGH);
    digitalWrite(b2A, HIGH);
    digitalWrite(b2B, HIGH);
}

bool findColorCenter(Mat& image, const Scalar& lowerBound, const Scalar& upperBound, Point& center)
{
    Mat hsv, mask;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    inRange(hsv, lowerBound, upperBound, mask);

    // Morphological operations to clean up the mask
   // morphologyEx(mask, mask, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
   // morphologyEx(mask, mask, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    
     // Morphological opening (remove small objects)
      erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
      dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

      // Morphological closing (fill small holes)
      dilate(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
      erode(mask, mask, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


    // Find the contours of the detected area
    vector<vector<Point>> contours;
    
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); //add hierarchy

    if (contours.empty()) return false; 

    double maxArea = 25;
    vector<Point> largestContour;
    for (const auto& contour : contours)
    {
        double area = contourArea(contour);
        if (area > maxArea)
        {
            maxArea = area;
            largestContour = contour;
        }
    }

    Moments m = moments(largestContour);
    center = Point(m.m10 / m.m00, m.m01 / m.m00);
    
    
    circle(image, center,5, Scalar(255, 0, 0), -1);
    return true;
}

// This function is taking care of the movement for the whole robot
/*
    It's finding the offset and based on how far it is from a certain range
    it rotates left or right 
*/ 

bool alignRobotWithTarget(const Point& frameCenter, const Point& targetCenter)
{
	
	// delay 
	delay(50);
    // get the offset of how far the object it
    Point offset = targetCenter - frameCenter;
    // get the offsetMagnitude by getting the object
    double offsetMagnitude = offset.x;
    // checking to see what the offseetMagnitude is
    cout << "Offset Magnitude: " << offsetMagnitude << endl;
    // checking to see the offset
    cout << "Offset for alignment: " << offset << endl;
    // if the OSM is less than -50 rotate left
    if(offsetMagnitude < -30)
	{

        rLeft();
        //delay(2000); //esteban
        cout << "Left" << endl;
        return false;
    }
    // else if the OSM is greater than 50 rotate right
    else if(offsetMagnitude > 30)
    {
        rRight();
       // delay(2000);//esteban
        cout << "Right" << endl;
        return false;
    }
    // else if it's in the middle stops and deley 
    else if (offsetMagnitude > -10 && offsetMagnitude < 10)
    {
        // stops the robot
        stop();
        delay(1000);
        cout << "Actually returning true" << endl;
        return true;
    }

    // rotating left
    //rLeft();
    cout << "the error is here"<< endl;
    return false;

}




// void overTake();
// {
//     if(ultrasonicSensor(3) >= 20)
//     {
//         // increase PWMS



//         if(ultrasonicSensor(2) >= 30)
//         {
//             // decreas PWMS
//         }

//     }
// }
