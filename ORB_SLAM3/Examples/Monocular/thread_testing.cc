#include<iostream>
#include <thread>         // std::thread
#include <opencv2/opencv.hpp> 


using namespace std;

void foo() 
{
    cv::Mat E = cv::Mat::eye(4, 4, CV_64F);
    cout << "E = " << endl << " " << E << endl << endl;
}

void bar(int x)
{
  cout << "Hello BAR " << x << endl;
}

int main() 
{
    std::thread first (foo);     // spawn new thread that calls foo()
    std::thread second (bar,0);  // spawn new thread that calls bar(0)
    
    // synchronize threads:
    first.join();                // pauses until first finishes
    second.join();               // pauses until second finishes

    std::cout << "foo and bar completed.\n";

    return 0;
}