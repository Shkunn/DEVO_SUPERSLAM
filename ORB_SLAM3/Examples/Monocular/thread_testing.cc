#include<iostream>
#include <thread>         // std::thread
#include <opencv2/opencv.hpp> 

using namespace std;

cv::Mat foo(cv::Mat E) 
{
    // cv::Mat E = cv::Mat::eye(4, 4, CV_64F);
    E += 1;
    // cout << "E = " << endl << " " << E << endl << endl;
    return E;
}

void bar(int count)
{
    count += 1;
    cout << "Hello BAR " << count << endl;
}

int main() 
{
    cv::Mat E = cv::Mat::eye(4, 4, CV_64F);
    int count = 0;
    while(1)
    {
        std::thread first (&foo, E);           // spawn new thread that calls foo()
        std::thread second (&bar, count);   // spawn new thread that calls bar(0)
        
        // synchronize threads:
        first.join();                         // pauses until first finishes
        second.join();                        // pauses until second finishes
        
        cout << "E = " << endl << " " << E << endl << endl;

        E.setTo(0, E > 1000);
    }
    

    return 0;
}