#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/viz.hpp>
#include <fstream>

// Include VelodyneCapture Header
#include "VelodyneCapture.h"

//Added include paths for camera module
#include "FlyCapture2.h"
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace FlyCapture2;

int main( int argc, char* argv[] )
{
    // Open VelodyneCapture that retrieve from Sensor
     
    const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.201" );
    const unsigned short port = 2368;
    velodyne::VLP16Capture capture( address, port );
    char *binFileName = "../../../data/live_feed/velodyne_points/data/image.bin";
       
    if( !capture.isOpen() ){
        std::cerr << "Can't open VelodyneCapture." << std::endl;
        return -1;
    }

    // camera module code
    Error error;
    Camera camera;
    error = camera.Connect( 0 );
    if ( error != PGRERROR_OK )
    {
        std::cout << "Failed to connect to camera" << std::endl;
        return false;
    }
    error = camera.StartCapture();
   
    while(capture.isRun()){

	//Camera module code
        Image rawImage;
        Error error = camera.RetrieveBuffer( &rawImage );
        if ( error != PGRERROR_OK )
        {
                std::cout << "capture error" << std::endl;
                continue;
        }

        // convert to rgb
        Image rgbImage;
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rgbImage );
	
        // convert to OpenCV Mat
        unsigned int rowBytes = (double)rgbImage.GetReceivedDataSize()/(double)rgbImage.GetRows();    
        cv::Mat image = cv::Mat(rgbImage.GetRows(), rgbImage.GetCols(), CV_8UC3, rgbImage.GetData(),rowBytes);
	// Camera module code ends

    	//Declaring bin file to store the laser data
   	 std::ofstream binHandler (binFileName, std::ios::out | std::ios::binary);
	
        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;
        if( lasers.empty() ){	
            continue;
        }

        // Convert to 3-dimension Coordinates
	
        for( const velodyne::Laser& laser : lasers ){
	    float laser_mat[4];	    
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = laser.azimuth  * CV_PI / 180.0;
            const double vertical = laser.vertical * CV_PI / 180.0;
            const double intensity = static_cast<float>(laser.intensity);

            float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
            float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
            float z = static_cast<float>( ( distance * std::sin( vertical ) ) );
	    
	    laser_mat[0] = x;
	    laser_mat[1] = y;
	    laser_mat[2] = z;
	    laser_mat[3] = intensity;
	    binHandler.write((char *)laser_mat,4*sizeof(float));
        }
	binHandler.close();

	cv::Size size(960,604);
	cv::Mat dest;
	cv::resize(image,dest,size);
	cv::imwrite("../../../data/live_feed/image_01/data/image.png",dest);
        usleep(2000000);	
			
    }
    // Close All Viewers
    cv::destroyAllWindows();
    error = camera.StopCapture();
    camera.Disconnect();
    return 0;
}
