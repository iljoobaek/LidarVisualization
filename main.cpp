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
using namespace FlyCapture2;

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

int main( int argc, char* argv[] )
{
    // Open VelodyneCapture that retrieve from Sensor
     
    const boost::asio::ip::address address = boost::asio::ip::address::from_string( "192.168.1.201" );
    const unsigned short port = 2368;
    velodyne::VLP16Capture capture( address, port );

    
    int save_flag =1;
    char *pythonIntrepreter="/usr/bin/python3"; 
    char *ipbasicPython="/home/lidarwb/workbench/git_repo/ip_basic/demos/depth_completion.py";
    char *ProjectionPython="/home/lidarwb/workbench/project2D_lidar_to_cam/to_depth.py"; 
    char *generatePCLPython="/home/lidarwb/workbench/project2D_lidar_to_cam/generate_pointcloud.py"; 
    char *pythonArgs[]={pythonIntrepreter,ProjectionPython,NULL};
    char *pythonArgs2[]={pythonIntrepreter,ipbasicPython,NULL};
    
    
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


    std::ofstream binHandler ("/home/lidarwb/workbench/project2D_lidar_to_cam/synced/live_feed/velodyne_points/data/image.bin", std::ios::out | std::ios::binary);

    
    while( capture.isRun()){

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
	
        // Capture One Rotation Data
        std::vector<velodyne::Laser> lasers;
        capture >> lasers;
        if( lasers.empty() ){
	
            continue;
        }

        // Convert to 3-dimention Coordinates

		auto start = std::chrono::high_resolution_clock::now();
	
        for( const velodyne::Laser& laser : lasers ){
	    float laser_mat[4];	
	    
	    
            const double distance = static_cast<double>( laser.distance );
            const double azimuth  = laser.azimuth  * CV_PI / 180.0;
            const double vertical = laser.vertical * CV_PI / 180.0;
			const double intensity = static_cast<float>laser.intensity;

            float x = static_cast<float>( ( distance * std::cos( vertical ) ) * std::sin( azimuth ) );
            float y = static_cast<float>( ( distance * std::cos( vertical ) ) * std::cos( azimuth ) );
            float z = static_cast<float>( ( distance * std::sin( vertical ) ) );
			
			laser_mat[0] = x;
			laser_mat[1]= y;
			laser_mat[2] = z;
			laser_mat[3] = intensity;

			binHandler.write((char *)laser_mat,4*sizeof(float));   
        }
	
		binHandler.close();
		auto elapsed = std::chrono::high_resolution_clock::now() - start;
		long long microseconds = std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count();
		std::cout<<"Time taken for bin file writing for one complete rotation data" << microseconds << std::endl;
		std::cout<<"Closing Camera capture" <<std::endl;
		capture.close();
		std::cout<<"Closed" <<std::endl;
	/*
		cv::Size size(960,604);
		cv::Mat dest;
		cv::resize(image,dest,size);*/
		cv::imwrite("/home/lidarwb/workbench/project2D_lidar_to_cam/synced/live_feed/image_01/data/image.png",image);

		if(save_flag)
		{
			save_flag = 1;
			std::cout<<"Forking python children" <<std::endl;
			int pid = fork();
			if(pid==0)
			{
				execvp(pythonIntrepreter,pythonArgs);
				exit(0);
			}
			std::cout<<"Waiting for child1" <<std::endl;
			waitpid(pid,NULL,0);
			std::cout<<"Done waiting" <<std::endl;
			
			int pid2 = fork();
			if(pid2==0)
			{
				execvp(pythonIntrepreter,pythonArgs2);
				exit(0);
			}
			std::cout<<"Waiting for child2" <<std::endl;
			waitpid(pid2,NULL,WNOHANG);
			std::cout<<"Done waiting" <<std::endl;
		
			int pid3 = fork();
			if(pid3==0)
			{
				char *pythonArgs3[]={pythonIntrepreter,generatePCLPython,"/home/lidarwb/workbench/project2D_lidar_to_cam/synced/live_feed/image_01/data/image.png","/home/lidarwb/workbench/project2D_lidar_to_cam/synced/live_feed/depth_completed/depth.png","fusion.ply"	};
				std::cout<<"Executing point cloud generator child"<<std::endl;
				execvp(pythonIntrepreter,pythonArgs3);
				exit(0);
			}
			waitpid(pid3,NULL,0);
			break;
						
		}	
	
    }

    // Close All Viewers
    cv::destroyAllWindows();
    error = camera.StopCapture();
    if ( error != PGRERROR_OK )
    {
        // This may fail when the camera was removed, so don't show 
        // an error message
    }

    camera.Disconnect();
    return 0;
}
