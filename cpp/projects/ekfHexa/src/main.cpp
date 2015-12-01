///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-12-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <core/time/time.h>
#include <implementations/sensors/MavrosSensor.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <thread> 
#include <mutex>
#include <string>

//---------------------------------------------------------------------------------------------------------------------
#ifdef _WIN32
	#include <Windows.h>
	inline void do_mkdir(std::string _filename) {
		CreateDirectory(_filename.c_str(), NULL);
	}
#elif __linux__
	#include <sys/stat.h>
	#include <sys/types.h>
	inline void do_mkdir(std::string _filename) {
		mkdir(_filename.c_str(), 0700);
	}
#endif


//---------------------------------------------------------------------------------------------------------------------


int main(int _argc, char ** _argv) {
	#if defined(_HAS_ROS_LIBRARIES_)
	std::cout <<  "Initializing ros" << std::endl;
	ros::init(_argc, _argv, "Dataset_Recorder");
	#endif
	MavrosSensor imu;
	BOViL::STime *timer = BOViL::STime::get();
	ImuData imuData;

	timer->delay(1);

	for(;;) {
		#if defined(_HAS_ROS_LIBRARIES_)
		ros::spinOnce();
		#endif
		double t = timer->getTime();
		imuData = imu.get();
	}

	


}


