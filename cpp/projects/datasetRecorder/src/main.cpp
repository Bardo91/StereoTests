///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-11-26
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <core/time/time.h>
#include <implementations/sensors/MavrosSensor.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <thread> 
#include <mutex>


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
volatile bool running = true;
std::mutex mutex;

int main(int _argc, char ** _argv) {
	BOViL::STime::init();

	// Start a thread to stop other ones.
	std::thread stopThread([&]() {
		int cmd = 1;
		while(cmd != 0) {
			std::cout << "If you want to close, enter 0" << std::endl;
			std::cin >> cmd;
		}
		running = false;
	});

	std::string folderName = "experiment_"+std::to_string(time(NULL))+"/";
	do_mkdir(folderName);

	// Start a thread for capturing imu.
	std::thread imuThread([&]() {
		MavrosSensor imu;
		BOViL::STime *timer = BOViL::STime::get();
		ImuData imuData;

		std::ofstream file(folderName+"imudata.txt");

		while (running) {
			double t = timer->getTime();
			imuData = imu.get();

			file << t << ", " <<
					imuData.mAltitude << ", " <<
					imuData.mEulerAngles[0] << ", " <<
					imuData.mEulerAngles[1] << ", " <<
					imuData.mEulerAngles[2] << ", " <<
					imuData.mLinearSpeed[0] << ", " <<
					imuData.mLinearSpeed[1] << ", " <<
					imuData.mLinearSpeed[2] << ", " <<
					imuData.mLinearAcc[0] << ", " <<
					imuData.mLinearAcc[1] << ", " <<
					imuData.mLinearAcc[2] << ", " <<
					imuData.mAngularSpeed[0] << ", " <<
					imuData.mAngularSpeed[1] << ", " <<
					imuData.mAngularSpeed[2] << ", " <<
					imuData.mAngularAcc[0] << ", " <<
					imuData.mAngularAcc[1] << ", " <<
					imuData.mAngularAcc[2] << std::endl;
		}

	});


	// Start a thread for capturing images
	std::thread frameThread([&]() {
		cv::VideoCapture cam1(0);
		cv::VideoCapture cam2(1);
		BOViL::STime *timer = BOViL::STime::get();
		cv::Mat frame1, frame2;	
		
		std::ofstream timeSpan(folderName+"timespan.txt");
		int index = 0;
		while (running) {
			double t = timer->getTime();
			cam1 >> frame1;
			cam2 >> frame2;

			timeSpan << t << std::endl;
			cv::imwrite(folderName+ "cam1_"+std::to_string(index)+".jpg", frame1);
			cv::imwrite(folderName+ "cam2_"+std::to_string(index)+".jpg", frame2);
			index++;
		}
	});

	BOViL::STime *timer = BOViL::STime::get();
	while (running) {
		timer->mDelay(100);
	}

	timer->delay(1);

	if(stopThread.joinable())
		stopThread.join();

	if(imuThread.joinable())
		imuThread.join();

	if(frameThread.joinable())
		frameThread.join();

}


