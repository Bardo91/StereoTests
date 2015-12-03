///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-12-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <core/time/time.h>
#include <implementations/sensors/MavrosSensor.h>
#include <implementations/sensors/ImuSimulatorSensor.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <thread> 
#include <mutex>
#include <string>

#include "../../objectsMap/src/utils/gui/graph2d.h"
#include "EkfImu.h"

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

	if (_argc < 2) {
		std::cout << "Not enough input arguments" << std::endl;
		return -1;
	}

	// Initialization
	#if defined(_HAS_ROS_LIBRARIES_)
	std::cout <<  "Initializing ros" << std::endl;
	ros::init(_argc, _argv, "Dataset_Recorder");
	#endif

	ImuSensor *imu;

	if (std::string(_argv[1]) == "real") {
		imu = new MavrosSensor;
	}
	else if (std::string(_argv[1]) == "file") {
		imu = new ImuSimulatorSensor(_argv[2]);
	}
	else {
		std::cout << "Wrong input arguments" << std::endl;
		return -1;
	}

	BOViL::STime *timer = BOViL::STime::get();
	ImuData imuData;

	timer->delay(1);

	EkfImu ekf;
	Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(15,15)*0.001;
	Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6,6)*0.001;
	Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(15,1);

	BOViL::plot::Graph2d posGraph("Position");
	BOViL::plot::Graph2d speedGraph("speed");
	BOViL::plot::Graph2d accGraph("acceleration");
	BOViL::plot::Graph2d angleGraph("angle");
	BOViL::plot::Graph2d angularSpeedGraph("angularSpeed");
	BOViL::plot::Graph2d angleImuGraph("AngleImu");
	
	std::vector<std::vector<double>> pos(3), speed(3), acc(3), angle(3), angularSpeed(3), angleImu(3);

	ekf.setUpEKF(Q, R, x0);

	double previousT;
	bool isFirst = true;
	//cv::waitKey();
	// Start
	for(int i = 0;i <7200; i++) {
		#if defined(_HAS_ROS_LIBRARIES_)
		ros::spinOnce();
		#endif
		double t = timer->getTime();
		imuData = imu->get();


		if (isFirst) {
			previousT = imuData.mTimeSpan - 0.01;
			isFirst = false;
			continue;
		}
		Eigen::MatrixXd zk(6,1);

		Eigen::Quaternion<double> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
		Eigen::Vector3d linAcc, angSpeed, gravity;
		linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1],imuData.mLinearAcc[2];
		angSpeed << imuData.mAngularSpeed[0], imuData.mAngularSpeed[1], imuData.mAngularSpeed[2];

		gravity << 0,0,9.81;
		linAcc = q*linAcc - gravity;
		angSpeed = q*angSpeed;

		zk << linAcc, angSpeed;

		ekf.stepEKF(zk, imuData.mTimeSpan - previousT);
		previousT = imuData.mTimeSpan;

		auto rotMat = q.toRotationMatrix();
		auto rots = rotMat.eulerAngles(2,0,2);


		Eigen::MatrixXd state = ekf.getStateVector();
		// Plot;
		angleImu[0].push_back(rots[0]);
		angleImu[1].push_back(rots[1]);
		angleImu[2].push_back(rots[2]);
		pos[0].push_back(state(0));
		pos[1].push_back(state(1));
		pos[2].push_back(state(2));
		speed[0].push_back(state(3));
		speed[1].push_back(state(4));
		speed[2].push_back(state(5));
		acc[0].push_back(state(6));
		acc[1].push_back(state(7));
		acc[2].push_back(state(8));
		angle[0].push_back(state(9));
		angle[1].push_back(state(10));
		angle[2].push_back(state(11));
		angularSpeed[0].push_back(state(12));
		angularSpeed[1].push_back(state(13));
		angularSpeed[2].push_back(state(14));

	
	}

	
	posGraph.clean();
	posGraph.draw(pos[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	posGraph.draw(pos[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	posGraph.draw(pos[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);
	
	speedGraph.clean();
	speedGraph.draw(speed[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	speedGraph.draw(speed[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	speedGraph.draw(speed[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);
	
	accGraph.clean();
	accGraph.draw(acc[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	accGraph.draw(acc[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	accGraph.draw(acc[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);
	
	angleGraph.clean();
	angleGraph.draw(angle[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	angleGraph.draw(angle[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	angleGraph.draw(angle[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);

	angularSpeedGraph.clean();
	angularSpeedGraph.draw(angularSpeed[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	angularSpeedGraph.draw(angularSpeed[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	angularSpeedGraph.draw(angularSpeed[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);

	angleImuGraph.clean();
	angleImuGraph.draw(angleImu[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	angleImuGraph.draw(angleImu[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	angleImuGraph.draw(angleImu[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);

	cv::waitKey();

}


