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
#include <StereoLib/EkfImuIcp.h>

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

Eigen::Vector3d extractGravity(ImuSensor *_imu, int _dataUsed);

void onlyAccTest(ImuSensor *_imu);
void accIcpTest(ImuSensor *_imu);

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
		BOViL::STime::get()->delay(1);
	}
	else if (std::string(_argv[1]) == "file") {
		imu = new ImuSimulatorSensor(_argv[2]);
	}
	else {
		std::cout << "Wrong input arguments" << std::endl;
		return -1;
	}

	BOViL::STime *timer = BOViL::STime::get();

	onlyAccTest(imu);
	
	//accIcpTest(imu);
}

Eigen::Vector3d extractGravity(ImuSensor * _imu, int _dataUsed) {
	Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
	ImuData imuData;
	// calculate offset.
	for (int i = 0;i < 500; i++) {
		imuData = _imu->get();

		Eigen::Quaternion<double> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
		Eigen::Vector3d linAcc, angSpeed;
		linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1], imuData.mLinearAcc[2];

		gravity += q*linAcc;
	}

	return gravity / 500;

}

void onlyAccTest(ImuSensor * _imu) {
	ImuData imuData;

	BOViL::plot::Graph2d posGraph("Position");
	BOViL::plot::Graph2d speedGraph("speed");
	BOViL::plot::Graph2d accGraph("acceleration");
	
	std::vector<std::vector<double>> pos(3), speed(3), acc(3);
	std::vector<double> vTime;

	double previousT;
	imuData = _imu->get();
	Eigen::Vector3d gravity = Eigen::Vector3d::Zero();
	
	// calculate offset.
	for (int i = 0;i < 200; i++) {
		imuData = _imu->get();

		Eigen::Quaternion<double> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
		Eigen::Vector3d linAcc, angSpeed;
		linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1], imuData.mLinearAcc[2];

		gravity += q*linAcc;
	}

	gravity = gravity / 200;



	imuData = _imu->get();
	Eigen::Quaternion<double> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
	Eigen::Vector3d linAcc, angSpeed;
	linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1], imuData.mLinearAcc[2];

	EkfImu ekf;
	Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12,12)*0.001;
	Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3,3)*0.001;
	Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(12,1);
	x0.block<3,1>(9,0) = q*linAcc - gravity;

	ekf.setUpEKF(Q, R, x0);
	ekf.parameters({ 0,0,0 }, {-1,-1,-1}, q*linAcc - gravity, {0.3,0.7,0.5});
	previousT=imuData.mTimeSpan;


	// Start
	for(int i = 0;i < 100; i++) {
		#if defined(_HAS_ROS_LIBRARIES_)
		ros::spinOnce();
		#endif
		
		imuData = _imu->get();
		vTime.push_back(imuData.mTimeSpan);
		Eigen::MatrixXd zk(6,1);

		Eigen::Quaternion<double> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
		Eigen::Vector3d linAcc, angSpeed;
		linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1], imuData.mLinearAcc[2];

		linAcc = q*linAcc - gravity;
		zk << linAcc;

		ekf.stepEKF(zk, imuData.mTimeSpan - previousT);
		previousT = imuData.mTimeSpan;

		auto rotMat = q.toRotationMatrix();
		auto rots = rotMat.eulerAngles(2,0,2);


		Eigen::MatrixXd state = ekf.getStateVector();
		// Plot;
		pos[0].push_back(state(0));
		pos[1].push_back(state(1));
		pos[2].push_back(state(2));
		speed[0].push_back(state(3));
		speed[1].push_back(state(4));
		speed[2].push_back(state(5));
		acc[0].push_back(state(6));
		acc[1].push_back(state(7));
		acc[2].push_back(state(8));
	}


	posGraph.clean();
	posGraph.draw(vTime,pos[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	posGraph.draw(vTime,pos[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	posGraph.draw(vTime,pos[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);

	speedGraph.clean();
	speedGraph.draw(vTime,speed[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	speedGraph.draw(vTime,speed[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	speedGraph.draw(vTime,speed[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);

	accGraph.clean();
	accGraph.draw(vTime,acc[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	accGraph.draw(vTime,acc[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	accGraph.draw(vTime,acc[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);

	cv::waitKey();
}

void accIcpTest(ImuSensor * _imu) {
	ImuData imuData;

	BOViL::plot::Graph2d posGraph("Position");
	BOViL::plot::Graph2d speedGraph("speed");
	BOViL::plot::Graph2d accGraph("acceleration");

	std::vector<std::vector<double>> pos(3), speed(3), acc(3);
	std::vector<double> vTime;

	double previousT;
	imuData = _imu->get();
	Eigen::Vector3d gravity = Eigen::Vector3d::Zero();

	// calculate offset.
	for (int i = 0;i < 500; i++) {
		imuData = _imu->get();

		Eigen::Quaternion<double> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
		Eigen::Vector3d linAcc, angSpeed;
		linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1], imuData.mLinearAcc[2];

		gravity += q*linAcc;
	}

	gravity = gravity / 500;



	imuData = _imu->get();
	Eigen::Quaternion<double> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
	Eigen::Vector3d linAcc, angSpeed;
	linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1], imuData.mLinearAcc[2];

	EkfImuIcp ekf;
	Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(12,12)*0.01;
	Eigen::MatrixXd R = Eigen::MatrixXd::Identity(6,6)*0.01;
	R.block<3,3>(0,0) = Eigen::MatrixXd::Identity(3,3)*0.1;
	Eigen::MatrixXd x0 = Eigen::MatrixXd::Zero(12,1);
	x0.block<3,1>(9,0) = q*linAcc - gravity;

	ekf.setUpEKF(Q, R, x0);
	ekf.parameters({ 0,0,0 }, {-1,-1,-1}, q*linAcc - gravity, {0.3,0.7,0.5});
	previousT=imuData.mTimeSpan;


	// Start
	for(int i = 0;i < 1800; i++) {
#if defined(_HAS_ROS_LIBRARIES_)
		ros::spinOnce();
#endif

		imuData = _imu->get();
		vTime.push_back(imuData.mTimeSpan);
		Eigen::MatrixXd zk(6,1);

		Eigen::Quaternion<double> q(imuData.mQuaternion[3], imuData.mQuaternion[0], imuData.mQuaternion[1], imuData.mQuaternion[2]);
		Eigen::Vector3d linAcc, linpos;
		linAcc << imuData.mLinearAcc[0],  imuData.mLinearAcc[1], imuData.mLinearAcc[2];
		linpos <<imuData.mPos3d[0], imuData.mPos3d[1],imuData.mPos3d[2];

		if (isnan(linpos[0])) {
			Eigen::Vector3d pPos(ekf.getStateVector());
			linpos = pPos;
		}

		linAcc = q*linAcc - gravity;
		zk << linpos, linAcc;

		ekf.stepEKF(zk, imuData.mTimeSpan - previousT);
		previousT = imuData.mTimeSpan;

		auto rotMat = q.toRotationMatrix();
		auto rots = rotMat.eulerAngles(2,0,2);


		Eigen::MatrixXd state = ekf.getStateVector();
		// Plot;
		pos[0].push_back(state(0));
		pos[1].push_back(state(1));
		pos[2].push_back(state(2));
		speed[0].push_back(state(3));
		speed[1].push_back(state(4));
		speed[2].push_back(state(5));
		acc[0].push_back(state(6));
		acc[1].push_back(state(7));
		acc[2].push_back(state(8));
	}


	posGraph.clean();
	posGraph.draw(vTime,pos[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	posGraph.draw(vTime,pos[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	posGraph.draw(vTime,pos[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);

	speedGraph.clean();
	speedGraph.draw(vTime,speed[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	speedGraph.draw(vTime,speed[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	speedGraph.draw(vTime,speed[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);

	accGraph.clean();
	accGraph.draw(vTime,acc[0], 255,0,0, BOViL::plot::Graph2d::eDrawType::Lines);
	accGraph.draw(vTime,acc[1], 0,255,0, BOViL::plot::Graph2d::eDrawType::Lines);
	accGraph.draw(vTime,acc[2], 0,0,255, BOViL::plot::Graph2d::eDrawType::Lines);

	std::ofstream facc("acc_938.txt");
	std::ofstream fspe("spe_938.txt");
	std::ofstream fpos("pos_938.txt");

	for (unsigned j = 0;j < vTime.size(); j++) {
		facc <<  acc[0][j] << "\t" << acc[1][j] << "\t" << acc[2][j] << std::endl;
		fspe <<  speed[0][j] << "\t" << speed[1][j] << "\t" << speed[2][j] << std::endl;
		fpos <<  pos[0][j] << "\t" << pos[1][j] << "\t" << pos[2][j] << std::endl;
	}

	cv::waitKey();
}
