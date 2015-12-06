///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-12-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#include "EkfImuIcp.h"

void EkfImuIcp::parameters(Eigen::Vector3d _scaleFactor, Eigen::Vector3d _c1, Eigen::Vector3d _c2, Eigen::Vector3d _t) {
	mScaleFactor = _scaleFactor;
	mC1 = _c1;
	mC2 = _c2;
	mT = _t;

}

//---------------------------------------------------------------------------------------------------------------------
double sign(double _var) {
	return _var < 0? -1:1;
}

void EkfImuIcp::updateJf(const double _incT) {
	mJf = Eigen::MatrixXd::Identity(mJf.rows(), mJf.cols());
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	mJf.block<3, 3>(0, 3) =I*_incT;
	mJf.block<3, 3>(0, 6) =I*_incT*_incT/2;
	mJf.block<3, 3>(3, 6) =I*_incT;
	
	Eigen::Vector3d auxAcc({sign(mXfk(6,0)*mScaleFactor[0]), sign(mXfk(6,0)*mScaleFactor[1]), sign(mXfk(6,0)*mScaleFactor[2])});
	Eigen::Matrix3d auxMatAcc = (auxAcc*Eigen::Matrix<double,1,3>({1,1,1})).cwiseProduct(I);;
	mJf.block<3, 3>(6, 6) += auxMatAcc;

	mJf.block<3, 3>(6, 9) =-I;
	
	Eigen::Matrix3d diagT = (mT*Eigen::Matrix<double,1,3>({1,1,1})).cwiseProduct(I);;
	Eigen::Matrix3d diagTpluxTime = (diagT + I*_incT);
	mJf(9, 9) = diagT(0,0)/diagTpluxTime(0,0);
	mJf(10, 10) = diagT(1,1)/diagTpluxTime(1,1);
	mJf(11, 11) = diagT(2,2)/diagTpluxTime(2,2);
}

//---------------------------------------------------------------------------------------------------------------------
void EkfImuIcp::updateHZk() {
	mHZk.block<3,1>(0,0) = mXfk.block<3,1>(0,0);
	mHZk.block<3,1>(3,0) = mXfk.block<3,1>(6,0);
}

//---------------------------------------------------------------------------------------------------------------------
void EkfImuIcp::updateJh() {
	mJh(0,0) = 1;
	mJh(1,1) = 1;
	mJh(2,2) = 1;
	mJh(3,6) = 1;
	mJh(4,7) = 1;
	mJh(5,8) = 1;
}
