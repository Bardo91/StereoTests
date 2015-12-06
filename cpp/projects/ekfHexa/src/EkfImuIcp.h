///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-12-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef EKFIMUICP_H_
#define EKFIMUICP_H_


#include <algorithms/state_estimators/ExtendedKalmanFilter.h>


// Estimation of th drone position using Acc and gyro.
//
// State vector Xk = {x, y, z, vx, vy, vz, ax, ay, az, bax, bay, baz}
//
//
//
class EkfImuIcp : public BOViL::algorithms::ExtendedKalmanFilter {
public:
	void parameters(Eigen::Vector3d _scaleFactor, Eigen::Vector3d _c1, Eigen::Vector3d _c2, Eigen::Vector3d _t);

protected:
	void updateJf(const double _incT);
	void updateHZk();
	void updateJh();

private:
	Eigen::Vector3d mScaleFactor, mC1, mC2, mT;
};


#endif	//	EKFIMU_H_