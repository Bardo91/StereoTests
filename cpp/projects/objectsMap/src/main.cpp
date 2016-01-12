///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "mainApplication.h"
#include "utils/TimeTools.h"
#include "utils/gui/graph2d.h"

#include <StereoLib/utils/LogManager.h>

#include <opencv2/opencv.hpp>

bool continous = false;
double delay = 0;
void pauseCallback(int event, int x, int y, int, void*) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		continous = !continous;
		delay = continous ? 3 : 0;

		return;
	}
}

int main(int _argc, char** _argv) {
	if (_argc < 2) {
		std::cerr << "Not enough input arguments" << std::endl;
	}
	cv::namedWindow("Drone position", cv::WINDOW_FREERATIO);
	cv::setMouseCallback("Drone position", pauseCallback);
	
	MainApplication app(_argc, _argv);

	unsigned stepNumber = 0;
	while (true) {
		(*LogManager::get())["ConsoleOutput.txt"] << "<----------------------------- Step Number: " << stepNumber++ << " ----------------------------->" << endl;
		
		app.step();
		cv::waitKey(delay);
	}

	system("PAUSE");
}
