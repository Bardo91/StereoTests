///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "mainApplication.h"
#include "TimeTools.h"
#include "graph2d.h"


#include <opencv2/opencv.hpp>

int main(int _argc, char** _argv) {
	if (_argc < 2) {
		std::cerr << "Not enough input arguments" << std::endl;
	}

	BOViL::STime * timer = BOViL::STime::get();

	MainApplication app(_argc, _argv);

	unsigned stepNumber = 0;
	std::vector<double> timeArray;
	BOViL::plot::Graph2d timePlot("Global Time");
	while (true) {
		cout << "<----------------------------- Step Number: " << stepNumber++ << " ----------------------------->" << endl;
		double t0 = timer->getTime();
		app.step();
		double t1 = timer->getTime();
		timeArray.push_back(t1-t0);
		timePlot.clean();
		timePlot.draw(timeArray,255,0,0,BOViL::plot::Graph2d::Lines);
		timePlot.show();
		cv::waitKey();
	}

	system("PAUSE");
}
