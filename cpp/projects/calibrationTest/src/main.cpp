///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-01
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "mainApplication.h"

int main(int _argc, char** _argv) {
	if (_argc < 2) {
		std::cerr << "Not enough input arguments" << std::endl;
	}

	MainApplication app(_argc, _argv);

	unsigned stepNumber = 0;
	while (app.step()) {
		cout << "<----------------------------- Step Number: " << stepNumber << " ----------------------------->" << endl;
	}

	system("PAUSE");
}
