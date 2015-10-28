//
//
//
//
//

#include "StereoCameras.h"

#ifndef MAINAPPLICATION_H_
#define MAINAPPLICATION_H_


class MainApplication {
public:
	bool init	(int _argc, char** _argv);
	bool step	();
private:
	bool loadArguments	(int _argc, char** _argv);
	bool initGui		();
	bool initCameras	();
	bool init3dMap		();

	void stepSearchPointsOnImage();
	void stepUpdateMap();
	void stepGetCandidates();

private:
	StereoCameras mCameras;


};

#endif	//	MAINAPPLICATION_H_