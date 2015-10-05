///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//		Author:	Pablo Ramon Soria
//		Date:	2015-10-02
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <opencv2/opencv.hpp>
#include <string>
#include <ctime>

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

using namespace cv;
using namespace std;


//---------------------------------------------------------------------------------------------------------------------
int main(int _argc, char ** _argv) {
	VideoCapture camera1(0);
	VideoCapture camera2(1);
	
	Mat frame1, frame2;
	std::string folderName = "Sequence_" + to_string(time(NULL)) + "";
	do_mkdir(folderName);

	int index = 0;
	for (;;) {
		camera1 >> frame1;
		camera2 >> frame2;
		
		imwrite(folderName + "/cam1_img" + to_string(index) + ".jpg", frame1);
		imwrite(folderName + "/cam2_img" + to_string(index) + ".jpg", frame2);
		index++;
	}
}
