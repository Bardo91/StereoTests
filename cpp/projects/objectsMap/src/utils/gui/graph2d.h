///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	BOViL Plotting Tool.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef GRAPH2D_H_
#define GRAPH2D_H_

#include <string>
#include <opencv2/opencv.hpp>

namespace BOViL {
	namespace plot {
		class Graph2d {
		public:		// Public interface
			enum eDrawType	{Points = 0, Lines, Circles, FilledCircles, Cross};

			struct Graph {
				std::vector<double> mX;
				std::vector<double> mY;
				cv::Scalar mColor;
				eDrawType mType;
			};

			/// Create a new instance of graph
			/// \param _name: Name to be displayed on the figure.
			Graph2d(std::string _name);

			/// Clean graph
			void clean();

			/// Show last render
			void show();
			
			/// Draw new data
			void draw(const std::vector<double> &_x, const std::vector<double> &_y, unsigned char _r, unsigned char _g, unsigned _b, eDrawType _type = eDrawType::FilledCircles);
			void draw(const std::vector<double> &_y, unsigned char _r, unsigned char _g, unsigned _b, eDrawType _type = eDrawType::Circles);

		private:	// Private methods
			void drawAxis();
			void drawGraph(const Graph &_graph);
			void drawPoints(const Graph &_graph);
			void drawLines(const Graph &_graph);

			void cleanGraph();

		private:	// Members

			std::string			mWindowName;
			cv::Size2i			mWindowSize			= {1200, 1200};	// Fixed resolution by now.
			const unsigned		cOffsetHorizontal	= 100;			// Fixed by now.
			const unsigned		cOffsetVertical		= 100;			// Fixed by now.
			const unsigned		cHorizontalDivisions= 10;			// Fixed by now.
			const unsigned		cVerticalDivisions	= 10;			// Fixed by now.

			cv::Mat				mLastRender;
			std::vector<Graph>	mGraphs;

			double mMinX=0, mMinY=0, mMaxX = 1, mMaxY = 0.0001;

		};	// class Graph2d.

	}	//	namespace plot.
}	//	namespace BOViL.

#endif	//	GRAPH2D_H_

