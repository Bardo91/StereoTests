///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	BOViL Plotting Tool.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "graph2d.h"

using namespace cv;
using namespace std;

namespace BOViL {
	namespace plot {
		//---------------------------------------------------------------------------------------------------------------------
		Graph2d::Graph2d(std::string _name) {
			mWindowName = _name;
			namedWindow(mWindowName, CV_WINDOW_FREERATIO);
			clean();
		}

		//---------------------------------------------------------------------------------------------------------------------
		void Graph2d::clean() {
			mGraphs.clear();
			cleanGraph();
		}

		//---------------------------------------------------------------------------------------------------------------------
		void Graph2d::show() {
			imshow(mWindowName, mLastRender);
		}

		//---------------------------------------------------------------------------------------------------------------------
		void Graph2d::draw(const std::vector<double>& _x, const std::vector<double>& _y, unsigned char _r, unsigned char _g, unsigned _b, eDrawType _type) {
			if (_x.size() != _y.size() || _x.size() == 0 || _y.size() == 0) {
				std::cerr << "Bad input arguments" << std::endl;
				return;
			}

			double minX = *min_element(_x.begin(), _x.end());
			double maxX = *max_element(_x.begin(), _x.end());
			double minY = *min_element(_y.begin(), _y.end());
			double maxY = *max_element(_y.begin(), _y.end());

			Graph graph = { _x, _y, Scalar(_r, _g, _b), _type };
			mGraphs.push_back(graph);

			if (minX < mMinX || maxX > mMaxX || minY < mMinY || maxY > mMaxY) {
				mMinX = minX < mMinX ? minX:mMinX;
				mMaxX = maxX > mMaxX ? maxX:mMaxX;
				mMinY = minY < mMinY ? minY:mMinY;
				mMaxY = maxY > mMaxY ? maxY:mMaxY;

				cleanGraph();
				drawAxis();
				// Redraw all graph
				for (Graph graph2 : mGraphs) {
					drawGraph(graph2);
				}

			}
			else {
				drawGraph(graph);
			}

			show();
		}

		//---------------------------------------------------------------------------------------------------------------------
		void Graph2d::draw(const std::vector<double>& _y, unsigned char _r, unsigned char _g, unsigned _b, eDrawType _type) {
			vector<double> x;
			for (unsigned i = 0; i < _y.size(); i++) {
				x.push_back(i);
			}
			draw(x,_y, _r,_g,_b, _type);
		}

		//---------------------------------------------------------------------------------------------------------------------
		void Graph2d::drawAxis() {
			double xStepD = (mWindowSize.width - cOffsetHorizontal*2) / cHorizontalDivisions;
			double yStepD = (mWindowSize.height - cOffsetVertical*2) / cVerticalDivisions;
			double xStep = (mMaxX - mMinX) / cHorizontalDivisions;
			double yStep = (mMaxY - mMinY) / cVerticalDivisions;

			// Draw horizontal axi
			Point2i p1(cOffsetHorizontal, mWindowSize.height - cOffsetVertical);
			Point2i p2(cOffsetHorizontal, mWindowSize.height - int(cOffsetVertical*0.75));
			for (unsigned i = 0; i <= cHorizontalDivisions; i++) {
				line(mLastRender, p1, p2, Scalar(0, 0, 0), 2);
				putText(mLastRender, to_string(mMinX + xStep*i).substr(0, 5), Point2i(p1.x - cOffsetHorizontal / 2, mWindowSize.height - int(cOffsetVertical*0.5)), CV_FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 0), 2);
				p1.x += int(xStepD);
				p2.x += int(xStepD);
			}

			// Draw vertical axi
			p1 = Point2i(int(cOffsetHorizontal*0.75), mWindowSize.height - cOffsetVertical);
			p2 = Point2i(cOffsetHorizontal, mWindowSize.height - cOffsetVertical);
			for (unsigned i = 0; i <= cHorizontalDivisions; i++) {
				line(mLastRender, p1, p2, Scalar(0, 0, 0), 2);
				putText(mLastRender, to_string(mMinY + yStep*i).substr(0, 5), Point2i(0, p2.y), CV_FONT_HERSHEY_PLAIN, 1.5, Scalar(0, 0, 0), 2);
				p1.y -= int(yStepD);
				p2.y -= int(yStepD);
			}
		}

		//---------------------------------------------------------------------------------------------------------------------
		void Graph2d::drawGraph(const Graph & _graph) {
			switch (_graph.mType) {
			case eDrawType::FilledCircles:
				drawPoints(_graph);
				break;
			case eDrawType::Lines:
				drawLines(_graph);
				break;
			}
		}

		//---------------------------------------------------------------------------------------------------------------------
		void Graph2d::drawPoints(const Graph &_graph) {
			for (unsigned i = 0; i < _graph.mX.size(); i++) {
				int x = int(cOffsetHorizontal + (_graph.mX[i] > mMinX ? _graph.mX[i] - mMinX : mMinX - _graph.mX[i])/(mMaxX-mMinX)*(mWindowSize.width - cOffsetHorizontal*2));
				int y = int((mWindowSize.height - cOffsetVertical) - (_graph.mY[i] > mMinY? _graph.mY[i] - mMinY:mMinY - _graph.mY[i])/(mMaxY-mMinY)*(mWindowSize.height - cOffsetVertical*2));
				Point2i point(x,y);
				circle(mLastRender, point, 5, _graph.mColor, CV_FILLED);
			}
		}

		//---------------------------------------------------------------------------------------------------------------------
		void Graph2d::drawLines(const Graph & _graph) {
			for (unsigned i = 0; i < _graph.mX.size()-1; i++) {
				int x1 = int(cOffsetHorizontal + (_graph.mX[i] > mMinX ? _graph.mX[i] - mMinX : mMinX - _graph.mX[i])/(mMaxX-mMinX)*(mWindowSize.width - cOffsetHorizontal*2));
				int y1 = int((mWindowSize.height - cOffsetVertical) - (_graph.mY[i] > mMinY? _graph.mY[i] - mMinY:mMinY - _graph.mY[i])/(mMaxY-mMinY)*(mWindowSize.height - cOffsetVertical*2));
				Point2i p1(x1,y1);

				int x2 = int(cOffsetHorizontal + (_graph.mX[i+1] > mMinX ? _graph.mX[i+1] - mMinX : mMinX - _graph.mX[i+1])/(mMaxX-mMinX)*(mWindowSize.width - cOffsetHorizontal*2));
				int y2 = int((mWindowSize.height - cOffsetVertical) - (_graph.mY[i+1] > mMinY? _graph.mY[i+1] - mMinY:mMinY - _graph.mY[i+1])/(mMaxY-mMinY)*(mWindowSize.height - cOffsetVertical*2));
				Point2i p2(x2,y2);

				line(mLastRender, p1, p2, _graph.mColor, 2);
			}
		}

		//---------------------------------------------------------------------------------------------------------------------
		void Graph2d::cleanGraph() {
			mLastRender = Mat(mWindowSize, CV_8UC3, Scalar(150, 150, 150));
			rectangle(mLastRender, Point2i(cOffsetHorizontal, cOffsetVertical), Point2i(mWindowSize.width - cOffsetHorizontal, mWindowSize.height - cOffsetVertical), Scalar(255, 255, 255), CV_FILLED);
			rectangle(mLastRender, Point2i(cOffsetHorizontal, cOffsetVertical), Point2i(mWindowSize.width - cOffsetHorizontal, mWindowSize.height - cOffsetVertical), Scalar(0, 0, 0), 2);
		}
	}	// namespace plot
}	// namespace BOViL