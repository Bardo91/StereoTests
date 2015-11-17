///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Single object detection - ObjectDetector
//		Author: Pablo R.S.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _BOVIL_ALGORITHMS_MACHINELEARNING_TOPICMODELING_DOCUMENTS_H_
#define _BOVIL_ALGORITHMS_MACHINELEARNING_TOPICMODELING_DOCUMENTS_H_

#include <vector>
#include <algorithm>

namespace BOViL {
	namespace algorithms {
		class Document {
		public:
			/// Default constructor;
			Document() {};

			/// Construct a document with a list of words
			Document(std::vector<unsigned> _words) : mWords(_words) {};

			/// Add new word to document
			void addWord(unsigned _word) { mWords.push_back(_word); };

			/// Shuffle word list.
			void shuffle() { std::random_shuffle(mWords.begin(), mWords.end()); };

			/// Get a copy of word list
			std::vector<unsigned>	words()	const { return mWords; };

			/// Access to a single word
			int word(unsigned _index) const { return mWords[_index]; };

			/// Get number of words in document.
			unsigned			lenght() { return mWords.size(); };
		private:
			std::vector<unsigned> mWords;
		};	// class Document

	}	//	namespace algorithm
}	//	namespace BOViL.
#endif	//	_BOVIL_ALGORITHMS_MACHINELEARNING_TOPICMODELING_DOCUMENTS_H_