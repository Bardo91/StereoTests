///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Single object detection - ObjectDetector
//		Author: Pablo R.S.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#ifndef _BOVIL_ALGORITHMS_MACHINELEARNING_TOPICMODELING_CORPUS_H_
#define _BOVIL_ALGORITHMS_MACHINELEARNING_TOPICMODELING_CORPUS_H_

#include "Document.h"

#include <vector>
namespace BOViL {
	namespace algorithms {
		class Corpus {
		public:
			/// Add a document to the corpus
			/// \param _document: document to be added to the corpus
			void addDocument(Document _document) { mDocuments.push_back(_document); };

			/// Get count of total words in document.
			/// \return Sum of all words in all documents
			unsigned totalWords() {
				unsigned numWords = 0;
				for (Document doc : mDocuments) {
					numWords += doc.lenght();
				}
				return numWords;
			};

			/// Get number of documents in the corpus
			/// \return Number of documents in the corpus
			unsigned numDocs() { return mDocuments.size(); };

			/// Access to a document
			/// \param index to the desired document.
			Document document(unsigned _index) { return mDocuments[_index]; };

			/// Shuffle word list.
			void shuffle(){ std::random_shuffle(mDocuments.begin(), mDocuments.end()); };

		private:
			std::vector<Document> mDocuments;
		};	//	class Corpus

	}	//	namespace algorithm
}	//	namespace BOViL

#endif	//	_BOVIL_ALGORITHMS_MACHINELEARNING_TOPICMODELING_CORPUS_H_