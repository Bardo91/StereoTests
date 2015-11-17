///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//	Single object detection - ObjectDetector
//		Author: Pablo R.S.
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "lda.h"
#include "Corpus.h"

#include <cassert>
#include <ctime>
#include <iostream>
#include <fstream>
#include <numeric>
using namespace std;

namespace BOViL {
	namespace algorithms {
		//-----------------------------------------------------------------------------------------------------------------
		void LDA::train(Corpus _corpus, unsigned _nTopics, unsigned _vocabSize, double _alpha, double _beta, unsigned _iterations) {
			srand((unsigned)time(NULL));	// Seed random number generator.
			mVocabSize = _vocabSize;
			mAlpha = _alpha;
			mBeta = _beta;
			mNumTopics = _nTopics;

			initVariables(_corpus);
			initCount(_corpus);

			// run sampler for 'iterations' times
			for (unsigned it = 0; it < _iterations; it++) {
				// for every word in every document (i.e. z)...
				for (unsigned m = 0; m < _corpus.numDocs(); m++) {
					unsigned l = _corpus.document(m).lenght();
					for (unsigned n = 0; n < l; n++) {
						mZ[m][n] = sampleZ(m, n, _corpus.document(m).word(n));
					}
				}
			}

			// PHI matrix
			mPhi.resize(_nTopics);
			for (unsigned k = 0; k < _nTopics; k++) {
				mPhi[k].resize(_vocabSize);
				for (unsigned w = 0; w < _vocabSize; w++) {
					mPhi[k][w] = double(mNwz[w][k] + _beta) / (mNz[k] + (_vocabSize*_beta));
				}
			}

			// THETA matrix
			mTheta.resize(_corpus.numDocs());
			for (unsigned m = 0; m < _corpus.numDocs(); m++) {
				mTheta[m].resize(_nTopics);
				for (unsigned k = 0; k < _nTopics; k++) {
					mTheta[m][k] = double(mNzm[k][m] + _alpha) / (_corpus.document(m).lenght() + (_alpha*_nTopics));
				}
			}

		}

		//-----------------------------------------------------------------------------------------------------------------
		std::vector<std::vector<double>> LDA::evaluate(Corpus _corpus, unsigned _nIters) {
			vector<vector<double>> corpusProbs(_corpus.numDocs());

			// Increase size of counters and topic assignement
			for (unsigned d = 0; d < _corpus.numDocs(); d++) {
				for (unsigned k = 0; k < mNumTopics; k++) {
					mNzm[k].push_back(0);
				}

				mZ.push_back(vector<unsigned>(_corpus.document(d).lenght()));
			}

			// Add new words count
			unsigned numNewDocs = _corpus.numDocs();
			for (unsigned d = 0; d < numNewDocs; d++) {
				for (unsigned n = 0; n < _corpus.document(d).lenght(); n++) {
					// for now we'll initialize with a RANDOM topic.
					// Later, let the user decide if it's random or null (-1)
					unsigned topic = rand() % mNumTopics;

					mZ[(mZ.size() - numNewDocs) + d][n] = topic;

					// now set the counts
					mNwz[_corpus.document(d).word(n)][topic]++;
					mNzm[topic][(mZ.size() - numNewDocs) + d]++;
					mNz[topic]++;
				}
			}

			// Iterate new data.
			// run sampler for 'iterations' times
			for (unsigned it = 0; it < _nIters; it++) {
				// for every word in every document (i.e. z)...
				for (unsigned d = 0; d < numNewDocs; d++) {
					for (unsigned n = 0; n < mZ[(mZ.size() - numNewDocs) + d].size(); n++) {
						mZ[(mZ.size() - numNewDocs) + d][n] = sampleZ((mZ.size() - numNewDocs) + d, n, _corpus.document(d).word(n));
					}
				}
			}

			// Update model parameters
			// PHI matrix
			mPhi.resize(mNumTopics);
			for (unsigned k = 0; k < mNumTopics; k++) {
				mPhi[k].resize(mVocabSize);
				for (unsigned w = 0; w < mVocabSize; w++) {
					mPhi[k][w] = double(mNwz[w][k] + mBeta) / (mNz[k] + (mVocabSize*mBeta));
				}
			}

			// THETA matrix
			mTheta.resize(mZ.size());
			for (unsigned m = 0; m < mZ.size(); m++) {
				mTheta[m].resize(mNumTopics);
				for (unsigned k = 0; k < mNumTopics; k++) {
					mTheta[m][k] = double(mNzm[k][m] + mBeta) / (mZ[m].size() + (mAlpha*mNumTopics));
				}
			}

			// Returning probabilities
			for (unsigned d = 0; d < numNewDocs; d++) {
				corpusProbs[d].resize(mNumTopics);
				for (unsigned t = 0; t < mNumTopics; t++) {
					corpusProbs[d][t] = mTheta[(mZ.size() - numNewDocs) + d][t];
				}
			}

			return corpusProbs;
		}

		//-----------------------------------------------------------------------------------------------------------------
		void LDA::save(std::string _path) {
			ofstream modelFile(_path);
			assert(modelFile.is_open());

			// Save matrix sizes.
			modelFile << mPhi.size() << std::endl;
			modelFile << mPhi[0].size() << std::endl;
			modelFile << mTheta.size() << std::endl;
			modelFile << mTheta[0].size() << std::endl;

			// Save Phi
			for (unsigned i = 0; i < mPhi.size(); i++) {
				for (unsigned j = 0; j < mPhi[i].size(); j++) {
					modelFile << mPhi[i][j] << std::endl;
				}
			}

			// Save Theta
			for (unsigned i = 0; i < mTheta.size(); i++) {
				for (unsigned j = 0; j < mTheta[i].size(); j++) {
					modelFile << mTheta[i][j] << std::endl;
				}
			}

			modelFile << mVocabSize << std::endl;
			modelFile << mNumTopics << std::endl;
			modelFile << mAlpha << std::endl;
			modelFile << mBeta << std::endl;

			modelFile <<  mNwz.size() << std::endl;
			for (unsigned i = 0; i < mNwz.size(); i++) {
				modelFile <<  mNwz[i].size() << std::endl;
				for (unsigned j = 0; j < mNwz[i].size(); j++) {
					modelFile << mNwz[i][j] << std::endl;
				}
			}

			modelFile << mNzm.size() << std::endl;
			for (unsigned i = 0; i < mNzm.size(); i++) {
				modelFile << mNzm[i].size() << std::endl;
				for (unsigned j = 0; j < mNzm[i].size(); j++) {
					modelFile << mNzm[i][j] << std::endl;
				}
			}

			modelFile << mNz.size() << std::endl;
			for (unsigned i = 0; i < mNz.size(); i++) {
				modelFile << mNz[i] << std::endl;
			}

			modelFile << mNwSum.size() << std::endl;
			for (unsigned i = 0; i < mNwSum.size(); i++) {
				modelFile << mNwSum[i] << std::endl;
			}

			modelFile << mZ.size() << std::endl;
			for (unsigned i = 0; i < mZ.size(); i++) {
				modelFile << mZ[i].size() << std::endl;
				for (unsigned j = 0; j < mZ[i].size(); j++) {
					modelFile << mZ[i][j] << std::endl;
				}
			}
		}

		//-----------------------------------------------------------------------------------------------------------------
		void LDA::load(std::string _path) {
			ifstream modelFile(_path);
			assert(modelFile.is_open());

			// Load matrix sizes.
			unsigned pRows, pCols, tRows, tCols;
			modelFile >> pRows;
			modelFile >> pCols;

			modelFile >> tRows;
			modelFile >> tCols;

			mNumTopics = pRows;
			mVocabSize = pCols;

			// Load Phi
			mPhi.resize(pRows);
			for (unsigned i = 0; i < mPhi.size(); i++) {
				mPhi[i].resize(pCols);
				for (unsigned j = 0; j < mPhi[i].size(); j++) {
					modelFile >> mPhi[i][j];
				}
			}

			// Load Theta
			mTheta.resize(tRows);
			for (unsigned i = 0; i < mTheta.size(); i++) {
				mTheta[i].resize(tCols);
				for (unsigned j = 0; j < mTheta[i].size(); j++) {
					modelFile >> mTheta[i][j];
				}
			}

			// Load words counts
			modelFile >> mVocabSize;
			modelFile >> mNumTopics;
			modelFile >> mAlpha;
			modelFile >> mBeta;

			unsigned inValue;
			modelFile >> inValue;
			mNwz.resize(inValue);
			for (unsigned i = 0; i < mNwz.size(); i++) {
				modelFile >> inValue;
				mNwz[i].resize(inValue);
				for (unsigned j = 0; j < mNwz[i].size(); j++) {
					modelFile >> mNwz[i][j];
				}
			}

			modelFile >> inValue;
			mNzm.resize(inValue);
			for (unsigned i = 0; i < mNzm.size(); i++) {
				modelFile >> inValue;
				mNzm[i].resize(inValue);
				for (unsigned j = 0; j < mNzm[i].size(); j++) {
					modelFile >> mNzm[i][j];
				}
			}

			modelFile >> inValue;
			mNz.resize(inValue);
			for (unsigned i = 0; i < mNz.size(); i++) {
				modelFile >> mNz[i];
			}

			modelFile >> inValue;
			mNwSum.resize(inValue);
			for (unsigned i = 0; i < mNwSum.size(); i++) {
				modelFile >> mNwSum[i];
			}

			modelFile >> inValue;
			mZ.resize(inValue);
			for (unsigned i = 0; i < mZ.size(); i++) {
				modelFile >> inValue;
				mZ[i].resize(inValue);
				for (unsigned j = 0; j < mZ[i].size(); j++) {
					modelFile >> mZ[i][j];
				}
			}

		}

		//-----------------------------------------------------------------------------------------------------------------
		void LDA::initVariables(Corpus _corpus) {
			// Initialize count variables.
			mNwz.resize(mVocabSize);
			for (unsigned i = 0; i < mVocabSize; i++)
				mNwz[i].resize(mNumTopics);

			mNzm.resize(mNumTopics);
			for (unsigned i = 0; i < mNumTopics; i++)
				mNzm[i].resize(_corpus.numDocs());

			mNz.resize(mNumTopics);

			// Initialize Markov Chaizn
			mZ.resize(_corpus.numDocs());
			for (unsigned m = 0; m < _corpus.numDocs(); m++) {
				unsigned l = _corpus.document(m).lenght();
				mZ[m].resize(l);
			}
		}

		//-----------------------------------------------------------------------------------------------------------------
		void LDA::initCount(Corpus _corpus) {
			for (unsigned m = 0; m < _corpus.numDocs(); m++) {
				unsigned l = _corpus.document(m).lenght();
				for (unsigned n = 0; n < l; n++) {
					// for now we'll initialize with a RANDOM topic.
					// Later, let the user decide if it's random or null (-1)
					unsigned topic = rand() % mNumTopics;

					mZ[m][n] = topic;

					// now set the counts
					mNwz[_corpus.document(m).word(n)][topic]++;
					mNzm[topic][m]++;
					mNz[topic]++;
				}
			}
		}

		//-----------------------------------------------------------------------------------------------------------------
		unsigned LDA::sampleZ(unsigned _m, unsigned _n, unsigned _word) {
			// get old topic
			unsigned oldtopic = mZ[_m][_n];
			//std::cout << "oldtopic: " << oldtopic << std::endl;


			// first decrement counts for this word/topic
			mNwz[_word][oldtopic]--;
			mNzm[oldtopic][_m]--;
			mNz[oldtopic]--;

			// sample new topic...

			// first, create the multinomial distribution
			vector< double > p(mNumTopics);
			for (unsigned k = 0; k < mNumTopics; k++)
				p[k] = (mNwz[_word][k] + mBeta) / (mNz[k] + (mBeta*mVocabSize)) * (mNzm[k][_m] + mAlpha);

			// because this distribution is unnormalized, we'll cumulate the parameters then take a sample
			for (unsigned k = 1; k < mNumTopics; k++)
				p[k] += p[k - 1];

			unsigned newtopic;
			double rn = ((double)rand() / (RAND_MAX)) * p[mNumTopics - 1];

			for (newtopic = 0; newtopic < mNumTopics; newtopic++) {
				if (rn <= p[newtopic])	// 666 Change.
					break;
			}

			//std::cout << "Newtopic: " << newtopic << std::endl;
			// increment counts for sampled topic
			mNwz[_word][newtopic]++;
			mNzm[newtopic][_m]++;
			mNz[newtopic]++;

			// set new topic
			return newtopic;
		}

	}	//	namesapce algorithm
}	//	namespace BOViL