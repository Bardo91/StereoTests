////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	BOVIL - core
//
//		Author: Pablo Ram8n Soria
//		Date: 2013/04/08
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Time and time functions

#ifndef _BOVIL_CORE_TIME_TIME_H_
#define _BOVIL_CORE_TIME_TIME_H_

#if defined(__linux__)
	#include <sys/time.h>
#elif defined(_WIN32)
	#include <Windows.h>
#endif

#include <iostream>

namespace BOViL {
	/** Class tool for time manage purposes
	*/
	class STime {
		// Static interface
	public:
		/** \brief Static method to get singleton.
		*/
		static STime*	get();

		/** \brief Static method to initialize singleton
		*/
		static void		init();

		/** \brief Static method to end destroy singleton
		*/
		static void		end();

	private:
		static	STime*				sTime;

		// Class interface
	public:
		/** \brief get current time
		*/
		double	getTime();

		/** \brief stop thread the given amount of seconds
		*	@param _seconds: number of second to be stopped
		*/
		void	delay(const unsigned _seconds);

		/** \brief stop thread the given amount of milliseconds
		*	@param _millis: number of millisecond to be stopped
		*/
		void	mDelay(const unsigned _millis);

	private:
		STime();
#if defined(__linux__)
		timeval			mInitTime;
#elif defined (_WIN32)
		LARGE_INTEGER	mInitTime;
#endif
	};

}        // namespace BOViL

#endif // _BOVIL_CORE_TIME_TIME_H_
