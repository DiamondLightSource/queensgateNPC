/* TakeLock.cpp
 * See .h file header for description.
 *
 * Author:  Jonathan Thompson
 *
 */

#include "FreeLock.h"
#include "TakeLock.h"
#include "asynPortDriver.h"

// Constructor.  Free a taken lock
FreeLock::FreeLock(TakeLock& takeLock)
	: driver(takeLock.driver)
    , mutex(takeLock.mutex)
{
	if(driver != NULL)
	{
		driver->unlock();
	}
	if(mutex != NULL)
	{
		mutex->unlock();
	}
}

// Destructor.  Return the lock to the taken state.
FreeLock::~FreeLock()
{
	if(driver != NULL)
	{
		driver->lock();
	}
	if(mutex != NULL)
	{
		mutex->lock();
	}
}

