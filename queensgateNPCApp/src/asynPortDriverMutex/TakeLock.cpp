/* TakeLock.cpp
 * See .h file header for description.
 *
 * Author:  Jonathan Thompson
 *
 */

#include "TakeLock.h"
#include "FreeLock.h"
#include "asynPortDriver.h"

/**
 * Constructor.  Use this to take the lock (or represent an already
 * taken lock if alreadyTaken is true).
 */
TakeLock::TakeLock(asynPortDriver* driver, bool alreadyTaken)
	: driver(driver)
	, mutex(NULL)
	, initiallyTaken(alreadyTaken)
{
	if(!alreadyTaken)
	{
		driver->lock();
	}
}

/**
 * Constructor.  Use this to take an epics mutex.
 */
TakeLock::TakeLock(epicsMutex* mutex)
		: driver(NULL)
		, mutex(mutex)
		, initiallyTaken(false)
{
	mutex->lock();
}

/**
 * Constructor.  Use this to take a lock that is represented by a FreeLock.
 */
TakeLock::TakeLock(FreeLock& freeLock)
	: driver(freeLock.driver)
	, mutex(freeLock.mutex)
	, initiallyTaken(false)
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

/**
 * Destructor.  Call parameter call backs (with the lock taken) and
 * return the lock to its initial state.
 */
TakeLock::~TakeLock()
{
	callParamCallbacks();
	if(!initiallyTaken)
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
}

/**
 * Call parameter callbacks
 */
void TakeLock::callParamCallbacks()
{
	if(driver != NULL)
	{
		for(int list=0; list<driver->maxAddr; list++)
		{
			driver->callParamCallbacks(list);
		}
	}
}

