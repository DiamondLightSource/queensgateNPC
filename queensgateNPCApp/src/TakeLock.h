/* TakeLock.h
 *
 * A class that tracks the acquisition of the lock belonging to
 * the asynPortDriver base class.  On destruction, parameter callbacks
 * are made and the lock is returned to its original state.  Use it
 * by creating an instance on the stack when the lock is required.
 * Then, when the instance goes out of scope, the lock is automatically
 * returned to its initial state.  This scoping mechanism allows
 * exceptions to be thrown without losing track of the lock.
 *
 * It is a good idea to pass the TakeLock
 * object on to called functions that require the lock to be taken
 * even though those functions may not actually access it.  The fact
 * that it is in the parameter list confirms the lock is taken.
 *
 * It can be used in functions where the lock is already taken on entry
 * (the writeXXX functions for example) by passing alreadyTaken as true.
 *
 *
 * Author:  Jonathan Thompson
 *
 */

#ifndef TAKELOCK_H_
#define TAKELOCK_H_

class asynPortDriver;
class epicsMutex;
class FreeLock;

class TakeLock {
friend class FreeLock;
public:
	TakeLock(asynPortDriver* driver, bool alreadyTaken=false);
	TakeLock(epicsMutex* mutex);
	TakeLock(FreeLock& freeLock);
	virtual ~TakeLock();
	void callParamCallbacks();
private:
	TakeLock();
	TakeLock(const TakeLock& other);
	TakeLock& operator=(const TakeLock& other);
	asynPortDriver* driver;
	epicsMutex* mutex;
	bool initiallyTaken;
};

#endif /* TAKELOCK_H_ */
