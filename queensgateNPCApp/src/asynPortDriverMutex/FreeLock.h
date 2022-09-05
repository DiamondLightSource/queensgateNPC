/* FreeLock.h
 *
 * Use this class to temporarily release a lock that has been
 * taken by a TakeLock object.  The lock is retaken when the
 * FreeLock goes out of scope.
 *
 * Author:  Jonathan Thompson
 *
 */

#ifndef _SRC_FREELOCK_H_
#define _SRC_FREELOCK_H_

class TakeLock;
class asynPortDriver;
class epicsMutex;

class FreeLock {
friend class TakeLock;
public:
	FreeLock(TakeLock& takeLock);
	virtual ~FreeLock();
private:
	FreeLock();
	FreeLock(const FreeLock& other);
	FreeLock& operator=(const FreeLock& other);
	asynPortDriver* driver;
	epicsMutex* mutex;
};

#endif /* _SRC_FREELOCK_H_ */
