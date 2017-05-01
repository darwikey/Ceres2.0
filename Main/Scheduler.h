#ifndef SCHEDULER_H
#define SCHEDULER_H


class Scheduler
{
public:
	static void setPeriod(unsigned long usPeriod);
	static unsigned long getPeriod();
	static void setOnOverflow( void (*)() );
	static void enable();
	static void disable();


	static void(*onOverflow)(); // not really public, but I can't work out the 'friend' for the SIGNAL

private:
	static bool enabled;
};

#endif
