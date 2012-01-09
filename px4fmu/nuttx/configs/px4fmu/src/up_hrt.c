/*
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the author or the names of contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * High-resolution timer callouts and timekeeping.
 *
 * This can use any general or advanced STM32 timer.
 */

#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>

#include "up_hrt.h"
#include "stm32_tim.h"

/*
 * Minimum/maximum deadlines.
 *
 * These are suitable for use with a 16-bit timer/counter clocked
 * at 1MHz.  The high-resolution timer need only guarantee that it
 * not wrap more than once in the 50ms period for absolute time to
 * be consistently maintained.
 *
 * The minimum deadline must be such that the time taken between
 * reading a time and writing a deadline to the timer cannot
 * result in missing the deadline.
 */
#define HRT_INTERVAL_MIN	3
#define HRT_INTERVAL_MAX	50000

/*
 * Period of the free-running counter, in microseconds.
 */
#define HRT_COUNTER_PERIOD	65536

/*
 * Scaling factor(s) for the free-running counter; convert an input
 * in counts to a time in microseconds.
 */
#define HRT_COUNTER_SCALE(_c)	(_c)

/*
 * The timer that we use.
 */
static struct stm32_tim_dev_s	*hrt_tim;

/*
 * Queue of callout entries.
 */
static struct sq_queue_s	callout_queue;

/*
 * The time corresponding to a counter value of zero, as of the
 * last time that hrt_absolute_time() was called.
 */
static hrt_abstime		base_time;

/* timer-specific functions */
static unsigned		hrt_tim_get_counter(void);
static void		hrt_tim_set_oneshot(unsigned interval);
static void		hrt_tim_init(int timer);
static int		hrt_tim_isr(int irq, void *context);

/* callout list manipulation */
static void		hrt_call_enter(struct hrt_call *entry);
static void		hrt_call_reschedule(void);
static void		hrt_call_invoke(void);

/*
 * Read the current timer count value.
 *
 * This is used to derive the low bits of the absolute time, as
 * well as when deciding how to set the compare register when 
 * setting a deadline.
 */
static unsigned
hrt_tim_get_counter(void)
{
	return STM32_TIM_GETCOUNT(hrt_tim);
}

/*
 * Set up the one-shot timer. 
 *
 * This isn't really a one-shot, as it will repeat every period,
 * but as we're guaranteed that the interval will always be less
 * than the period, we can ignore that.
 */
static void
hrt_tim_set_oneshot(unsigned interval)
{
	irqstate_t flags = irqsave();
	unsigned compare;
	unsigned now = hrt_tim_get_counter();

	ASSERT(interval < HRT_COUNTER_PERIOD);

	/* Set the compare value accounting for timer wrap */
	compare = now + interval;
	if (compare >= HRT_COUNTER_PERIOD)
		compare -= HRT_COUNTER_PERIOD;

	STM32_TIM_SETCOMPARE(hrt_tim, 1, compare);

	irqrestore(flags);
}

/*
 * Initialise the timer we are going to use.
 *
 * We expect that we'll own one of the reduced-function STM32 general
 * timers, and that we can use channel 1 in compare mode.
 */
static void
hrt_tim_init(int timer)
{
	/* Get our timer and enable it */
	hrt_tim = stm32_tim_init(timer);

	/* Wire up our interrupt and disable the update interrupt */
	STM32_TIM_SETISR(hrt_tim, hrt_tim_isr, 0);
	STM32_TIM_DISABLEINT(hrt_tim, 0);	/* don't want update interrupt */
	STM32_TIM_ENABLEINT(hrt_tim, 1);	/* do want compare1 interrupt */

	/* Configure the timer to free-run at 1MHz and set a compare deadline
	 * a little ways away.
	 */
	STM32_TIM_SETPERIOD(hrt_tim, HRT_COUNTER_PERIOD - 1);
	STM32_TIM_SETCOMPARE(hrt_tim, 1, 1000);
	STM32_TIM_SETCLOCK(hrt_tim, 1000000);
	STM32_TIM_SETMODE(hrt_tim, STM32_TIM_MODE_UP | STM32_TIM_MODE_CK_INT);

	/* configure the timer in output-compare mode */
	STM32_TIM_SETCHANNEL(hrt_tim, 1, STM32_TIM_CH_OUTCOMPARE);
}

/*
 * Handle the compare interupt by calling the callout dispatcher
 * and then re-scheduling the next deadline.
 */
static int
hrt_tim_isr(int irq, void *context)
{

	/* ack the interrupt that got us here */
	STM32_TIM_ACKINT(hrt_tim, 1);

	hrt_call_invoke();
	hrt_call_reschedule();

	return OK;
}

/*
 * Fetch a never-wrapping absolute time value in microseconds from
 * some arbitrary epoch shortly after system start.
 */
hrt_abstime
hrt_absolute_time(void)
{
	static uint32_t last_count;
	uint32_t	count;

	count = hrt_tim_get_counter();

	/* This simple test is made possible by the guarantee that
	 * we are always called at least once per counter period.
	 */
	if (count < last_count)
		base_time += HRT_COUNTER_PERIOD;

	return base_time + HRT_COUNTER_SCALE(count);
}

/*
 * Convert a timespec to absolute time
 */
hrt_abstime
ts_to_abstime(struct timespec *ts)
{
	hrt_abstime	result;

	result = (hrt_abstime)(ts->tv_sec) * 1000000;
	result += ts->tv_nsec / 1000;

	return result;
}

/*
 * Convert absolute time to a timespec.
 */
void
abstime_to_ts(struct timespec *ts, hrt_abstime abstime)
{
	ts->tv_sec = abstime / 1000000;
	abstime -= ts->tv_sec * 1000000;
	ts->tv_nsec = abstime * 1000;
}

/*
 * Initalise the high-resolution timing module.
 */
void
hrt_init(int timer)
{
	sq_init(&callout_queue);
	hrt_tim_init(timer);
}

/*
 * Call callout(arg) after interval has elapsed.
 */
void
hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	entry->deadline = hrt_absolute_time() + delay;
	entry->period = 0;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);
}

/*
 * Call callout(arg) at calltime.
 */
void
hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	entry->deadline = calltime;
	entry->period = 0;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);	
}

/*
 * Call callout(arg) every period.
 */
void
hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	entry->deadline = hrt_absolute_time() + delay;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);	
}

/*
 * If this returns true, the entry has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool
hrt_called(struct hrt_call *entry)
{
	return entry->callout == NULL;
}

/*
 * Remove the entry from the callout list.
 */
void
hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = irqsave();

	sq_rem(&entry->link, &callout_queue);

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;

	irqrestore(flags);
}

static void
hrt_call_enter(struct hrt_call *entry)
{
	irqstate_t flags = irqsave();
	struct hrt_call	*call, *next;

	call = (struct hrt_call *)sq_peek(&callout_queue);

	if ((call == NULL) || (entry->deadline < call->deadline)) {
		sq_addfirst(&entry->link, &callout_queue);

		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();
	} else {
		while ((next = (struct hrt_call *)sq_next(&call->link)) != NULL) {
			if (entry->deadline < next->deadline) {
				sq_addafter(&entry->link, &call->link, &callout_queue);
				break;
			}
			call = next;
		}
	}
	if (call == NULL) {
		sq_addlast(&entry->link, &callout_queue);
	}

	irqrestore(flags);
}

static void
hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_callout	callout;
	void		*arg;

	while (true) {

		call = (struct hrt_call *)sq_peek(&callout_queue);
		if (call == NULL)
			break;
		if (call->deadline > hrt_absolute_time())
			break;
		sq_rem(&call->link, &callout_queue);

		/* copy the call state in case it's re-used by the callout */
		callout = call->callout;
		call->callout = NULL;
		arg = call->arg;

		/* invoke the callout */
		lldbg("call %p: %p(%p)\n", call, callout, arg);
		callout(arg);

		/* if the callout has a non-zero period, it has to be re-entered */
		if ((call->callout == NULL) && (call->period != 0)) {
			call->deadline += call->period;
			hrt_call_enter(call);
		}
	}
}

static void
hrt_call_reschedule()
{
	hrt_abstime	now = hrt_absolute_time();
	struct hrt_call	*next = (struct hrt_call *)sq_peek(&callout_queue);
	uint32_t	interval = HRT_INTERVAL_MAX;

	if (next != NULL) {
		if (next->deadline <= now) {
			/* set a minimal interval so that we call ASAP */
			interval = HRT_INTERVAL_MIN;
		} else if ((next->deadline - now) < HRT_INTERVAL_MAX) {
			/* call when due */
			interval = next->deadline - now;
		}
	}
	hrt_tim_set_oneshot(interval);
}
