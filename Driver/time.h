#ifndef __TIME_H__
#define __TIME_H__

#include <stdint.h>

void     time_init(void);
uint64_t time_nowUs(void);
uint32_t time_nowMs(void);
void     time_waitUs(uint32_t delay);
void     time_waitMs(uint32_t delay);

#define TIME_EXECUTE_PERIODICALLY_BEGIN(time_ms) \
{\
    static uint32_t target = 0;\
    if(target <= time_nowMs())\
    {\
        target = time_nowMs() + time_ms;
#define TIME_EXECUTE_PERIODICALLY_END() \
    }\
}

#endif // __TIME_H__
