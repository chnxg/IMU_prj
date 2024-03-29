#include "time.h"
#include "stm32f10x.h"

static struct 
{
    volatile uint32_t msPeriod;    // 整周期的时间，ms。
    uint32_t ticksPerUs;  // 每us等于的tick数。
    uint32_t ticksPerMs;  // 每ms等于的tick数。
    uint32_t msPerPeriod; // 每周期的ms数。
}time;

// 初始化时间。
void time_init(void)
{
    time.msPeriod = 0;
    time.ticksPerUs = SystemCoreClock / 1e6;
    time.ticksPerMs = SystemCoreClock / 1e3;
    time.msPerPeriod = 10;
    //
    SysTick_Config(SystemCoreClock/(1000/time.msPerPeriod));
}

// SysTick中断。
void SysTick_Handler(void)
{
    time.msPeriod += time.msPerPeriod;
}

// 获取当前时间，us。
uint64_t time_nowUs(void)
{
    return time.msPeriod * (uint64_t)1000 + (SysTick->LOAD - SysTick->VAL) / time.ticksPerUs;
}

// 获取当前时间，ms。
uint32_t time_nowMs(void)
{
    return time.msPeriod + (SysTick->LOAD - SysTick->VAL) / time.ticksPerMs;
}

// 延时delay us，delay>=4时才准确。
void time_waitUs(uint32_t delay)
{
    uint64_t target = time_nowUs() + delay - 2;
    while(time_nowUs() <= target)
        ; // 空循环。
}

// 延时delay ms。
void time_waitMs(uint32_t delay)
{
    time_waitUs(delay * 1000);
}
