#include "bsp_dwt.h"
#include "RMLibHead.h"

static uint32_t CPU_FREQ_HZ;//CPU工作频率（frequency）单位hz
static uint32_t CPU_FREQ_HZ_US;//CPU....单位每us
static uint32_t CPU_FREQ_HZ_MS;//CPU....单位每ms

static uint32_t CYCCNT_LAST = 0;//上次计数
static uint32_t CYCCNT_ROUND = 0;//计数周期数，将25秒的dwt计数拓展


/**
 *@brief 初始化，传入主频（单位MHZ）
 */
void DWT_Init(uint32_t cpu_mhz)
{
    CPU_FREQ_HZ     = cpu_mhz * 1000000UL;
    CPU_FREQ_HZ_US  = CPU_FREQ_HZ / 1000000UL;
    CPU_FREQ_HZ_MS  = CPU_FREQ_HZ / 1000UL;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    CYCCNT_LAST  = DWT->CYCCNT;
    CYCCNT_ROUND = 0;
}

/**
 * @brief 更新函数,用于检查DWT CYCCNT寄存器是否溢出,并更新CYCCNT_RountCount
 * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出（放在一个定时里）
 *
 */
void DWT_Update(void)
{
    uint32_t now = DWT->CYCCNT;

    if (now < CYCCNT_LAST)   // 溢出
        CYCCNT_ROUND++;

    CYCCNT_LAST = now;
}
//
/**
 * @brief 计算两次调用之间的时间差（单位：秒，float精度）
 * @param  last_cnt  上一次保存的 DWT->CYCCNT 值
 * @note   自动适配 CYCCNT 溢出（32位减法天然支持回绕）
 * 
 */
float DWT_GetDeltaT(uint32_t *last_cnt)
{
    uint32_t now = DWT->CYCCNT;                         // 当前CPU周期计数
    float dt = (now - *last_cnt) * (1.0f / CPU_FREQ_HZ); // 周期数 / 频率 = 秒
    *last_cnt = now;                                    // 更新时间戳
    return dt;
}


/**
 * @brief 双精度版本 dt（更高数值精度，计算量大）
 * @note 仅在需要高精度积分或长时间统计时使用
 */
double DWT_GetDeltaT64(uint32_t *last_cnt)
{
    uint32_t now = DWT->CYCCNT;
    double dt = (double)(now - *last_cnt) / (double)CPU_FREQ_HZ;
    *last_cnt = now;
    return dt;
}


/**
 * @brief 获取扩展后的 64 位 CPU 周期计数值 3,500 年
 * @note 通过软件统计溢出，把 32bit CYCCNT 扩展为 64bit，必须保证 DWT_Update() 周期性调用
 */
static inline uint64_t DWT_GetCYCCNT64(void)
{
    DWT_Update();   // 检查是否发生32位溢出
    return ((uint64_t)CYCCNT_ROUND << 32) | DWT->CYCCNT;
}


/**
 * @brief 获取系统运行时间（单位：微秒）
 * @retval 从 DWT_Init 之后的累计运行时间
 */
uint64_t DWT_GetTime_us(void)
{
    return DWT_GetCYCCNT64() / CPU_FREQ_HZ_US;
}


/**
 * @brief 获取系统运行时间（单位：毫秒）
 */
uint32_t DWT_GetTime_ms(void)
{
    return DWT_GetCYCCNT64() / CPU_FREQ_HZ_MS;
}


/**
 * @brief 获取系统运行时间（单位：秒，float）
 * @note 适合日志、统计，不建议用于高精度控制
 */
float DWT_GetTime_s(void)
{
    return (float)DWT_GetCYCCNT64() / (float)CPU_FREQ_HZ;
}


/**
 * @brief 微秒级阻塞延时（Busy-wait）
 * @note 期间CPU空转，不会让出RTOS调度
 *       适合短延时：SPI时序、传感器触发等
 */
void DWT_Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * CPU_FREQ_HZ_US;   // 需要等待的CPU周期数

    while ((DWT->CYCCNT - start) < ticks);  // 等待直到时间到
}


/**
 * @brief 毫秒级阻塞延时
 * @warning 不适合长时间延时（会占满CPU）
 */
void DWT_Delay_ms(uint32_t ms)
{
    DWT_Delay_us(ms * 1000UL);
}
