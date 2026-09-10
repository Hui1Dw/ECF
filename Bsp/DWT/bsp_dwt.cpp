/**
 * @file    bsp_dwt.cpp
 * @brief   DWT的BSP层文件，用于计时
 *
 * @verbatim
 *  DWT计时函数,支持防止dwt被多次初始化
 *  使用方法：
 *      在目标文件里面定义一个BSP_DWT_c实例指针，并调用ECF_Get_DwtInstance()函数来获取唯一实例的地址
 *      然后即可通过指针调用成员函数
 *  demo：
 *      // 定义一个BSP_DWT_c实例指针，并调用ECF_Get_DwtInstance()函数让外部实例指针指向唯一实例的地址
        static BSP_DWT_n::BSP_DWT_c* dwt_time = BSP_DWT_n::BSP_DWT_c::ECF_Get_DwtInstance();
 *
 *      // 延时函数
 *      dwt_time->ECF_DWT_Delay_s(1); // 延时1s
 *
 *      // 获取当前时间,单位为秒/s,即初始化后的时间
 *      float time = dwt_time->ECF_DWT_GetTimeline_s(); // 获取当前时间,单位为秒/s
 *
 *      // 获取两次调用之间的时间间隔,单位为秒/s
 *      // @param cnt_last 定义一个uint32_t的计时变量，将其地址作为参数传入
 *      // @return float dt 时间间隔,单位为: 秒/s
 *      dwt_time->ECF_DWT_GetDeltaT(uint32_t *cnt_last)
 *      float dt = dwt_time->ECF_DWT_GetDeltaT(uint32_t *cnt_last)
 *
 *      // 获取两次调用之间的时间间隔,单位为秒/s,高精度
 *      // @param cnt_last 定义一个uint32_t的计时变量，将其地址作为参数传入
 *      // @return double dt 时间间隔,单位为: 秒/s
 *      dwt_time->ECF_DWT_GetDeltaT64(uint32_t *cnt_last)
 *      double dt = dwt_time->ECF_DWT_GetDeltaT64(uint32_t *cnt_last)
 * @attention
 *      无
 */
#include "bsp_dwt.hpp"      // 本文件头文件

namespace BSP_n
{
    /**
     * @brief 初始化DWT,传入参数为CPU频率,单位MHz
     *
     * @param cpu_freq_mhz c板为168MHz,A板为180MHz
     */
    DWT_c::DWT_c()
    {
        /* 使能DWT外设 */
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

        /* DWT CYCCNT寄存器计数清0 */
        DWT->CYCCNT = 0u;
        /* 使能Cortex-M DWT CYCCNT寄存器 */
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

        uint32_t cpu_freq_hz = HAL_RCC_GetSysClockFreq();
        this->cpu_freq_hz_ = cpu_freq_hz;
        this->cpu_freq_hz_ms_ = cpu_freq_hz / 1000;
        this->cpu_freq_hz_us_ = cpu_freq_hz / 1000000;
        this->cyccnt_rount_count_ = 0u;
        this->CntUpdate();
    }

    /**
     * @brief 获取两次调用之间的时间间隔,单位为秒/s
     *
     * @param cnt_last 上一次调用的时间戳
     * @return float 时间间隔,单位为: 秒/s
     */
    float DWT_c::GetDeltaT(uint32_t *cnt_last)
    {
        volatile uint32_t cnt_now = DWT->CYCCNT;
        float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(this->cpu_freq_hz_));
        *cnt_last = cnt_now;
        this->CntUpdate();
        return dt;
    }

    /**
     * @brief 获取两次调用之间的时间间隔,单位为秒/s,高精度
     *
     * @param cnt_last 上一次调用的时间戳
     * @return double 时间间隔,单位为秒/s
     */
    double DWT_c::GetDeltaT64(uint32_t *cnt_last)
    {
        volatile uint32_t cnt_now = DWT->CYCCNT;
        double dt = ((uint32_t)(cnt_now - *cnt_last)) / ((double)(this->cpu_freq_hz_));
        *cnt_last = cnt_now;
        this->CntUpdate();
        return dt;
    }

    /**
     * @brief DWT更新时间轴函数,会被三个timeline函数调用
     * @attention 如果长时间不调用timeline函数,则需要手动调用该函数更新时间轴,否则CYCCNT溢出后定时和时间轴不准确
     */
    void DWT_c::SystimeUpdate()
    {
        volatile uint32_t cnt_now = DWT->CYCCNT;
        static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;
        CntUpdate();
        this->cyccnt64_ = (uint64_t)this->cyccnt_rount_count_ * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
        CNT_TEMP1 = this->cyccnt64_ / this->cpu_freq_hz_;
        CNT_TEMP2 = this->cyccnt64_ - CNT_TEMP1 * this->cpu_freq_hz_;
        this->sys_time_.s = CNT_TEMP1;
        this->sys_time_.ms = CNT_TEMP2 / this->cpu_freq_hz_ms_;
        CNT_TEMP3 = CNT_TEMP2 - this->sys_time_.ms * this->cpu_freq_hz_ms_;
        this->sys_time_.us = CNT_TEMP3 / this->cpu_freq_hz_us_;
    }

    /**
     * @brief 获取当前时间,单位为秒/s,即初始化后的时间
     *
     * @return float 时间轴
     */
    float DWT_c::GetTimeline_s()
    {
        this->SystimeUpdate();
        float DWT_Timelinef32 = this->sys_time_.s + this->sys_time_.ms * 0.001f + this->sys_time_.us * 0.000001f;
        return DWT_Timelinef32;
    }

    /**
     * @brief 获取当前时间,单位为毫秒/ms,即初始化后的时间
     *
     * @return float
     */
    float DWT_c::GetTimeline_ms()
    {
        this->SystimeUpdate();
        float DWT_Timelinef32 = this->sys_time_.s * 1000 + this->sys_time_.ms + this->sys_time_.us * 0.001f;
        return DWT_Timelinef32;
    }

    /**
     * @brief 获取当前时间,单位为微秒/us,即初始化后的时间
     *
     * @return uint64_t
     */
    uint64_t DWT_c::GetTimeline_us()
    {
        this->SystimeUpdate();
        uint64_t DWT_Timelinef32 = this->sys_time_.s * 1000000 + this->sys_time_.ms * 1000 + this->sys_time_.us;
        return DWT_Timelinef32;
    }

    /**
     * @brief DWT延时函数,单位为秒(s)
     * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
     * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
     *
     * @param Delay 延时时间,单位为秒(s)
     */
    void DWT_c::Delay_s(float delay)
    {
        uint32_t tickstart = DWT->CYCCNT;
        float wait = delay;
        while ((DWT->CYCCNT - tickstart) < wait * (float)this->cpu_freq_hz_);
    }

    /**
     * @brief DWT延时函数,单位为秒(ms)
     * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
     * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
     *
     * @param Delay 延时时间,单位为秒(ms)
     */
    void DWT_c::Delay_ms(float delay)
    {
        delay *= 0.001;
        uint32_t tickstart = DWT->CYCCNT;
        float wait = delay;
        while ((DWT->CYCCNT - tickstart) < wait * (float)this->cpu_freq_hz_);
    }

    /**
     * @brief DWT延时函数,单位为秒(us)
     * @attention 该函数不受中断是否开启的影响,可以在临界区和关闭中断时使用
     * @note 禁止在__disable_irq()和__enable_irq()之间使用HAL_Delay()函数,应使用本函数
     *
     * @param Delay 延时时间,单位为秒(us)
     */
    void DWT_c::Delay_us(float delay)
    {
        delay *= 0.000001;
        uint32_t tickstart = DWT->CYCCNT;
        float wait = delay;
        while ((DWT->CYCCNT - tickstart) < wait * (float)this->cpu_freq_hz_);
    }

    /**
     * @brief 私有函数,用于检查DWT CYCCNT寄存器是否溢出,并更新cyccnt_rount_count_
     * @attention 此函数假设两次调用之间的时间间隔不超过一次溢出
     *            它会检测DWT_CYCCNT寄存器的值是否小于上一次读取时的值，如果是，则认为计数器已经溢出，将cyccnt_rount_count_加1，
     *            然后更新cyccnt_last_的值为当前的计数器值，以备下次比较使用。
     */
    void DWT_c::CntUpdate()
    {
        static volatile uint8_t bit_locker = 0;
        if (!bit_locker)
        {
            bit_locker = 1;
            volatile uint32_t cnt_now = DWT->CYCCNT;
            if (cnt_now < this->cyccnt_last_)
            {
                this->cyccnt_rount_count_++;
            }
            this->cyccnt_last_ = DWT->CYCCNT;
            bit_locker = 0;
        }
    }


    /**
     * @brief 用于获取DWT单例
     * @return BSP_DWT_c* DWT实例
     */
    DWT_c* DWT_c::Get_DwtInstance()
    {
        static DWT_c dwt_time;
        return &dwt_time;
    }
}
