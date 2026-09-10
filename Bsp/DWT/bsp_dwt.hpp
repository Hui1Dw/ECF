#ifndef __BSP_DWT_HPP
#define __BSP_DWT_HPP

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "main.h"

#ifdef  __cplusplus
}
#endif
namespace BSP_n
{
    class DWT_c
    {
        public:
            static DWT_c* Get_DwtInstance();
            float GetDeltaT(uint32_t *cnt_last);
            double GetDeltaT64(uint32_t *cnt_last);
            float GetTimeline_s();
            float GetTimeline_ms();
            uint64_t GetTimeline_us();
            void Delay_s(float delay);
            void Delay_ms(float delay);
            void Delay_us(float delay);
        private:
            typedef struct
            {
                uint32_t s;
                uint16_t ms;
                uint16_t us;
            } Time_t;

            static DWT_c* dwt_instance_;  // 单例模式
            uint32_t cpu_freq_hz_, cpu_freq_hz_ms_, cpu_freq_hz_us_; // 分别表示CPU的频率，以及以毫秒和微秒为单位的CPU频率
            uint32_t cyccnt_rount_count_;                            // 用于记录CYCCNT计数器溢出的次数
            uint32_t cyccnt_last_;                                   // 用于记录上一次读取CYCCNT计数器的值
            uint64_t cyccnt64_;                                      // 用于存储CYCCNT计数器的值，作为64位整数
            Time_t sys_time_;

            DWT_c();  // 私有构造函数
            void CntUpdate();
            void SystimeUpdate();
    };
}

#endif /* BSP_DWT_HPP */