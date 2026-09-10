#ifndef __BSP_SPI_HPP
#define __BSP_SPI_HPP

/* 根据开发板引出的spi引脚以及CubeMX中的初始化配置设定 */
#define SPI_DEVICE_CNT 2       // C型开发板引出两路spi,分别连接BMI088/作为扩展IO在8pin牛角座引出
#define MX_SPI_BUS_SLAVE_CNT 4 // 单个spi总线上挂载的从机数目

#ifdef __cplusplus
extern "C" {
#endif

#include "spi.h"
#include "stdint.h"
#include "gpio.h"

#ifdef __cplusplus
}
#endif

// BMI088初始化配置的前向声明
namespace BSP_n
{
    struct SPI_Init_Config_s;

/* spi transmit recv mode enumerate*/
typedef enum
{
    SPI_BLOCK_MODE = 0, // 默认使用阻塞模式
    SPI_IT_MODE,
    SPI_DMA_MODE,
} SPI_TXRX_MODE_e;

class SPI_c
{
    public:
        SPI_c(SPI_Init_Config_s config);
        // 外部调用函数
        void Transmit(uint8_t *ptr_data, uint8_t len);
        void Receive(uint8_t *ptr_data, uint8_t len);
        void TransRecv(uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len);
        void SetMode(SPI_TXRX_MODE_e spi_mode);
        static void My_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
    protected:
        uint8_t *rx_buffer_;            // 本次接收的数据缓冲区
        static SPI_c *spi_instance_[MX_SPI_BUS_SLAVE_CNT]; // SPI实例指针数组
        static uint8_t idx_;
        static uint8_t SPIDeviceOnGoing[SPI_DEVICE_CNT];
        SPI_HandleTypeDef *spi_handle_; // SPI外设handle
        GPIO_TypeDef *GPIOx_;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
        uint16_t cs_pin_;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等
        SPI_TXRX_MODE_e spi_work_mode_; // 传输工作模式
        uint8_t rx_size_;               // 本次接收的数据长度
        uint8_t CS_State_;              // 片选信号状态,用于中断模式下的片选控制
        uint8_t * cs_pin_state_;        // 片选信号状态,用于中断模式下的片选控制
        // 接收的回调函数,用于解析接收到的数据
        void (*spi_module_callback)(SPI_c* register_instance); // 接收回调函数
};

    /* SPI初始化配置 */
    struct SPI_Init_Config_s
    {
        SPI_HandleTypeDef *spi_handle; // SPI外设handle
        // SPI的CSS引脚的GPIO设置
        GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
        uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等
        SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式
        void (*spi_module_callback)(SPI_c* register_instance); // 接收回调函数
    };

}

#endif /* BSP_SPI_HPP */