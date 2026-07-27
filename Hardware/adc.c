#include "adc.h"
volatile bool gCheckADC;        //ADC采集成功标志位
//读取ADC的数据
float Get_battery_volt(void)
{
        static float last_voltage = 8.4f;
        uint32_t timeout = 100000U;

        gCheckADC = false;
        //软件触发ADC开始转换
        DL_ADC12_startConversion(ADC12_VOLTAGE_INST);

        /* 主循环中等待转换完成；中断仍保持开启，不阻塞400Hz控制中断。 */
        while (!gCheckADC && timeout > 0U) timeout--;
        if (gCheckADC) {
                last_voltage = DL_ADC12_getMemResult(
                        ADC12_VOLTAGE_INST, ADC12_VOLTAGE_ADCMEM_0)
                        * 3.3f * 11.0f / 4096.0f;
        }

        return last_voltage;
}

//ADC中断服务函数
void ADC12_VOLTAGE_INST_IRQHandler(void)
{
        //查询并清除ADC中断
        switch (DL_ADC12_getPendingInterrupt(ADC12_VOLTAGE_INST))
        {
                  //检查是否完成数据采集
                  case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
                       gCheckADC = true;//将标志位置1
                       break;
                  default:
                       break;
        }
}

