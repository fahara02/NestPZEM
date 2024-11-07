#ifndef UART_CONFIG_H
#define UART_CONFIG_H

#include "driver/uart.h"
#include "PZEM_constants.h"

namespace tmbus
{
struct uartDataFrame
{
    int baudrate;
    uart_word_length_t databits;
    uart_parity_t paritybits;
    uart_stop_bits_t stopbits;
    uart_hw_flowcontrol_t flow_ctrl;
    int rx_flow_control_threshold;

    uartDataFrame(int baudrate = pzemCore::PZEM_BAUD_RATE,
                  uart_word_length_t databits = UART_DATA_8_BITS,
                  uart_parity_t paritybits = UART_PARITY_DISABLE,
                  uart_stop_bits_t stopbits = UART_STOP_BITS_1,
                  uart_hw_flowcontrol_t flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
                  int rx_flow_control_threshold = 0) :
        baudrate(baudrate),
        databits(databits),
        paritybits(paritybits),
        stopbits(stopbits),
        flow_ctrl(flow_ctrl),
        rx_flow_control_threshold(rx_flow_control_threshold)
    {
    }

    uart_config_t configure() const
    {
        uart_config_t config;
        config.baud_rate = baudrate;
        config.data_bits = databits;
        config.parity = paritybits;
        config.stop_bits = stopbits;
        config.flow_ctrl = flow_ctrl;
        config.rx_flow_ctrl_thresh = rx_flow_control_threshold;
        config.source_clk = UART_SCLK_APB; // Assuming default clock source
        return config;
    }
};

struct UART_Config
{
    uart_port_t p;
    int gpio_rx;
    int gpio_tx;
    uart_config_t uartcfg;

    UART_Config(uart_port_t _p = PZEM_UART, int _rx = UART_PIN_NO_CHANGE,
                int _tx = UART_PIN_NO_CHANGE, uart_config_t ucfg = uartDataFrame().configure()) :
        p(_p), gpio_rx(_rx), gpio_tx(_tx), uartcfg(ucfg)
    {
    }
};

} // namespace tmbus
#endif
