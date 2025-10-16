
#include <ctype.h>
#include <stdbool.h>
#include <string.h>
#include "uart_driver.h"
#include "Uart_Communicate.h"
//#include "Uart_Communicate.h"
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;

UartPort_t rk3576_uart_port = {
        .huart = &huart3,
        .name = "UART3",
};
UartPort_t debug_uart_port = {
        .huart = &huart1,
        .name = "UART1",
};

uint8_t uart1_dma_rx_buf[UART_RX_DMA_BUFFER_SIZE];   // UART1 RX buffer
uint8_t uart3_dma_rx_buf[UART_RX_DMA_BUFFER_SIZE];   // UART3 RX buffer
void rk3576_uart_port_Init(UartPort_t *port)
{
    port->tx_queue=NULL;
    port->tx_queue = xQueueCreate(UART_TX_QUEUE_LENGTH, sizeof(UartTxMessage_t));
    configASSERT(port->tx_queue != NULL);

    port->rx_queue=NULL;
    port->rx_queue = xQueueCreate(UART_RX_QUEUE_SIZE, sizeof(UartRxMessage_t));
    configASSERT(port->rx_queue != NULL);

    port->uartTxDoneSem=NULL;
    port->uartTxDoneSem=xSemaphoreCreateBinary();
    xSemaphoreGive(port->uartTxDoneSem);  // 鍒濆?�?寲鏃跺彲鍙戦�??


    port->dma_rx_buf = uart3_dma_rx_buf;

    port->parser=parse_rk3576_uart_port_stream;
    port->sender = send_rk3576_uart_port_frame;

    port->crc=crc16_modbus;

    if (port->huart->hdmarx != NULL) { 
        HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);
        __HAL_UART_ENABLE_IT(port->huart, UART_IT_IDLE); 
    } else { 
        HAL_UARTEx_ReceiveToIdle_IT(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);
     }


}
uint8_t uart2_dma_rx_buf[UART_RX_DMA_BUFFER_SIZE];   // reserved
void debug_uart_port_Init(UartPort_t *port)
{
     // 鍙戦�?侀槦鍒?
    port->tx_queue = NULL;
    port->tx_queue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogMessage_t));
    configASSERT(port->tx_queue != NULL);

    port->rx_queue=NULL;
    port->rx_queue = xQueueCreate(UART_RX_QUEUE_SIZE, sizeof(AsciiCmdMessage_t));
    configASSERT(port->rx_queue != NULL);


    port->dma_rx_buf = uart1_dma_rx_buf;

    port->parser=parse_debug_uart_port_stream;
   

    if (port->huart->hdmarx != NULL) { 
        HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE); 
        __HAL_UART_ENABLE_IT(port->huart, UART_IT_IDLE); 
    } else { 
        HAL_UARTEx_ReceiveToIdle_IT(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE); 
    }

}
void rk3576_uart_port_RxCallback(UartPort_t *port, uint16_t size)
{
    __HAL_UART_CLEAR_IDLEFLAG(port->huart);  // 娓呴櫎绌�?棽涓?�??鏍囧�?

    UartRxMessage_t msg;
    memcpy(msg.data, port->dma_rx_buf, size);
    msg.length = size;

    // 閲嶅�? DMA
    HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);

    // 鍏ラ�?
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(port->rx_queue, &msg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void debug_uart_port_RxCallback(UartPort_t *port, uint16_t size)
{
    __HAL_UART_CLEAR_IDLEFLAG(port->huart);  // 娓呴櫎绌�?棽涓?�??鏍囧�?

    UartRxMessage_t msg;
    memcpy(msg.data, port->dma_rx_buf, size);
    msg.length = size;

    // 閲嶅�? DMA
    HAL_UARTEx_ReceiveToIdle_DMA(port->huart, port->dma_rx_buf, UART_RX_DMA_BUFFER_SIZE);

    // 鍏ラ�?
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(port->rx_queue, &msg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == rk3576_uart_port.huart) {
        rk3576_uart_port_RxCallback(&rk3576_uart_port, Size);
    }
    if (huart == debug_uart_port.huart) {
        debug_uart_port_RxCallback(&debug_uart_port, Size);
    }
    // �??浠ョ户缁?娣�?��?? uart2_port銆乽art3_port �??
}

bool send_rk3576_uart_port_frame(DataType_t type, uint16_t frame_id, const void *data)
{
    uint16_t data_len;

    // 鏍�?�嵁�?诲瀷鎺�??兼暟�??闀�?�?
    switch (type) {
        case DATA_FLOAT:     data_len = sizeof(float); break;
        case DATA_UINT8_T:   data_len = sizeof(uint8_t); break;
        case DATA_TYPE_TEXT:
            data_len = strlen((const char *)data);
            if (data_len > FRAME_MAX_DATA_LEN) data_len = FRAME_MAX_DATA_LEN;
            break;
        default:
            return false;
    }
    if (data_len > FRAME_MAX_DATA_LEN) return false;  // 瀹�?�叏�?�?�??
    UartTxMessage_t msg;
    memset(&msg, 0, sizeof(msg));
    uint8_t *p = msg.data;
    // �?у�?
    *p++ = FRAME_HEADER_1;
    *p++ = FRAME_HEADER_2;
    // �??ID
    memcpy(p, &frame_id, sizeof(frame_id));
    p += sizeof(frame_id);
    // 鏁版嵁绫诲�?
    *p++ = (uint8_t)type;
    // 鏁版嵁闀�?�?
    memcpy(p, &data_len, sizeof(data_len));
    p += sizeof(data_len);
    // 鏁版嵁鍐�???
    memcpy(p, data, data_len);
    p += data_len;
    // CRC 鏍￠獙锛堜粠 frame_id �?�?�?嬶級
    uint16_t crc = crc16_modbus(msg.data + 2, sizeof(frame_id) + 1 + 2 + data_len);
    memcpy(p, &crc, 2);
    p += 2;
    // �?у�?
    *p++ = FRAME_TAIL_1;
    *p++ = FRAME_TAIL_2;
    // 璁剧疆鎬婚暱搴﹀苟鍏ラ槦
    msg.length = p - msg.data;
    return xQueueSend(rk3576_uart_port.tx_queue, &msg, 100) == pdPASS;
}
void parse_rk3576_uart_port_stream(const uint8_t *buf, uint16_t len)
{
    uint16_t index = 0;

    while (index + 11 <= len)  // 鏈€灏忓抚闀�?害鑷冲皯11瀛�?�妭
    {
        // 鎵惧抚澶?
        if (buf[index] == FRAME_HEADER_1 && buf[index + 1] == FRAME_HEADER_2)
        {
            const uint8_t *p = buf + index + 2;

            // 瑙ｆ瀽鍥哄畾瀛�????
            uint16_t frame_id;
            memcpy(&frame_id, p, 2); p += 2;

            uint8_t data_type = *p++;

            uint16_t data_len;
            memcpy(&data_len, p, 2); p += 2;

            // �?�?鏌ラ暱搴︽槸鍚﹀悎娉?
            if (data_len > FRAME_MAX_DATA_LEN || index + 11 + data_len > len) {
                // 鏁版嵁涓�??燂紝璺冲�?绛�?�緟鏇村?�?暟鎹?
                break;
            }

            const uint8_t *data_ptr = p;
            p += data_len;

            uint16_t crc_recv;
            memcpy(&crc_recv, p, 2); p += 2;

            // 鏍￠獙甯у�?
            if (p[0] != FRAME_TAIL_1 || p[1] != FRAME_TAIL_2) {
                index++;  // 涓嶆槸鏈夋晥�?э紝鍚戝悗鍋忕Щ1瀛�?�妭缁х画�??
                continue;
            }

            // CRC 鏍￠�?
            uint16_t crc_calc = rk3576_uart_port.crc(buf + index + 2, 2 + 1 + 2 + data_len);
            if (crc_recv != crc_calc) {
                LOG("CRC mismatch, frame skipped\r\n");
                index += (2 + 2 + 1 + 2 + data_len + 2 + 2);  // 璺宠繃鏁翠釜�??
                continue;
            }
            // 鍦ㄦ牎�?�屾垚鍔熷悗璋冪�?
            UartFrame_Dispatch(frame_id, data_ptr, data_len);

            index += (2 + 2 + 1 + 2 + data_len + 2 + 2);  // 绉�?�姩鍒�?�笅涓€�?ц捣濮?
        }
        else {
            index++;  // 娌℃壘鍒板抚澶达紝缁х画鍚戝悗�??
        }
    }
}
void parse_debug_uart_port_stream(const uint8_t *buf, uint16_t len)
{
    uint16_t index = 0;

    while (index + 11 <= len)  // 鏈€灏忓抚闀�?害鑷冲皯11瀛�?�妭
    {
        // 鎵惧抚澶?
        if (buf[index] == FRAME_HEADER_1 && buf[index + 1] == FRAME_HEADER_2)
        {
            const uint8_t *p = buf + index + 2;

            // 瑙ｆ瀽鍥哄畾瀛�????
            uint16_t frame_id;
            memcpy(&frame_id, p, 2); p += 2;

            uint8_t data_type = *p++;

            uint16_t data_len;
            memcpy(&data_len, p, 2); p += 2;

            // �?�?鏌ラ暱搴︽槸鍚﹀悎娉?
            if (data_len > FRAME_MAX_DATA_LEN || index + 11 + data_len > len) {
                // 鏁版嵁涓�??燂紝璺冲�?绛�?�緟鏇村?�?暟鎹?
                break;
            }

            const uint8_t *data_ptr = p;
            p += data_len;

            uint16_t crc_recv;
            memcpy(&crc_recv, p, 2); p += 2;

            // 鏍￠獙甯у�?
            if (p[0] != FRAME_TAIL_1 || p[1] != FRAME_TAIL_2) {
                index++;  // 涓嶆槸鏈夋晥�?э紝鍚戝悗鍋忕Щ1瀛�?�妭缁х画�??
                continue;
            }

            // CRC 鏍￠�?
            uint16_t crc_calc = rk3576_uart_port.crc(buf + index + 2, 2 + 1 + 2 + data_len);
            if (crc_recv != crc_calc) {
                LOG("CRC mismatch, frame skipped\r\n");
                index += (2 + 2 + 1 + 2 + data_len + 2 + 2);  // 璺宠繃鏁翠釜�??
                continue;
            }
            // 鍦ㄦ牎�?�屾垚鍔熷悗璋冪�?
            UartFrame_Dispatch(frame_id, data_ptr, data_len);

            index += (2 + 2 + 1 + 2 + data_len + 2 + 2);  // 绉�?�姩鍒�?�笅涓€�?ц捣濮?
        }
        else {
            index++;  // 娌℃壘鍒板抚澶达紝缁х画鍚戝悗�??
        }
    }
}

uint16_t crc16_modbus(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        uint32_t error = HAL_UART_GetError(huart);

        if (error & HAL_UART_ERROR_ORE) // 婧㈠�?閿欒??
        {
            //LOG_ISR("UART 婧㈠�?閿欒?痋n");
            __HAL_UART_CLEAR_OREFLAG(huart); // 娓呴櫎婧㈠嚭鏍囧�?
            // 娓呯┖鎺ユ敹缂撳啿鍖猴紝闃叉?�??婚攣
            while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE))
            {
                volatile uint8_t dummy = (uint8_t)(huart->Instance->RDR & 0xFF);
                (void)dummy; // 闃叉?㈢紪璇戝櫒璀﹀�?
            }
        }

        if (error & HAL_UART_ERROR_FE) // �?ч敊璇?
        {
            //LOG_ISR("UART �?ч敊璇痋n");
            __HAL_UART_CLEAR_FEFLAG(huart); // 娓呴櫎甯ч敊璇?鏍囧�?
        }

        if (error & HAL_UART_ERROR_NE) // �??澹伴敊璇?
        {
            //LOG_ISR("UART �??澹伴敊璇痋n");
            __HAL_UART_CLEAR_NEFLAG(huart); // 娓呴櫎鍣?澹伴敊璇?鏍囧�?
        }

        if (error & HAL_UART_ERROR_DMA) // DMA閿欒??
        {
            //LOG_ISR("UART DMA 閿欒?痋n");
            if (huart->hdmarx != NULL)
            {
                HAL_DMA_Abort(huart->hdmarx); // 鍋滄??DMA
            }
        }

        // 閲嶆柊鍚?鍔―MA鎺ユ敹锛堜繚璇佹帴鍙ｅ彲�??�??
        if ( HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart1_dma_rx_buf, UART_RX_DMA_BUFFER_SIZE) == HAL_OK)
        {
            //LOG_ISR("UART2 DMA鎺ユ敹閲嶅惎鎴愬姛\n");
        }
        else
        {
            //LOG_ISR("UART2 DMA鎺ユ敹閲嶅惎澶辫�?\n");
        }
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (huart == rk3576_uart_port.huart)
        xSemaphoreGiveFromISR(rk3576_uart_port.uartTxDoneSem, &xHigherPriorityTaskWoken);
    // else if (huart == wuhua_uart_port.huart)
    //     xSemaphoreGiveFromISR(wuhua_uart_port.uartTxDoneSem, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




