#ifndef _MR_CONFIG_H_
#define _MR_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#define MR_USING_ASSERT
#define MR_CFG_HEAP_SIZE 4096
#define MR_USING_LOG_ERROR
#define MR_USING_LOG_WARN
#define MR_USING_LOG_INFO
#define MR_USING_LOG_DEBUG
#define MR_USING_LOG_SUCCESS
#define MR_CFG_PRINTF_BUFSZ 128
#define MR_CFG_PRINTF_DEV_NAME "serial1"
#define MR_CFG_DEV_NAME_LEN 12
#define MR_CFG_DESC_NUM 64
#define MR_USING_DESC_CHECK
#define MR_USING_RDWR_CTL
#define MR_USING_SERIAL
#define MR_CFG_SERIAL_RD_BUFSZ 32
#define MR_CFG_SERIAL_WR_BUFSZ 0
#define MR_USING_UART1
#define MR_USING_MSH
#define MR_CFG_MSH_BUFSZ 32
#define MR_CFG_MSH_ARGS_NUM 16
#define MR_CFG_MSH_HISTORY_LINES 4
#define MR_CFG_MSH_DEV_NAME "serial1"
#define MR_CFG_MSH_PRINTF_BUFSZ 128
#define MR_CFG_MSH_PROMPT "msh"
#define MR_USING_MSH_ECHO
#define MR_USING_MSH_DEV_CMD
#define MR_USING_PIN
#define MR_USING_SPI
#define MR_USING_SPI1
#define MR_CFG_SPI_RD_BUFSZ 64
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _MR_CONFIG_H_ */
