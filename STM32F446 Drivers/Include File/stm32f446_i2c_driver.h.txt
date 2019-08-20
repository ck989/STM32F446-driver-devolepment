/*
 * stm32f446xx_i2c_driver.h
 *
 *  Created on: 05-Aug-2019
 *      Author: chaya kumar gowda
 */

#ifndef INC_STM32F446XX_I2C_DRIVER_H_
#define INC_STM32F446XX_I2C_DRIVER_H_


#include "stm32f446xx.h"

/*
 * this is a configuration for I2C pin
 */

typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_ACKControl;
	uint32_t I2C_FMDutyCycle;

}I2C_PinConfig_t;


/*
 * this is a structure handle for I2C pin
 */

typedef struct
{
	I2C_RegDef_t *pI2Cx;            //this holds the address of the I2C pin which it belongs to.
	I2C_PinConfig_t I2C_PinConfig;  //this holds I2C configuration settings.
	uint8_t       *pTxBuffer;
	uint8_t       *pRxBuffer;
	uint32_t       TxLen;
	uint32_t       RxLen;
	uint8_t        TxRxState;     //since I2C is half duplex
	uint8_t        DevAddr;
	uint32_t       RxSize;
	uint8_t        Sr;           //to store the repeated start value
}I2C_Handle_t;

/*
 * @I2C_SCLSpeed
 *
 */

#define I2C_SCL_SPEED_STDM                100000
#define I2C_SCL_SPEED_FM4K                400000
#define I2C_SCL_SPEED_FM2K                200000

/*
 *  @I2C_ACKControl
 */

#define I2C_ACK_EABLE                      1
#define I2C_ACK_DISABLE                    0

/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2                      0
#define I2C_FM_DUTY_16_9                   1

/*
 * @TxRxState
 */
#define I2C_BUSY_IN_TX                     1
#define I2C_BUSY_IN_RX                     1

/*
 * I2C related flag definition
 */

#define I2C_TXE_FLAG                        (1 << I2C_SR1_TXE)
#define I2C_RXNE_FLAG                       (1 << I2C_SR1_RXNE)
#define I2C_OVR_FLAG                        (1 << I2C_SR1_OVR)
#define I2C_SB_FLAG                         (1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG                       (1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG                        (1 << I2C_SR1_BTF)
#define I2C_ADD10_FLAG                      (1 << I2C_SR1_ADD10)
#define I2C_STOPF_FLAG                      (1 << I2C_SR1_STOPF)
#define I2C_ARLO_FLAG                       (1 << I2C_SR1_ARLO)
#define I2C_AR_FLAG                         (1 << I2C_SR1_AR)
#define I2C_PECER_FLAG                      (1 << I2C_SR1_PECER)
#define I2C_TIMEOUT_FLAG                    (1 << I2C_SR1_TIMEOUT)
#define I2C_BERR_FLAG                       (1 << I2C_SR1_BERR)

#define I2C_NO_SR                           RESET
#define I2C_SR                              SET

/*
 * I2C application event handler
 */

#define I2C_EV_TX_CMPLT                     0
#define I2C_EV_RX_CMPLT                     1
#define I2C_EV_STOP                         2
#define I2C_ERROR_BERR                      3
#define I2C_ERROR_ARLO                      4
#define I2C_ERROR_AF                        5
#define I2C_ERROR_OVR                       6
#define I2C_ERROR_TIMEOUT                   7
#define I2C_EV_DATA_REQ                     8
#define I2C_EV_DATA_RCV                     9


#define I2C_READY                           1
/*********************************************************************************************************
 *                                  APIs Supported by this driver
 *********************************************************************************************************/

/*
 * peripheral clock setup
 */

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * I2C initialize and deinitialize
 */

void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data send and receive
 */

void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr);


uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);
/*
 * IRQ Configuration and ISR handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityconfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_IRQ_EV_Handling(I2C_Handle_t *pI2CHandle);
void I2C_IRQ_ERR_Handling(I2C_Handle_t *pI2CHandle);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);


/*
 * other APIs
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagstatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);











#endif /* INC_STM32F446XX_I2C_DRIVER_H_ */
