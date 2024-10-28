/*! ----------------------------------------------------------------------------
 * @file    port.c
 * @brief   HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2016 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */
#include "port.h"
#include "deca_device_api.h"
#include "delay.h"

/* @fn    usleep
 * @brief precise usleep() delay
 * */
uint8_t usleep(uint16_t usec)
{
    delay_us(usec);
    return 0;
}

/* @fn    Sleep
 * @brief Sleep delay in ms using SysTick timer
 * */
void Sleep(uint32_t x)
{
    delay_ms(x);
}

/* @fn    peripherals_init
 * */
int peripherals_init(void)
{
    spi_peripheral_init();
    return 0;
}

/* @fn    spi_peripheral_init
 * */
void spi_peripheral_init()
{
    SPI_Configuration();
}

/* @fn      reset_DW1000
 * @brief   DW_RESET pin on DW1000 has 2 functions
 *          In general it is output, but it also can be used to reset the digital
 *          part of DW1000 by driving this pin low.
 *          Note, the DW_RESET pin should not be driven high externally.
 * */
void reset_DW1000(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    // Enable GPIO used for DW1000 reset as open collector output
    GPIO_InitStruct.GPIO_Pin = DW_RESET_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);

    //    //drive the RSTn pin low
    GPIO_WriteBit(DW_RESET_GPIO_Port, DW_RESET_Pin, Bit_RESET);

    usleep(1);

    // put the pin back to output open-drain (not active)
    GPIO_InitStruct.GPIO_Pin = DW_RESET_Pin;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DW_RESET_GPIO_Port, &GPIO_InitStruct);
    Sleep(2);
}


/* @fn      port_wakeup_dw1000
 * @brief   "slow" waking up of DW1000 using DW_CS only
 * */
void port_wakeup_dw1000(void)
{
    // HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET);
    // Sleep(1);
    // HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET);
    // Sleep(7); // wait 7ms for DW1000 XTAL to stabilise

    GPIO_WriteBit(DW_NSS_GPIO_Port,DW_NSS_Pin,Bit_RESET);
    Sleep(1);
    GPIO_WriteBit(DW_NSS_GPIO_Port,DW_NSS_Pin,Bit_SET);
    Sleep(7);
}

void SPI_ChangeRate(uint16_t scalingfactor)
{
    uint16_t tmpreg = 0;

    /* Get the SPIx CR1 value */
    tmpreg = SPIx->CR1;

    /*clear the scaling bits*/
    tmpreg &= 0xFFC7;

    /*set the scaling bits*/
    tmpreg |= scalingfactor;

    /* Write to SPIx CR1 */
    SPIx->CR1 = tmpreg;
}

void port_set_dw1000_slowrate(void)
{
    //hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    // hspi1.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    // SPI_Init(SPI1,&hspi1);
    SPI_ChangeRate(SPI_BaudRatePrescaler_32);
}

/* @fn      port_set_dw1000_fastrate
 * @brief   set 18MHz
 *          note: hspi1 is clocked from 72MHz
 * */
void port_set_dw1000_fastrate(void)
{
    //hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    // HAL_SPI_Init(&hspi1);    
    // hspi1.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
    // SPI_Init(SPI1,&hspi1);
    SPI_ChangeRate(SPI_BaudRatePrescaler_4);
}

int SPI_Configuration(void)
{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1|RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);
    SPI_InitTypeDef SPI_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    SPI_I2S_DeInit(SPIx);

    // SPIx Mode setup
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;   //
    //SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    //SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //
    //SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    //SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //sets BR[2:0] bits - baudrate in SPI_CR1 reg bits 4-6
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPIx, &SPI_InitStructure);

    // SPIx SCK and MOSI pin setup
    GPIO_InitStructure.GPIO_Pin = SPIx_SCK | SPIx_MOSI;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

    // SPIx MISO pin setup
    GPIO_InitStructure.GPIO_Pin = SPIx_MISO;
    //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;

    GPIO_Init(SPIx_GPIO, &GPIO_InitStructure);

    // SPIx CS pin setup
    GPIO_InitStructure.GPIO_Pin = SPIx_CS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(SPIx_CS_GPIO, &GPIO_InitStructure);

    // Disable SPIx SS Output
    SPI_SSOutputCmd(SPIx, DISABLE);

    // Enable SPIx
    SPI_Cmd(SPIx, ENABLE);

    // Set CS high
    GPIO_SetBits(SPIx_CS_GPIO, SPIx_CS);

    return 0;
}

/**
  * @brief  Checks whether the specified EXTI line is enabled or not.
  * @param  EXTI_Line: specifies the EXTI line to check.
  *   This parameter can be:
  *     @arg EXTI_Linex: External interrupt line x where x(0..19)
  * @retval The "enable" state of EXTI_Line (SET or RESET).
  */
ITStatus EXTI_GetITEnStatus(uint32_t EXTI_Line)
{
    ITStatus bitstatus = RESET;
    uint32_t enablestatus = 0;
    /* Check the parameters */
    assert_param(IS_GET_EXTI_LINE(EXTI_Line));

    enablestatus =  EXTI->IMR & EXTI_Line;
    if (enablestatus != (uint32_t)RESET)
    {
        bitstatus = SET;
    }
    else
    {
        bitstatus = RESET;
    }
    return bitstatus;
}
