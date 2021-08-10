/*
 * Copyright 2021 Minchul Jun (mcjun37@naver.com).  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY MINCHUL JUN ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL MINCHUL JUN OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of Minchul Jun.
 *
 */
 
#include "ADS131A04.h"
#include "cmsis_os.h"
#include "stdlib.h"
#include "stm32f7xx_it.h"

_adcConfA* adcConfA = NULL;

// Declare function prototype
HAL_StatusTypeDef ADS131A04_send_command(uint16_t cmd, uint16_t addr, uint16_t regs);
void ADS131A04_control_cs_signal(_signalState onOff);
GPIO_PinState ADS131A04_read_cs_signal();
HAL_StatusTypeDef ADS131A04_recv_response(uint16_t cmd, uint16_t addr, uint8_t regs, _sendFlg sendFlg);

HAL_StatusTypeDef ADS131A04_init(_adcType adcType, SPI_HandleTypeDef* hspi)
{
    HAL_StatusTypeDef result = HAL_OK;
    _adcConfA* adcConf;

    if(adcType == ADS131A04_ADC1)
    {
        adcConfA                  = (_adcConfA*)malloc(sizeof(_adcConfA));
        memset(adcConfA, 0, sizeof(_adcConfA));
        adcConfA->type            = adcType;
        adcConfA->stat            = ADS131A04_INIT;
        adcConfA->hspi            = hspi;
        adcConfA->nReset.port     = SPI1_NRESET_GPIO_Port;
        adcConfA->nReset.pin      = SPI1_NRESET_Pin;
        adcConfA->cs.port         = SPI1_CS_GPIO_Port;
        adcConfA->cs.pin          = SPI1_CS_Pin;
        adcConfA->nDrdy.port      = SPI1_NDRDY_GPIO_Port;
        adcConfA->nDrdy.pin       = SPI1_NDRDY_Pin;
        adcConfA->bufLen          = WORDS_IN_FRAME * WORD_LENGTH;      // 5 Words x 32 Bits
        adcConfA->rxBuf           = (uint8_t*)malloc(adcConfA->bufLen);
        memset(adcConfA->rxBuf, 0, adcConfA->bufLen);
        adcConfA->txBuf           = (uint8_t*)malloc(adcConfA->bufLen);
        memset(adcConfA->txBuf, 0, adcConfA->bufLen);

        adcConf = adcConfA;
    }
    else
    {
        result = HAL_ERROR;
        return result;
    }

    // Reset Sequence
    HAL_GPIO_WritePin(adcConf->cs.port, adcConf->cs.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(adcConf->nReset.port, adcConf->nReset.pin, GPIO_PIN_RESET);

    result = ADS131A04_startup();

    if(result == HAL_OK)
    {
        adcConf->stat = ADS131A04_NORMAL;
    }
    else
    {
        adcConf->stat = ADS131A04_INIT_FAIL;
    }

    return result;
}

SPI_HandleTypeDef* is_spi_type(_adcType adcType)
{
    if(adcType == ADS131A04_ADC1)
    {
        return adcConfA->hspi;
    }
    else
    {
        return NULL;
    }
}

uint8_t* is_rxbuf(_adcType adcType)
{
    if(adcType == ADS131A04_ADC1)
    {
        return adcConfA->rxBuf;
    }
    else
    {
        return NULL;
    }
}

_ADS131A04_stat is_adc_status(_adcType adcType)
{
    if(adcType == ADS131A04_ADC1)
    {
        return adcConfA->stat;
    }
    else
    {
        return 0;
    }
}

uint8_t is_buf_len(_adcType adcType)
{
    if(adcType == ADS131A04_ADC1)
    {
        return adcConfA->bufLen;
    }
    else
    {
        return 0;
    }
}

HAL_StatusTypeDef ADS131A04_send_command(uint16_t cmd, uint16_t addr, uint16_t regs)
{
    HAL_StatusTypeDef result = HAL_OK;
    _sendFlg sendFlg = SEND_SYSTEM_CMD;
    GPIO_PinState pinStat = GPIO_PIN_RESET;

    switch(cmd)
    {
        case ADS131A04_CMD_NULL:
        case ADS131A04_CMD_RESET:
        case ADS131A04_CMD_STANDBY:
        case ADS131A04_CMD_WAKEUP:
        case ADS131A04_CMD_LOCK:
        case ADS131A04_CMD_UNLOCK:
            adcConfA->txBuf[0] = ADS131A04_UPPER(cmd);
            adcConfA->txBuf[1] = ADS131A04_LOWER(cmd);
            adcConfA->txBuf[2] = 0;
            adcConfA->txBuf[3] = 0;
            adcConfA->command = cmd;
            adcConfA->regAddr = 0;

            sendFlg = SEND_SYSTEM_CMD;
            break;
        case ADS131A04_CMD_RREG:    // ADS131A04_CMD_RREGS
            adcConfA->txBuf[0] = ADS131A04_UPPER(ADS131A04_REG_COMMAND(cmd,addr,regs));
            adcConfA->txBuf[1] = ADS131A04_LOWER(ADS131A04_REG_COMMAND(cmd,addr,regs));
            adcConfA->txBuf[2] = 0;
            adcConfA->txBuf[3] = 0;
            adcConfA->command = cmd;
            adcConfA->regAddr = addr;
            adcConfA->regs = regs;

            sendFlg = SEND_REGISTER_CMD;
            break;
        case ADS131A04_CMD_WREG:
        case ADS131A04_CMD_WREGS:
            adcConfA->txBuf[0] = ADS131A04_UPPER(ADS131A04_REG_COMMAND(cmd,addr,regs));
            adcConfA->txBuf[1] = ADS131A04_LOWER(ADS131A04_REG_COMMAND(cmd,addr,regs));
            adcConfA->txBuf[2] = 0;
            adcConfA->txBuf[3] = 0;
            adcConfA->command = cmd;
            adcConfA->regAddr = addr;
            adcConfA->regs = regs;
            adcConfA->sr.mp[addr] = regs;

            sendFlg = SEND_REGISTER_CMD;
            break;
        default:
            break;
    }

    // Check availablity to send data
    
    while(GPIO_PIN_RESET == pinStat)
    {
        pinStat = ADS131A04_read_cs_signal();
    }
    ADS131A04_control_cs_signal(RESET_SIGNAL);

    result = HAL_SPI_TransmitReceive(adcConfA->hspi, adcConfA->txBuf, adcConfA->rxBuf, adcConfA->bufLen, 500);        
    ADS131A04_control_cs_signal(SET_SIGNAL);

    if(sendFlg == SEND_SYSTEM_CMD)
    {
        adcConfA->response = ADS131A04_MERGE_BYTES(adcConfA->rxBuf[0], adcConfA->rxBuf[1]);
    }
    else if(sendFlg == SEND_REGISTER_CMD)
    {
        adcConfA->sr.mp[adcConfA->regAddr] = regs;
    }

    if(HAL_OK != result)
    {
        ADS131A04_control_cs_signal(SET_SIGNAL);
    }
    else
    {
        // DO NOTHING
    }
    return result;
}

HAL_StatusTypeDef ADS131A04_recv_response(uint16_t cmd, uint16_t addr, uint8_t regs, _sendFlg sendFlg)
{
    uint8_t confirmBuf[2];
    uint16_t temp;
    // Initialize buffer to send NULL command
    memset(adcConfA->txBuf, 0, adcConfA->bufLen);

    // Check availablity to send data
    while(GPIO_PIN_RESET == ADS131A04_read_cs_signal())
    {}
    ADS131A04_control_cs_signal(RESET_SIGNAL);

    if(HAL_OK != HAL_SPI_TransmitReceive(adcConfA->hspi, adcConfA->txBuf, adcConfA->rxBuf, adcConfA->bufLen, 500))
    {
        return HAL_ERROR;
    }
    ADS131A04_control_cs_signal(SET_SIGNAL);

    if(sendFlg == SEND_SYSTEM_CMD)
    {
        adcConfA->response = ADS131A04_MERGE_BYTES(adcConfA->rxBuf[0], adcConfA->rxBuf[1]);

        if(cmd != adcConfA->response)
        {
            return HAL_ERROR;
        }
    }
    else if(sendFlg == SEND_REGISTER_CMD)
    {
        temp = ADS131A04_REG_COMMAND(cmd,addr,regs);
        confirmBuf[0] = ADS131A04_UPPER(temp);

        if(confirmBuf[0] != adcConfA->rxBuf[0])
        {
            return HAL_ERROR;
        }
        else
        {
            adcConfA->sr.mp[adcConfA->regAddr] = adcConfA->rxBuf[1];    
        }

        if(regs != adcConfA->sr.mp[adcConfA->regAddr])
        {
            return HAL_ERROR;
        }
    }
    else
    {
        // Do Nothing
    }

    return HAL_OK;

}

uint32_t ADS131A04_convert_adc_data(const uint8_t* dataBuf)
{
    uint32_t upperByte;
    uint32_t middleByte;
    uint32_t lowerByte;

    // The output data extends to 32 bits with eight zeroes(0b00000000, 1Byte) added to the least significant bits when using the 32-bit device word length setting, datasheet 38p
    upperByte    = ((uint32_t) dataBuf[0] << 16);
    middleByte   = ((uint32_t) dataBuf[1] << 8);
    lowerByte  = ((uint32_t) dataBuf[2] << 0);

    return (upperByte | middleByte | lowerByte);
}

float ADS131A04_convert_to_mVolt(uint32_t reg)
{
    // FS = AVDD - AVSS = 2.5v -(-2.5v) = 5v, FS / 2^23 = 0x000001 (Output Code)
    const float unitFS = 5000.0f / 8388607.0f; // unit: mV (if unit is V, calculated value is out of 'float' range)
	const uint32_t boundaryValue = 0x7FFFFF; // threshold of positive value 
	int signFlg = 0;		// +: 1, -: -1

    // negative value
    if (reg > boundaryValue)
    {
        reg = (0xFFFFFF - reg) + 1; // if value is 0xFFFFFF, register is -FS/2^23. so plus 1 
        signFlg = -1;
    }
    // positive value
    else
    {
        signFlg = 1;
    }

    // convert register to mVolt
    return (unitFS * (float)reg * (float)signFlg);
}

void ADS131A04_parse_adc_data()
{
    _ADS131A04_ch ch;
    uint8_t index;

    adcConfA->response = ADS131A04_MERGE_BYTES(adcConfA->rxBuf[0], adcConfA->rxBuf[1]);

    for(ch = ADC1_CH1, index = 1; ch < NUMB_ADC1_CH; ch++, index++)
    {
        adcConfA->chData[ch].r = ADS131A04_convert_adc_data(&adcConfA->rxBuf[index * WORD_LENGTH]);        
        adcConfA->chData[ch].v = ADS131A04_convert_to_mVolt(adcConfA->chData[ch].r);
    }
}

void ADS131A04_control_cs_signal(_signalState onOff)
{
    if(onOff == SET_SIGNAL)
    {
        HAL_GPIO_WritePin(adcConfA->cs.port, adcConfA->cs.pin, GPIO_PIN_SET);
    }
    else if(onOff == RESET_SIGNAL)
    {
        HAL_GPIO_WritePin(adcConfA->cs.port, adcConfA->cs.pin, GPIO_PIN_RESET);
    }
    else
    {
        // Do Nothing
    }
}

GPIO_PinState ADS131A04_read_cs_signal()
{
    return HAL_GPIO_ReadPin(adcConfA->cs.port, adcConfA->cs.pin);
}

void ADS131A04_receive_data()
{
    ADS131A04_control_cs_signal(RESET_SIGNAL);
    HAL_SPI_Receive_DMA(adcConfA->hspi, adcConfA->rxBuf, adcConfA->bufLen);
}

void ADS131A04_set_default_register()
{
    adcConfA->sr.mp[ID_MSB_ADDRESS]         =   ID_MSB_NU_CH_4;               /* NOTE: This a read-only register */
    adcConfA->sr.mp[ID_LSB_ADDRESS]         =   0x00;                         /* NOTE: REV_ID value is unknown until read */
    adcConfA->sr.mp[STAT_1_ADDRESS]         =   STAT_1_DEFAULT;
    adcConfA->sr.mp[STAT_P_ADDRESS]         =   STAT_P_DEFAULT;
    adcConfA->sr.mp[STAT_N_ADDRESS]         =   STAT_N_DEFAULT;
    adcConfA->sr.mp[STAT_S_ADDRESS]         =   STAT_S_DEFAULT;
    adcConfA->sr.mp[ERROR_CNT_ADDRESS]      =   ERROR_CNT_DEFAULT;
    adcConfA->sr.mp[STAT_M2_ADDRESS]        =   STAT_M2_DEFAULT & STAT_M2_DEFAULT_MASK;
    adcConfA->sr.mp[A_SYS_CFG_ADDRESS]      =   A_SYS_CFG_DEFAULT;
    adcConfA->sr.mp[D_SYS_CFG_ADDRESS]      =   D_SYS_CFG_DEFAULT;
    adcConfA->sr.mp[CLK1_ADDRESS]           =   CLK1_DEFAULT;
    adcConfA->sr.mp[CLK2_ADDRESS]           =   CLK2_DEFAULT;
    adcConfA->sr.mp[ADC_ENA_ADDRESS]        =   ADC_ENA_DEFAULT;
    adcConfA->sr.mp[ADC1_ADDRESS]           =   ADC1_DEFAULT;
    adcConfA->sr.mp[ADC2_ADDRESS]           =   ADC2_DEFAULT;
    adcConfA->sr.mp[ADC3_ADDRESS]           =   ADC3_DEFAULT;
    adcConfA->sr.mp[ADC4_ADDRESS]           =   ADC4_DEFAULT;
}

HAL_StatusTypeDef ADS131A04_startup()
{
    uint8_t regs;
    int i;

    HAL_GPIO_WritePin(adcConfA->nReset.port, adcConfA->nReset.pin, GPIO_PIN_SET);
    HAL_Delay(50);

    ADS131A04_control_cs_signal(SET_SIGNAL);
    
    ADS131A04_set_default_register();

    HAL_Delay(5);

    // Send NULL command to start SPI
    ADS131A04_send_command(ADS131A04_CMD_NULL, 0, 0);

    if(HAL_OK != ADS131A04_recv_response(ADS131A04_RES_READYWORD, 0, 0, SEND_SYSTEM_CMD))
    {
        return HAL_ERROR;
    }

    ADS131A04_send_command(ADS131A04_CMD_UNLOCK, 0, 0);

    if(HAL_OK != ADS131A04_recv_response(ADS131A04_CMD_UNLOCK, 0, 0, SEND_SYSTEM_CMD))
    {
        return HAL_ERROR;
    }

    ///////////////////////////////////////
    // Write Clock Configuration 1 Register
    ///////////////////////////////////////

    regs = 0;
    // Set Clock Configuration (clock divider ratio of fCLKIN for fICLK)
    regs |= CLK1_CLK_DIV_MASK & CLK1_CLK_DIV_2;

    ADS131A04_send_command(ADS131A04_CMD_WREG, CLK1_ADDRESS, regs);

    // Response to write register command(0b010) is Read register(0b001)
    // so confirm command with read register(0b001)
    if(HAL_OK != ADS131A04_recv_response(ADS131A04_CMD_RREG, CLK1_ADDRESS, regs, SEND_REGISTER_CMD))
    {
        return HAL_ERROR;
    }
    
    ///////////////////////////////////////
    // Write Clock Configuration 2 Register
    ///////////////////////////////////////

    regs = 0;
    // Set Clock Configuration 2 (clock divider ratio of fICLK for fMOD)
    regs |= CLK2_ICLK_DIV_MASK & CLK2_ICLK_DIV_2;
    // Set Clock Configuration 2 (clock divider ratio of fDATA for fMOD
    regs |= CLK2_OSR_MASK & CLK2_OSR_4096;

    ADS131A04_send_command(ADS131A04_CMD_WREG, CLK2_ADDRESS, regs);

    if(HAL_OK != ADS131A04_recv_response(ADS131A04_CMD_RREG, CLK2_ADDRESS, regs, SEND_REGISTER_CMD))
    {
        return HAL_ERROR;
    }

    ///////////////////////////////////////
    // Enable ALL ADC Channel
    ///////////////////////////////////////

    regs = 0;
    // Set ADC Enable Register to Enable All ADC Channel
    regs = ADC_ENA_ENA_MASK | ADC_ENA_ENA_ALL_CH_PWUP;

    ADS131A04_send_command(ADS131A04_CMD_WREG, ADC_ENA_ADDRESS, regs);

    if(HAL_OK != ADS131A04_recv_response(ADS131A04_CMD_RREG, ADC_ENA_ADDRESS, regs, SEND_REGISTER_CMD))
    {
        return HAL_ERROR;
    }

    // Wakeup device
    ADS131A04_send_command(ADS131A04_CMD_WAKEUP, 0, 0);

    if(HAL_OK != ADS131A04_recv_response(ADS131A04_CMD_WAKEUP, 0, 0, SEND_SYSTEM_CMD))
    {
        return HAL_ERROR;
    }

    // Ignore the first 5 conversion results to allow for the
    // output buffers to fill-up and the SINC3 filter to settle
    for(i = 0; i < 5; i++)
    {
        ADS131A04_wait_drdy_int(100);
        ADS131A04_control_cs_signal(RESET_SIGNAL);
        HAL_SPI_Receive(adcConfA->hspi, adcConfA->rxBuf, adcConfA->bufLen, 500);
        ADS131A04_control_cs_signal(SET_SIGNAL);
    }

    return HAL_OK;
}