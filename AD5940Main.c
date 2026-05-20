/******************************************************************************
Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

#include "DPV.h"

/**

**/

#define APPBUFF_SIZE 1024
//#define NUM_DPV_CYCLES 5
#define INTER_CYCLE_DELAY  5000      /* ms between scans */
uint32_t AppBuff[APPBUFF_SIZE];
float LFOSCFreq;    /* Measured LFOSC frequency */

/**
 * @brief An example to deal with data read back from AD5940. Here we just print data to UART
 * @note UART has limited speed, it has impact when sample rate is fast. Try to print some of the data not all of them.
 * @param pData: the buffer stored data for this application. The data from FIFO has been pre-processed.
 * @param DataCount: The available data count in buffer pData.
 * @return return 0.
*/

/*
static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
  static uint32_t idx = 0;
  for(uint32_t i = 0; i < DataCount; i++)
  {
    const char *phase = (idx % 2 == 0) ? "BASE" : "PULSE";
    printf("%lu %s %.6e\r\n", (unsigned long)idx, phase, pData[i]);
    idx++;
  }
  return 0;
}*/

static int32_t RampShowResult(float *pData, uint32_t DataCount)
{
  static uint32_t index = 0;

  AppSWVCfg_Type *pCfg;
  AppSWVGetCfg(&pCfg);

  for(uint32_t i = 0; i < DataCount; i++)
  {
    float voltage_V =
      (pCfg->RampStartVolt + index * pCfg->StepHeight_mV) / 1000.0f;

    printf("%.4f, %.6e\r\n", voltage_V, pData[i]);

    index++;
    if(index >= pCfg->StepNumber)
      index = 0;   /* reset for next scan if any */
  }
  return 0;
}






/**
 * @brief The general configuration to AD5940 like FIFO/Sequencer/Clock. 
 * @note This function will firstly reset AD5940 using reset pin.
 * @return return 0.
*/
static int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  SEQCfg_Type seq_cfg;  
  FIFOCfg_Type fifo_cfg;
  LFOSCMeasure_Type LfoscMeasure;

  AD5940_Initialize();    /* Call this right after AFE reset */
	
  /* Platform configuration */
  /* Step1. Configure clock */
  clk_cfg.HFOSCEn = bTRUE;
  clk_cfg.HFXTALEn = bFALSE;
  clk_cfg.LFOSCEn = bTRUE;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.SysClkSrc = SYSCLKSRC_HFOSC;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_HFOSC;
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  AD5940_CLKCfg(&clk_cfg);
  /* Step2. Configure FIFO and Sequencer*/
  fifo_cfg.FIFOEn = bTRUE;           /* We will enable FIFO after all parameters configured */
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;   /* 2kB for FIFO, The reset 4kB for sequencer */
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;   /* */
  fifo_cfg.FIFOThresh = 4;            /*  Don't care, set it by application paramter */
  AD5940_FIFOCfg(&fifo_cfg);
  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;  /* 4kB SRAM is used for sequencer, others for data FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bTRUE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);
  /* Step3. Interrupt controller */
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE);   /* Enable all interrupt in INTC1, so we can check INTC flags */
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH|AFEINTSRC_ENDSEQ|AFEINTSRC_CUSTOMINT0, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Measure LFOSC frequency */
  /**@note Calibrate LFOSC using system clock. The system clock accuracy decides measurment accuracy. Use XTAL to get better result. */
  LfoscMeasure.CalDuration = 1000.0;  /* 1000ms used for calibration. */
  LfoscMeasure.CalSeqAddr = 0;        /* Put sequence commands from start address of SRAM */
  LfoscMeasure.SystemClkFreq = 16000000.0f; /* 16MHz in this firmware. */
  AD5940_LFOSCMeasure(&LfoscMeasure, &LFOSCFreq);
  printf("LFOSC Freq:%f\n", LFOSCFreq);
  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);         /*  */
  return 0;
}

/**
 * @brief The interface for user to change application paramters.
 * @return return 0.
*/
void AD5940RampStructInit(void)
{
  AppSWVCfg_Type *pRampCfg;
  AppSWVGetCfg(&pRampCfg);

  pRampCfg->SeqStartAddr = 0x10;
  pRampCfg->MaxSeqLen    = 1024 - 0x10;
  pRampCfg->RcalVal      = 200.0f;       /* verify board RCAL */
  pRampCfg->ADCRefVolt   = 1820.0f;
  pRampCfg->SysClkFreq   = 16000000.0f;
  pRampCfg->LFOSCClkFreq = LFOSCFreq;

  /* DPV signal */
  pRampCfg->RampStartVolt     = -200.0f;     /* E begin = -0.2 V */
  pRampCfg->RampPeakVolt      = +800.0f;     /* E end   = +0.8 V */
  pRampCfg->VzeroStart        = 1300.0f;
  pRampCfg->StepNumber        = 80;         /* (800 - (-200)) / 5 */
  pRampCfg->StepHeight_mV     = 12.5f;        /* E step = 5 mV */
  pRampCfg->PulseAmplitude_mV = 25.0f;       /* E pulse = 25 mV */
  pRampCfg->PulseWidth_ms     = 20.0f;       /* T pulse = 20 ms */
  pRampCfg->QuietTime_ms      = 230.0f;       /* 100ms - 20ms for 0.05V/s scan */
  //for 5mV, estep 5, step number 200, quiettime 80
  //for 10mV, estep 10, step number 100, quiettime 180
  //for 15mV, estep 15, step number 67, quiettime 280

  /* ADC */
  pRampCfg->SampleDelay  = 18.0f;            /* 20-2, sample 2ms before end of pulse */
  pRampCfg->LPTIARtiaSel = LPTIARTIA_256K;   /* ±3.5 µA full scale */
}



//change Estep: step number 1000/x, stepheight=x, quiet time: 


void AD5940_Main(void)
{
  uint32_t temp;  
  AD5940PlatformCfg();
  AD5940RampStructInit();

  AppSWVInit(AppBuff, APPBUFF_SIZE);    // Initialize RAMP application. Provide a buffer, which is used to store sequencer commands 
  // RTIA calibration check 
  //AppSWVCfg_Type *pCfg; 
  //AppSWVGetCfg(&pCfg); 
  //printf("RcalVal=%.0f ohm, RTIA calibrated=%.0f ohm\r\n", pCfg->RcalVal, pCfg->RtiaValue.Magnitude); 

//eq time
  AD5940_Delay10us(500000);
  AppSWVCtrl(APPCTRL_START, 0);          // Control IMP measurment to start. Second parameter has no meaning with this command. 

  while(1)
  {
    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppSWVISR(AppBuff, &temp);
      RampShowResult((float*)AppBuff, temp);
    }

  }
  
}
/*



void AD5940_Main(void)
{
  uint32_t temp;
  uint32_t cycleNum = 0;
  uint32_t samplesThisScan = 0;
  int scanReady = 1;

  AD5940PlatformCfg();
  AD5940RampStructInit();

  AppSWVCfg_Type *pCfg;
  AppSWVGetCfg(&pCfg);

  AppSWVInit(AppBuff, APPBUFF_SIZE);

  while(1)
  {
    if(cycleNum < NUM_DPV_CYCLES && scanReady)
    {
      scanReady = 0;
      samplesThisScan = 0;
      printf("\r\n===== DPV Cycle %lu =====\r\n", (unsigned long)(cycleNum + 1));
      printf("voltage_V, current_uA\r\n");

      if(cycleNum > 0)
        AppSWVInit(AppBuff, APPBUFF_SIZE);

      AppSWVCtrl(APPCTRL_START, 0);
    }

    if(AD5940_GetMCUIntFlag())
    {
      AD5940_ClrMCUIntFlag();
      temp = APPBUFF_SIZE;
      AppSWVISR(AppBuff, &temp);

      if(temp != 0)
      {
        RampShowResult((float*)AppBuff, temp);
        samplesThisScan += temp;

        if(samplesThisScan >= pCfg->StepNumber)
        {
       
          AD5940_Delay10us(INTER_CYCLE_DELAY * 100UL); 
          cycleNum++;
          scanReady = 1;
        }
      }
    }

    if(cycleNum >= NUM_DPV_CYCLES)
    {
  
    }
  }
}

*/
