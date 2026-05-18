#include "ad5940.h"
#include <stdio.h>
#include "string.h"
#include "math.h"
#include "DPV.h"

uint32_t CurrBaseCode;
uint32_t PulseCode;

static AppSWVCfg_Type AppSWVCfg = 
{
  .bParaChanged = bFALSE,
  .SeqStartAddr = 0,
  .MaxSeqLen = 0,
  .SeqStartAddrCal = 0,
  .MaxSeqLenCal = 0,

  .LFOSCClkFreq = 32000.0,
  .SysClkFreq = 16000000.0,
  .AdcClkFreq = 16000000.0,
  .RcalVal = 10000.0,
  .ADCRefVolt = 1820.0f,

  .RampStartVolt = -200.0f,
  .RampPeakVolt  = +800.0f,
  .VzeroStart    = 1300.0f,

  .StepNumber        = 20,
  .StepHeight_mV     = 50.0f,
  .PulseAmplitude_mV = 25.0f,
  .PulseWidth_ms     = 20.0f,
  .QuietTime_ms      = 200.0f,

  .SampleDelay       = 5.0f,
  .LPTIARtiaSel      = LPTIARTIA_20K,
  .ExternalRtiaValue = 20000.0f,
  .AdcPgaGain        = ADCPGA_1P5,
  .ADCSinc3Osr       = ADCSINC3OSR_4,
  .FifoThresh        = 2,

  .DPVInited     = bFALSE,
  .StopRequired  = bFALSE,
  .RampState     = DPV_STATE0,
};

/* Vzero 6-bit code, computed once in AppSWVInit from VzeroStart */
static uint32_t CurrVzeroCode = 0;

/* Flag for first-time DAC sequence generation (was in config struct) */
static BoolFlag bFirstDACSeq = bTRUE;

static void UpdateDACCodes(void)
{
  float base_mV = AppSWVCfg.RampStartVolt +
                  AppSWVCfg.CurrStepPos * AppSWVCfg.StepHeight_mV;

  float pulse_mV = base_mV + AppSWVCfg.PulseAmplitude_mV;

  int32_t baseCode  = (int32_t)(CurrVzeroCode * 64) +
                      (int32_t)(base_mV  / DAC12BITVOLT_1LSB);
  int32_t pulseCode = (int32_t)(CurrVzeroCode * 64) +
                      (int32_t)(pulse_mV / DAC12BITVOLT_1LSB);

  if(baseCode  < 0)    baseCode  = 0;
  if(baseCode  > 4095) baseCode  = 4095;
  if(pulseCode < 0)    pulseCode = 0;
  if(pulseCode > 4095) pulseCode = 4095;
  
  //ref_afe_lpdacdat0 layout: bits [11:0] 12 bit vbias dac code; [17:12] 6 bit vzero dac code

  CurrBaseCode = (CurrVzeroCode <<12) | (uint32_t)baseCode;
  PulseCode    = (CurrVzeroCode << 12) | (uint32_t)pulseCode;
}

AD5940Err AppSWVGetCfg(void *pCfg)
{
  if(pCfg)
  {
    *(AppSWVCfg_Type**)pCfg = &AppSWVCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

AD5940Err AppSWVCtrl(uint32_t Command, void *pPara)
{
  switch (Command)
  {
    case APPCTRL_START:
    {
      WUPTCfg_Type wupt_cfg;

      if(AD5940_WakeUp(10) > 10)
        return AD5940ERR_WAKEUP;
      if(AppSWVCfg.DPVInited == bFALSE)
        return AD5940ERR_APPERROR;
      if(AppSWVCfg.RampState == DPV_STOP)
        return AD5940ERR_APPERROR;

      // confirm values
    printf("WUPT cfg: SampleDelay=%.1f QuietTime=%.1f PulseWidth=%.1f LFOSC=%.0f\r\n", AppSWVCfg.SampleDelay, AppSWVCfg.QuietTime_ms, AppSWVCfg.PulseWidth_ms, AppSWVCfg.LFOSCClkFreq);

      /* DPV cycle per step:
         SEQ0 (base DAC) -> quiet -> SEQ2 (ADC sample base)
         SEQ1 (pulse DAC) -> pulse width -> SEQ2 (ADC sample pulse)
      */
      wupt_cfg.WuptEn = bTRUE;
      wupt_cfg.WuptEndSeq = WUPTENDSEQ_D;
      wupt_cfg.WuptOrder[0] = SEQID_0;   /* write base DAC */
      wupt_cfg.WuptOrder[1] = SEQID_2;   /* sample base */
      wupt_cfg.WuptOrder[2] = SEQID_1;   /* write pulse DAC */
      wupt_cfg.WuptOrder[3] = SEQID_2;   /* sample pulse */

      /* SEQID_2: ADC sample, runs after SampleDelay */
      wupt_cfg.SeqxSleepTime[SEQID_2]  = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_2] =
        (uint32_t)(AppSWVCfg.LFOSCClkFreq * AppSWVCfg.SampleDelay / 1000.0f) - 4 - 2;

      /* SEQID_0: after base DAC, wait (QuietTime - SampleDelay) before ADC */
      wupt_cfg.SeqxSleepTime[SEQID_0]  = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_0] =
        (uint32_t)(AppSWVCfg.LFOSCClkFreq *
                   (AppSWVCfg.QuietTime_ms - AppSWVCfg.SampleDelay) / 1000.0f) - 4 - 2;

      /* SEQID_1: after pulse DAC, wait (PulseWidth - SampleDelay) before ADC */
      wupt_cfg.SeqxSleepTime[SEQID_1]  = 4;
      wupt_cfg.SeqxWakeupTime[SEQID_1] =
        (uint32_t)(AppSWVCfg.LFOSCClkFreq *
                   (AppSWVCfg.PulseWidth_ms - AppSWVCfg.SampleDelay) / 1000.0f) - 4 - 2;

      AD5940_WUPTCfg(&wupt_cfg);
      break;
    }
    case APPCTRL_STOPNOW:
    {
      if(AD5940_WakeUp(10) > 10)
        return AD5940ERR_WAKEUP;
      AD5940_WUPTCtrl(bFALSE);
      AD5940_WUPTCtrl(bFALSE);
      break;
    }
    case APPCTRL_STOPSYNC:
    {
      AppSWVCfg.StopRequired = bTRUE;
      break;
    }
    case APPCTRL_SHUTDOWN:
    {
      AppSWVCtrl(APPCTRL_STOPNOW, 0);
      AD5940_ShutDownS();
    }
    break;
    default:
    break;
  }
  return AD5940ERR_OK;
}

static AD5940Err AppSWVSeqInitGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;
  AFERefCfg_Type aferef_cfg;
  LPLoopCfg_Type lploop_cfg;
  DSPCfg_Type dsp_cfg;

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_AFECtrlS(AFECTRL_ALL, bFALSE);

  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);

  lploop_cfg.LpAmpCfg.LpAmpSel = LPAMP0;
  lploop_cfg.LpAmpCfg.LpAmpPwrMod = LPAMPPWR_NORM;
  lploop_cfg.LpAmpCfg.LpPaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaPwrEn = bTRUE;
  lploop_cfg.LpAmpCfg.LpTiaRf = LPTIARF_20K;
  lploop_cfg.LpAmpCfg.LpTiaRload = LPTIARLOAD_10R;
  lploop_cfg.LpAmpCfg.LpTiaRtia = AppSWVCfg.LPTIARtiaSel;
  if(AppSWVCfg.LPTIARtiaSel == LPTIARTIA_OPEN)
    lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5)|LPTIASW(9);
  else
    lploop_cfg.LpAmpCfg.LpTiaSW = LPTIASW(2)|LPTIASW(4)|LPTIASW(5);

  lploop_cfg.LpDacCfg.LpdacSel = LPDAC0;
  lploop_cfg.LpDacCfg.DataRst = bFALSE;
  lploop_cfg.LpDacCfg.LpDacSW = LPDACSW_VBIAS2LPPA|LPDACSW_VZERO2LPTIA;
  lploop_cfg.LpDacCfg.LpDacRef = LPDACREF_2P5;
  lploop_cfg.LpDacCfg.LpDacSrc = LPDACSRC_MMR;

  CurrVzeroCode = (uint32_t)((AppSWVCfg.VzeroStart - 200) / DAC6BITVOLT_1LSB);

  lploop_cfg.LpDacCfg.DacData6Bit  = CurrVzeroCode;
  lploop_cfg.LpDacCfg.DacData12Bit =
    (int32_t)(AppSWVCfg.RampStartVolt / DAC12BITVOLT_1LSB) + CurrVzeroCode * 64;
  lploop_cfg.LpDacCfg.PowerEn = bTRUE;
  AD5940_LPLoopCfgS(&lploop_cfg);

  AD5940_StructInit(&dsp_cfg, sizeof(dsp_cfg));
  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_LPTIA0_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_LPTIA0_P;
  dsp_cfg.ADCBaseCfg.ADCPga = AppSWVCfg.AdcPgaGain;

  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppSWVCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = ADCSINC2OSR_1067;
  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_2;
  AD5940_DSPCfgS(&dsp_cfg);

  AD5940_SEQGenInsert(SEQ_STOP());
  AD5940_SEQGenCtrl(bFALSE);
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  if(error == AD5940ERR_OK)
  {
    AD5940_StructInit(&AppSWVCfg.InitSeqInfo, sizeof(AppSWVCfg.InitSeqInfo));
    if(SeqLen >= AppSWVCfg.MaxSeqLen)
      return AD5940ERR_SEQLEN;

    AppSWVCfg.InitSeqInfo.SeqId = SEQID_3;
    AppSWVCfg.InitSeqInfo.SeqRamAddr = AppSWVCfg.SeqStartAddr;
    AppSWVCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppSWVCfg.InitSeqInfo.SeqLen = SeqLen;
    AppSWVCfg.InitSeqInfo.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&AppSWVCfg.InitSeqInfo);
  }
  else
    return error;
  return AD5940ERR_OK;
}

static AD5940Err AppSWVSeqADCCtrlGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  const uint32_t *pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataCount = 1;
  clks_cal.DataType = DATATYPE_SINC3;
  clks_cal.ADCSinc3Osr = AppSWVCfg.ADCSinc3Osr;
  clks_cal.ADCSinc2Osr = ADCSINC2OSR_1067;
  clks_cal.ADCAvgNum = ADCAVGNUM_2;
  clks_cal.RatioSys2AdcClk = AppSWVCfg.SysClkFreq/AppSWVCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  AD5940_SEQGenCtrl(bTRUE);
  AD5940_SEQGpioCtrlS(AGPIO_Pin2);
  AD5940_AFECtrlS(AFECTRL_ADCPWR, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(16*250));
  AD5940_AFECtrlS(AFECTRL_ADCCNV, bTRUE);
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));
  AD5940_AFECtrlS(AFECTRL_ADCPWR|AFECTRL_ADCCNV, bFALSE);
  AD5940_SEQGpioCtrlS(0);
  AD5940_EnterSleepS();

  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE);

  if(error == AD5940ERR_OK)
  {
    AD5940_StructInit(&AppSWVCfg.ADCSeqInfo, sizeof(AppSWVCfg.ADCSeqInfo));
    if((SeqLen + AppSWVCfg.InitSeqInfo.SeqLen) >= AppSWVCfg.MaxSeqLen)
      return AD5940ERR_SEQLEN;
    AppSWVCfg.ADCSeqInfo.SeqId = SEQID_2;
    AppSWVCfg.ADCSeqInfo.SeqRamAddr = AppSWVCfg.InitSeqInfo.SeqRamAddr + AppSWVCfg.InitSeqInfo.SeqLen;
    AppSWVCfg.ADCSeqInfo.pSeqCmd = pSeqCmd;
    AppSWVCfg.ADCSeqInfo.SeqLen = SeqLen;
    AppSWVCfg.ADCSeqInfo.WriteSRAM = bTRUE;
    AD5940_SEQInfoCfg(&AppSWVCfg.ADCSeqInfo);
  }
  else
    return error;
  return AD5940ERR_OK;
}

/**
 * DPV DAC sequence generation.
 * Each "DPV step" = one base write (SEQ0) + one pulse write (SEQ1).
 * SEQLEN_ONESTEP commands per sequence write.
 * Block0 / Block1 ping-pong; when half consumed, CUSTOMINT0 triggers refill.
 */
static AD5940Err AppSWVSeqDACCtrlGen(void)
{
  #define SEQLEN_ONESTEP   4L
  #define CURRBLK_BLK0     0
  #define CURRBLK_BLK1     1
  AD5940Err error = AD5940ERR_OK;
  uint32_t BlockStartSRAMAddr;
  uint32_t SRAMAddr;
  uint32_t i;
  uint32_t StepsThisBlock;
  BoolFlag bIsFinalBlk;
  uint32_t SeqCmdBuff[SEQLEN_ONESTEP];

  /* bCmdForSeq0: bTRUE -> this write goes to SEQ0 (base),
                  bFALSE -> this write goes to SEQ1 (pulse).
     Strictly alternating: base,pulse,base,pulse,...  */
  static BoolFlag bCmdForSeq0 = bTRUE;
  static uint32_t DACSeqBlk0Addr, DACSeqBlk1Addr;
  static uint32_t WritesRemaining;   /* each DPV step = 2 writes (base+pulse) */
  static uint32_t WritesPerBlock;
  static uint32_t DACSeqCurrBlk;

  if(bFirstDACSeq == bTRUE)
  {
    int32_t DACSeqLenMax;

    /* One DPV step = 2 DAC writes (base + pulse).
       Produce StepNumber DPV steps total. */
    WritesRemaining = AppSWVCfg.StepNumber * 2;

    /* FIFO threshold: 2 ADC samples per DPV step */
    AppSWVCfg.FifoThresh = AppSWVCfg.StepNumber * 2;

    DACSeqLenMax = (int32_t)AppSWVCfg.MaxSeqLen
                 - (int32_t)AppSWVCfg.InitSeqInfo.SeqLen
                 - (int32_t)AppSWVCfg.ADCSeqInfo.SeqLen;
    if(DACSeqLenMax < SEQLEN_ONESTEP*4)
      return AD5940ERR_SEQLEN;
    DACSeqLenMax -= SEQLEN_ONESTEP*2;
    WritesPerBlock = DACSeqLenMax/SEQLEN_ONESTEP/2;
    DACSeqBlk0Addr = AppSWVCfg.ADCSeqInfo.SeqRamAddr + AppSWVCfg.ADCSeqInfo.SeqLen;
    DACSeqBlk1Addr = DACSeqBlk0Addr + WritesPerBlock*SEQLEN_ONESTEP;
    DACSeqCurrBlk = CURRBLK_BLK0;

    AppSWVCfg.CurrStepPos = 0;
    bCmdForSeq0 = bTRUE;
  }

  if(WritesRemaining == 0) return AD5940ERR_OK;
  bIsFinalBlk = WritesRemaining <= WritesPerBlock ? bTRUE : bFALSE;
  StepsThisBlock = bIsFinalBlk ? WritesRemaining : WritesPerBlock;
  WritesRemaining -= StepsThisBlock;

  BlockStartSRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk0Addr : DACSeqBlk1Addr;
  SRAMAddr = BlockStartSRAMAddr;

  for(i = 0; i < StepsThisBlock - 1; i++)
  {
    uint32_t CurrAddr = SRAMAddr;
    uint32_t DACData;
    SRAMAddr += SEQLEN_ONESTEP;

    UpdateDACCodes();
    if(bCmdForSeq0)
    {
      DACData = CurrBaseCode;
    }
    else
    {
      DACData = PulseCode;
      /* pulse write finishes this DPV step; advance to next step */
      AppSWVCfg.CurrStepPos++;
    }

    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR) |
                           (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_SLP();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
  }

  if(bIsFinalBlk)
  {
    uint32_t CurrAddr = SRAMAddr;
    uint32_t DACData;
    SRAMAddr += SEQLEN_ONESTEP;

    UpdateDACCodes();
    if(bCmdForSeq0)
    {
      DACData = CurrBaseCode;
    }
    else
    {
      DACData = PulseCode;
      AppSWVCfg.CurrStepPos++;
    }

    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR) |
                           (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_SLP();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    CurrAddr += SEQLEN_ONESTEP;

    SeqCmdBuff[0] = SEQ_NOP();
    SeqCmdBuff[1] = SEQ_NOP();
    SeqCmdBuff[2] = SEQ_NOP();
    SeqCmdBuff[3] = SEQ_STOP();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);

    bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
  }
  else
  {
    uint32_t CurrAddr = SRAMAddr;
    uint32_t DACData;
    SRAMAddr = (DACSeqCurrBlk == CURRBLK_BLK0) ? DACSeqBlk1Addr : DACSeqBlk0Addr;

    UpdateDACCodes();
    if(bCmdForSeq0)
    {
      DACData = CurrBaseCode;
    }
    else
    {
      DACData = PulseCode;
      AppSWVCfg.CurrStepPos++;
    }

    SeqCmdBuff[0] = SEQ_WR(REG_AFE_LPDACDAT0, DACData);
    SeqCmdBuff[1] = SEQ_WAIT(10);
    SeqCmdBuff[2] = SEQ_WR(bCmdForSeq0 ? REG_AFE_SEQ1INFO : REG_AFE_SEQ0INFO,
                           (SRAMAddr<<BITP_AFE_SEQ1INFO_ADDR) |
                           (SEQLEN_ONESTEP<<BITP_AFE_SEQ1INFO_LEN));
    SeqCmdBuff[3] = SEQ_INT0();
    AD5940_SEQCmdWrite(CurrAddr, SeqCmdBuff, SEQLEN_ONESTEP);
    bCmdForSeq0 = bCmdForSeq0 ? bFALSE : bTRUE;
  }

  DACSeqCurrBlk = (DACSeqCurrBlk == CURRBLK_BLK0) ? CURRBLK_BLK1 : CURRBLK_BLK0;

  if(bFirstDACSeq)
  {
    bFirstDACSeq = bFALSE;
    if(bIsFinalBlk == bFALSE)
    {
      error = AppSWVSeqDACCtrlGen();
      if(error != AD5940ERR_OK)
        return error;
    }
    AppSWVCfg.DACSeqInfo.SeqId = SEQID_0;
    AppSWVCfg.DACSeqInfo.SeqLen = SEQLEN_ONESTEP;
    AppSWVCfg.DACSeqInfo.SeqRamAddr = BlockStartSRAMAddr;
    AppSWVCfg.DACSeqInfo.WriteSRAM = bFALSE;
    AD5940_SEQInfoCfg(&AppSWVCfg.DACSeqInfo);
  }
  return AD5940ERR_OK;
}

static AD5940Err AppSWVRtiaCal(void)
{
  fImpPol_Type RtiaCalValue;
  LPRTIACal_Type lprtia_cal;
  AD5940_StructInit(&lprtia_cal, sizeof(lprtia_cal));

  lprtia_cal.LpAmpSel = LPAMP0;
  lprtia_cal.bPolarResult = bTRUE;
  lprtia_cal.AdcClkFreq = AppSWVCfg.AdcClkFreq;
  lprtia_cal.SysClkFreq = AppSWVCfg.SysClkFreq;
  lprtia_cal.ADCSinc3Osr = ADCSINC3OSR_4;
  lprtia_cal.ADCSinc2Osr = ADCSINC2OSR_22;
  lprtia_cal.DftCfg.DftNum = DFTNUM_2048;
  lprtia_cal.DftCfg.DftSrc = DFTSRC_SINC2NOTCH;
  lprtia_cal.DftCfg.HanWinEn = bTRUE;
  lprtia_cal.fFreq = AppSWVCfg.AdcClkFreq/4/22/2048*3;
  lprtia_cal.fRcal = AppSWVCfg.RcalVal;
  lprtia_cal.LpTiaRtia = AppSWVCfg.LPTIARtiaSel;
  lprtia_cal.LpAmpPwrMod = LPAMPPWR_NORM;
  lprtia_cal.bWithCtia = bFALSE;
  AD5940_LPRtiaCal(&lprtia_cal, &RtiaCalValue);
  AppSWVCfg.RtiaValue = RtiaCalValue;
  return AD5940ERR_OK;
}

AD5940Err AppSWVInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;
  FIFOCfg_Type fifo_cfg;
  SEQCfg_Type seq_cfg;

  if(AD5940_WakeUp(10) > 10)
    return AD5940ERR_WAKEUP;

  seq_cfg.SeqMemSize = SEQMEMSIZE_4KB;
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  if((AppSWVCfg.DPVInited == bFALSE) || (AppSWVCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;

    if(AppSWVCfg.LPTIARtiaSel == LPTIARTIA_OPEN)
    {
      AppSWVCfg.RtiaValue.Magnitude = AppSWVCfg.ExternalRtiaValue;
      AppSWVCfg.RtiaValue.Phase = 0;
    }
    else
      AppSWVRtiaCal();

    AppSWVCfg.DPVInited = bFALSE;
    AD5940_SEQGenInit(pBuffer, BufferSize);
    error = AppSWVSeqInitGen();
    if(error != AD5940ERR_OK) return error;
    error = AppSWVSeqADCCtrlGen();
    if(error != AD5940ERR_OK) return error;
    AppSWVCfg.bParaChanged = bFALSE;
  }

  AppSWVCfg.RampState = DPV_STATE0;
  AppSWVCfg.StopRequired = bFALSE;

  AD5940_FIFOCtrlS(FIFOSRC_SINC3, bFALSE);
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOSrc = FIFOSRC_SINC3;
  fifo_cfg.FIFOThresh = AppSWVCfg.FifoThresh;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_2KB;
  AD5940_FIFOCfg(&fifo_cfg);

  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  bFirstDACSeq = bTRUE;
  error = AppSWVSeqDACCtrlGen();
  if(error != AD5940ERR_OK) return error;

  AppSWVCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppSWVCfg.InitSeqInfo);

  AD5940_SEQCtrlS(bTRUE);
  AD5940_SEQMmrTrig(AppSWVCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);

  AppSWVCfg.ADCSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppSWVCfg.ADCSeqInfo);

  AppSWVCfg.DACSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppSWVCfg.DACSeqInfo);

  AD5940_SEQCtrlS(bFALSE);
  AD5940_WriteReg(REG_AFE_SEQCNT, 0);
  AD5940_SEQCtrlS(bTRUE);
  AD5940_ClrMCUIntFlag();

  AD5940_AFEPwrBW(AFEPWR_LP, AFEBW_250KHZ);

  AppSWVCfg.DPVInited = bTRUE;
  return AD5940ERR_OK;
}

static int32_t AppSWVRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppSWVCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  return AD5940ERR_OK;
}

static int32_t AppSWVDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t i;
  uint32_t datacount = *pDataCount;
  float *pOut = (float *)pData;

  // ADC code -> current (uA) 
  for(i = 0; i < datacount; i++)
  {
    float volt = AD5940_ADCCode2Volt(
        pData[i] & 0xffff,
        AppSWVCfg.AdcPgaGain,
        AppSWVCfg.ADCRefVolt);
    pOut[i] = -volt / AppSWVCfg.RtiaValue.Magnitude * 1e3f;
  }

 /*  DPV subtraction: pulse - base, pairs are (base, pulse).
     Only process complete pairs. Forward traversal is safe because
     we read indices 2i, 2i+1 and write index i (i <= 2i always). */
  uint32_t pairs = datacount / 2;
  for(i = 0; i < pairs; i++)
  {
    pOut[i] = pOut[2*i + 1] - pOut[2*i];
  }

  *pDataCount = pairs;
  return 0;
}

/*
static int32_t AppSWVDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t i;
  uint32_t datacount = *pDataCount;
  float *pOut = (float *)pData;

  for(i = 0; i < datacount; i++)
  {
    float volt = AD5940_ADCCode2Volt(
        pData[i] & 0xffff,
        AppSWVCfg.AdcPgaGain,
        AppSWVCfg.ADCRefVolt);
    pOut[i] = -volt / AppSWVCfg.RtiaValue.Magnitude * 1e3f;
  }

  // SUBTRACTION DISABLED FOR DEBUG 
  return 0;
}
*/

AD5940Err AppSWVISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount;
  uint32_t FifoCnt;
  BuffCount = *pCount;
  uint32_t IntFlag;

  if(AD5940_WakeUp(10) > 10)
    return AD5940ERR_WAKEUP;
  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);
  *pCount = 0;
  IntFlag = AD5940_INTCGetFlag(AFEINTC_0);
  if(IntFlag & AFEINTSRC_CUSTOMINT0)
  {
    AD5940Err error;
    AD5940_INTCClrFlag(AFEINTSRC_CUSTOMINT0);
    error = AppSWVSeqDACCtrlGen();
    if(error != AD5940ERR_OK) return error;
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
  }
  if(IntFlag & AFEINTSRC_DATAFIFOTHRESH)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    if(FifoCnt > BuffCount)
    {
      /* buffer too small */
    }
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AppSWVRegModify(pBuff, &FifoCnt);
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);
    AppSWVDataProcess((int32_t*)pBuff, &FifoCnt);
    *pCount = FifoCnt;
    return 0;
  }
  if(IntFlag & AFEINTSRC_ENDSEQ)
  {
    FifoCnt = AD5940_FIFOGetCnt();
    AD5940_INTCClrFlag(AFEINTSRC_ENDSEQ);
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AppSWVDataProcess((int32_t*)pBuff, &FifoCnt);
    *pCount = FifoCnt;
    AppSWVCtrl(APPCTRL_STOPNOW, 0);
  }
  return 0;
}

