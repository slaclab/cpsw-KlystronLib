#include "KlystronFw.h"

#include <cpsw_yaml.h>
#include <yaml-cpp/yaml.h>
#include <cpsw_sval.h>
#include <cpsw_entry_adapt.h>
#include <cpsw_hub.h>
#include <fstream>
#include <sstream>

#include "TrigPulse.h"

/**
 * Constants for board access 
 */
#define FWC_COMMON_PLATFORM_IQFB_CONST_COR_COEF_FRACTION 30                                                  /* fraction bits number of the coefficients for I/Q modulator imbalance correction */
#define FWC_COMMON_PLATFORM_IQFB_CONST_GAIN_FRACTION     30                                                  /* fraction bits number of the feedback gain */
#define FWC_COMMON_PLATFORM_IQFB_CONST_PHS_FRACTION      30                                                  /* fraction bits number of the phase in radian */

#define FWC_COMMON_PLATFORM_IQFB_CONST_SP_TAB_BUF_DEPTH  4096                                                /* buffer length for writing to FPGA (for I/Q set point tables) */

#define FWC_COMMON_PLATFORM_IQFB_CONST_DAQ_BUF_DEPTH     CON_COMMON_PLATFORM_IQFB_DAQ_BUF_DEPTH               /* buffer length for a DAQ channel */
#define FWC_COMMON_PLATFORM_IQFB_CONST_DAQ_BUF_NUM       CON_COMMON_PLATFORM_IQFB_DAQ_BUF_NUM                 /* DAQ buffer number */

#define FWC_COMMON_PLATFORM_IQFB_CONST_ADC_SAMPLE_MAX    65536                                               /* 64k points (65536) */

#define PI 3.14159265358979323846


class CKlystronFwAdapt;
typedef shared_ptr<CKlystronFwAdapt> KlystronFwAdapt;

class CKlystronFwAdapt : public IKlystronFw, public IEntryAdapt {
protected:
    Path pKlystron_;
/* put ScalVals, etc. here */

    ScalVal    trigMode_;
    
    ScalVal    LO_KP_;
    ScalVal    LO_KI_;
    ScalVal    CLK_KP_;
    ScalVal    CLK_KI_;
    ScalVal    ADPLL_KP_;
    ScalVal    ADPLL_KI_;
    
    ScalVal    OutputEnable_; 
    ScalVal    CWModeEnable_; 
    ScalVal    referenceChannel_; 
    ScalVal    feedbackChannel_;
    ScalVal    referencePhaseSet_; 
    ScalVal    feedbackRotI_;
    ScalVal    feedbackRotQ_;

    ScalVal    ICorrSoftware_TS1_;
    ScalVal    QCorrSoftware_TS1_;
    ScalVal    ICorrSoftware_TS2_;
    ScalVal    QCorrSoftware_TS2_;
    ScalVal    ICorrSoftware_TS3_;
    ScalVal    QCorrSoftware_TS3_;
    ScalVal    ICorrSoftware_TS4_;
    ScalVal    QCorrSoftware_TS4_;
    ScalVal    ICorrSoftware_TS5_;
    ScalVal    QCorrSoftware_TS5_;
    ScalVal    ICorrSoftware_TS6_;
    ScalVal    QCorrSoftware_TS6_;


    ScalVal    enableTimeslotCorrection_;
    ScalVal    feedforwardP_; 
    ScalVal    feedforwardA_;
    ScalVal    gainP_;
    ScalVal    gainA_;
    ScalVal    feedbackCorrLimitP_;
    ScalVal    feedbackCorrLimitA_;
    ScalVal    integrationStart_;
    ScalVal    integrationEnd_;
    ScalVal    applyCorrStart_;
    ScalVal    applyCorrEnd_;
    ScalVal    dacOffset_;
    ScalVal    dacOutLimitHi_;
    ScalVal    dacOutLimitLow_;
    ScalVal    debugStream0Select_;
    ScalVal    debugStream1Select_;
    ScalVal    debugStream2Select_;
    ScalVal    debugStream3Select_;
    ScalVal    debugStream4Select_;
    ScalVal    debugStream5Select_;
    ScalVal    setpointTableI_;
    ScalVal    setpointTableQ_;
    ScalVal    setpointTableNelms_;
    ScalVal_RO nonIQCoefCurr_;
    ScalVal    nonIQCoefOffset_;
    ScalVal_RO sysgenMajorVersion_;
    ScalVal_RO sysgenMinorVersion_;
    ScalVal_RO fpgaVersion_;
    ScalVal_RO measuredTrigPeriod_;
    ScalVal_RO trigCounter_;
    ScalVal    msgNoDelay_;
    Command    trigHwArm_;
    Command    waveform0Init_;
    Command    waveform1Init_;
    ScalVal    daqSize_;
    ScalVal    daqBufStartAddr_;
    ScalVal    daqBufEndAddr_;

    ScalVal_RO debugPacketCnt_;
    ScalVal_RO debugTrgCnt0_;
    ScalVal_RO debugTrgCnt1_;
    
    ScalVal    atten0inBay0_;
    ScalVal    atten1inBay0_;
    ScalVal    atten2inBay0_;
    ScalVal    atten3inBay0_;
    ScalVal    atten4inBay0_;
    ScalVal    atten5inBay0_;
    
    ScalVal    atten0inBay1_;
    ScalVal    atten1inBay1_;
    ScalVal    atten2inBay1_;
    ScalVal    atten3inBay1_;
    ScalVal    atten4inBay1_;
    ScalVal    atten5inBay1_;
    
    ScalVal_RO bay0Temp0MSB_;
    ScalVal_RO bay0Temp0LSB_;
    ScalVal_RO bay0Temp1MSB_;
    ScalVal_RO bay0Temp1LSB_;
    ScalVal_RO bay0Temp2MSB_;
    ScalVal_RO bay0Temp2LSB_;
    ScalVal_RO bay0Temp3MSB_;
    ScalVal_RO bay0Temp3LSB_;
    
    ScalVal_RO bay1Temp0MSB_;
    ScalVal_RO bay1Temp0LSB_;
    ScalVal_RO bay1Temp1MSB_;
    ScalVal_RO bay1Temp1LSB_;
    ScalVal_RO bay1Temp2MSB_;
    ScalVal_RO bay1Temp2LSB_;
    ScalVal_RO bay1Temp3MSB_;
    ScalVal_RO bay1Temp3LSB_;
    
    ScalVal_RO intPhaseError_;
    ScalVal    ckPhaseSet_;
      
    
    

//    Stream stream0_;
//    Stream stream1_;
//    Stream stream2_;
//    Stream stream3_;
//    Stream stream4_;
//    Stream stream5_;
//    Stream stream6_;
//    Stream stream7_;

    std::map<StreamId, ScalVal> debugStreamSelect_;
    
    std::map<int, ScalVal> atten_;

    std::map<int, ScalVal_RO> tempBytes_;
    
    
// 
//  Interlock RTM board
//
    ScalVal_RO   rtmStatus_;
    ScalVal_RO   rtmFirmwareVersion_;
    ScalVal_RO   rtmSystemId_;
    ScalVal_RO   rtmSubType_;
    ScalVal_RO   rtmFirmwareDate_;
    ScalVal      rtmKlyWiperRegA_;
    ScalVal      rtmKlyWiperRegB_;
    ScalVal      rtmKlyNVRegA_;
    ScalVal      rtmKlyNVRegB_;
    ScalVal      rtmModWiperRegA_;
    ScalVal      rtmModWiperRegB_;
    ScalVal      rtmModNVRegA_;
    ScalVal      rtmModNVRegB_;
    ScalVal      rtmCfgRegister_;
    ScalVal_RO   rtmFaultOutStatus_;
    ScalVal_RO   rtmAdcLockedStatus_;
    ScalVal_RO   rtmRFOffStatus_;
    ScalVal_RO   rtmAdcIn_;
    ScalVal      rtmMode_;
    ScalVal      rtmTuneSled_;
    ScalVal      rtmDetuneSled_;
    ScalVal_RO   rtmAdcBufferBeamIV_;
    ScalVal_RO   rtmAdcBufferFwdRef_;
    Command      rtmRearm_Cmd_;
    Command      rtmSwTrigger_Cmd_;
    Command      rtmClearFault_Cmd_;


public:
        CKlystronFwAdapt(Key &k, Path p, shared_ptr<const CEntryImpl> ie);

public:
    virtual void reset();
    virtual void setLO_KP(uint32_t KP);
    virtual void setLO_KI(uint32_t KI);
    virtual void setCLK_KP(uint32_t KP);
    virtual void setCLK_KI(uint32_t KI);
    virtual void setADPLL_KP(uint32_t KP);
    virtual void setADPLL_KI(uint32_t KI);
    virtual void OutputEnable(bool enable);
    virtual void CWOutputEnable(bool enable);
    virtual void setDebugStreamSelect(StreamId sid, StreamSel ssel);

    virtual void setTrigMode      (TrigMode mode);
    virtual void setTrigPolarity  (Trigger trig, TrigPolarity polarity);
    virtual void setTrigDelay     (Trigger trig, double value_ns, double freq_MHz = 119.0);
    virtual void setTrigWidth     (Trigger trig, double value_ns, double freq_MHz = 119.0);
    virtual void setTrigOpCode    (Trigger trig, std::vector<int> opCodes);
    
    virtual void setDaqSize( uint32_t buf_size );  /* setup DAQ size, byte size, will be rouneded up to 4k blocks */

    virtual void selectRefFbkChannel(uint32_t refCh, uint32_t fbkCh);
    
    virtual void setRefPhaSP(double phaSP_deg);


    virtual void  setSWFeedbackCorrection(Timeslot timeslot, double amplitude_norm, double phase_deg);   /* scale and rotate the feedback signal */
    virtual void  getSWFeedbackCorrection(Timeslot timeslot, double *amplitude_norm, double *phase_deg);   /* get current settings for phase/amp correct */
    virtual void  enableTimeslotCorrection( bool enable );

    virtual void  setFeedforward(double phase, double amplitude);          /* set the feedforward value for I component */

    virtual void  setGain(double phase, double amplitude);                                   /* set the gain value for I component */

    virtual void  setFeedbackCorrLimits(double phase, double amplitude);

    virtual void  setIntgStart(double value_ns, double freq_MHz = 357.0);            /* start time for integration, relative to the trigger */
    virtual void  setIntgEnd(  double value_ns, double freq_MHz = 357.0);             

    virtual void  setApplStart(double value_ns, double freq_MHz = 357.0);            /* start time for applying the correction, relative to the trigger */
    virtual void  setApplEnd(  double value_ns, double freq_MHz = 357.0);             

    virtual void  setDACOffset(int16_t offset);                       /* set the DAC offset */

    virtual void  setAmpLimit( int16_t high, int16_t low);                        /* set the output limit */

    virtual void  setIQSPTable(uint32_t pno, double *ISPTable, double *QSPTable);                        /* set the set point table */

    virtual void  setNonIQCoefOffset(uint32_t offset);

/* Fimrware readings */
    virtual void  getAppFwInfo(uint32_t *firmwareName, uint32_t *majorVer, uint32_t *minorVer, uint32_t *buildNum);

    virtual void  getRaceConditionFlags(uint32_t *accFlags, uint32_t *stdbyFlags, uint32_t *spareFlags);

    virtual void  getADCValid(bool *valid);

    virtual void  getPulseCounter(uint32_t *pulseCnt);
    virtual void  getMeaTrigPeriod(double *value_ms, double freq_MHz = 119.0);       /* get the measurement trigger period */

    virtual void  getNonIQCoefCur(uint32_t *cur);

    virtual void  getFwStatus(uint32_t *platformStatus, uint32_t *RFCtrlStatus);


//    virtual void  getAllStreamData(uint32_t pno,                            /* data from DRAM */
//                                                            uint8_t *data0, uint8_t *data1,            
//                                                            uint8_t *data2, uint8_t *data3,
//                                                            uint8_t *data4, uint8_t *data5,
//                                                            uint8_t *data6, uint8_t *data7);

    virtual void msgNoDelay(uint32_t no_delay);
    virtual void armDaq();
    virtual void initBuf(void);

    virtual void loadConfigFromYamlFile( const char *filename, const char *yaml_dir = 0 );
    virtual void dumpConfigToYamlFile(   const char *filename, const char *yaml_dir = 0 );

    virtual void getDebugPacketCnt(uint32_t *dbgCnt);
    virtual void getDebugTrgCnt(uint32_t *dbgCnt0, uint32_t *dbgCnt1);
    
    virtual void setAtten(uint32_t att, int bay, int chn);
    virtual void setAtten(uint32_t att, int index);
    virtual void getTemp(int bay, int sensor, double *temp);
    virtual void getTemp(int index, double *temp);
    virtual void getIntPhaseError(int32_t *phase);
    virtual void setCKPhase(uint32_t ckPhase);

//
//
//


    virtual void getRtmStatus(uint32_t *rtmStatus);
    virtual void getRtmFirmwareVersion(uint32_t *rtmFirmwareVersion);
    virtual void getRtmSystemId(char *systemIdString);
    virtual void getRtmSubType(char *subTypeString);
    virtual void getRtmFirmwareDate(char *firmwareDateString);
    virtual void setRtmKlyWiperRegA(uint32_t v);
    virtual void setRtmKlyWiperRegB(uint32_t v);
    virtual void setRtmKlyNVRegA(uint32_t v);
    virtual void setRtmKlyNVRegB(uint32_t v);
    virtual void setRtmModWiperRegA(uint32_t v);
    virtual void setRtmModWiperRegB(uint32_t v);
    virtual void setRtmModNVRegA(uint32_t v);
    virtual void setRtmModNVRegB(uint32_t v);
    virtual void setRtmCfgRegister(uint32_t v);
    virtual void getRtmFaultOutStatus(uint32_t *rtmFaultOutStatus);
    virtual void getRtmAdcLockedStatus(uint32_t *rtmAdcLockedStatus);
    virtual void getRtmRFOffStatus(uint32_t *rtmRFOffStatus);
    virtual void getRtmAdcIn(uint32_t v[]);
    virtual void setRtmMode(uint32_t v);
    virtual void setRtmDesiredSled(bool tune);
    virtual void getRtmFastAdcBufferBeamCurrentVoltage(uint32_t v[]);
    virtual void getRtmFastAdcBufferForwardReflect(uint32_t v[]);
    virtual void cmdRtmRearm(void);
    virtual void cmdRtmSwTrigger(void);
    virtual void cmdRtmClearFault(void);
};

CKlystronFwAdapt::CKlystronFwAdapt(Key &k, Path p, shared_ptr<const CEntryImpl> ie) : 
  IEntryAdapt(k, p, ie),
  pKlystron_(                                      p->findByName("AppTop/AppCore") ),
  
  trigMode_(            IScalVal::create( pKlystron_->findByName("TriggerMode") ) ),
  
  LO_KP_(               IScalVal::create( pKlystron_->findByName("SysgenMR/LO_KP") ) ),
  LO_KI_(               IScalVal::create( pKlystron_->findByName("SysgenMR/LO_KI") ) ),
  CLK_KP_(              IScalVal::create( pKlystron_->findByName("SysgenMR/CLK_KP") ) ),
  CLK_KI_(              IScalVal::create( pKlystron_->findByName("SysgenMR/CLK_KI") ) ),
  ADPLL_KP_(            IScalVal::create( pKlystron_->findByName("SysgenMR/ADPLL") ) ),
  ADPLL_KI_(            IScalVal::create( pKlystron_->findByName("SysgenMR/ADPLL_KI") ) ),
  
  OutputEnable_(        IScalVal::create( pKlystron_->findByName("SysgenMR/OutputEnable") ) ),
  CWModeEnable_(        IScalVal::create( pKlystron_->findByName("SysgenMR/CWModeEnable") ) ),
  referenceChannel_(    IScalVal::create( pKlystron_->findByName("SysgenMR/ReferenceChannel") ) ),
  feedbackChannel_(     IScalVal::create( pKlystron_->findByName("SysgenMR/FeedbackChannel") ) ),
  referencePhaseSet_(   IScalVal::create( pKlystron_->findByName("SysgenMR/ReferencePhaseSet") ) ),
  feedbackRotI_(        IScalVal::create( pKlystron_->findByName("SysgenMR/FeedbackRotI") ) ),
  feedbackRotQ_(        IScalVal::create( pKlystron_->findByName("SysgenMR/FeedbackRotQ") ) ),

  ICorrSoftware_TS1_(       IScalVal::create( pKlystron_->findByName("SysgenMR/ICorrSoftware_TS1") ) ),
  QCorrSoftware_TS1_(       IScalVal::create( pKlystron_->findByName("SysgenMR/QCorrSoftware_TS1") ) ),
  ICorrSoftware_TS2_(       IScalVal::create( pKlystron_->findByName("SysgenMR/ICorrSoftware_TS2") ) ),
  QCorrSoftware_TS2_(       IScalVal::create( pKlystron_->findByName("SysgenMR/QCorrSoftware_TS2") ) ),
  ICorrSoftware_TS3_(       IScalVal::create( pKlystron_->findByName("SysgenMR/ICorrSoftware_TS3") ) ),
  QCorrSoftware_TS3_(       IScalVal::create( pKlystron_->findByName("SysgenMR/QCorrSoftware_TS3") ) ),
  ICorrSoftware_TS4_(       IScalVal::create( pKlystron_->findByName("SysgenMR/ICorrSoftware_TS4") ) ),
  QCorrSoftware_TS4_(       IScalVal::create( pKlystron_->findByName("SysgenMR/QCorrSoftware_TS4") ) ),
  ICorrSoftware_TS5_(       IScalVal::create( pKlystron_->findByName("SysgenMR/ICorrSoftware_TS5") ) ),
  QCorrSoftware_TS5_(       IScalVal::create( pKlystron_->findByName("SysgenMR/QCorrSoftware_TS5") ) ),
  ICorrSoftware_TS6_(       IScalVal::create( pKlystron_->findByName("SysgenMR/ICorrSoftware_TS6") ) ),
  QCorrSoftware_TS6_(       IScalVal::create( pKlystron_->findByName("SysgenMR/QCorrSoftware_TS6") ) ),
  
  enableTimeslotCorrection_(IScalVal::create( pKlystron_->findByName("SysgenMR/EnableTimeslotCorrection") ) ),
  feedforwardP_(        IScalVal::create( pKlystron_->findByName("SysgenMR/FeedforwardP") ) ),
  feedforwardA_(        IScalVal::create( pKlystron_->findByName("SysgenMR/FeedforwardA") ) ),
  gainP_(               IScalVal::create( pKlystron_->findByName("SysgenMR/GainP") ) ),
  gainA_(               IScalVal::create( pKlystron_->findByName("SysgenMR/GainA") ) ),
  feedbackCorrLimitP_(  IScalVal::create( pKlystron_->findByName("SysgenMR/FeedbackCorrLimitP") ) ),
  feedbackCorrLimitA_(  IScalVal::create( pKlystron_->findByName("SysgenMR/FeedbackCorrLimitA") ) ),
  integrationStart_(    IScalVal::create( pKlystron_->findByName("SysgenMR/IntegrationStart") ) ),
  integrationEnd_(      IScalVal::create( pKlystron_->findByName("SysgenMR/IntegrationEnd") ) ),
  applyCorrStart_(      IScalVal::create( pKlystron_->findByName("SysgenMR/ApplyCorrStart") ) ),
  applyCorrEnd_(        IScalVal::create( pKlystron_->findByName("SysgenMR/ApplyCorrEnd") ) ),
  dacOffset_(           IScalVal::create( pKlystron_->findByName("SysgenMR/DACOffset") ) ),
  dacOutLimitHi_(       IScalVal::create( pKlystron_->findByName("SysgenMR/DACLimitHi") ) ),
  dacOutLimitLow_(      IScalVal::create( pKlystron_->findByName("SysgenMR/DACLimitLow") ) ),
  debugStream0Select_ ( IScalVal::create( pKlystron_->findByName("SysgenMR/DebugStream0Select") ) ),
  debugStream1Select_ ( IScalVal::create( pKlystron_->findByName("SysgenMR/DebugStream1Select") ) ),
  debugStream2Select_ ( IScalVal::create( pKlystron_->findByName("SysgenMR/DebugStream2Select") ) ),
  debugStream3Select_ ( IScalVal::create( pKlystron_->findByName("SysgenMR/DebugStream3Select") ) ),
  debugStream4Select_ ( IScalVal::create( pKlystron_->findByName("SysgenMR/DebugStream4Select") ) ),
  debugStream5Select_ ( IScalVal::create( pKlystron_->findByName("SysgenMR/DebugStream5Select") ) ),
  setpointTableI_(      IScalVal::create( pKlystron_->findByName("SysGenWaveform_I/MemoryArray") ) ),
  setpointTableQ_(      IScalVal::create( pKlystron_->findByName("SysGenWaveform_Q/MemoryArray") ) ),
  setpointTableNelms_(  IScalVal::create( pKlystron_->findByName("SysgenMR/SetpointTableNelms") ) ),
  nonIQCoefCurr_(       IScalVal_RO::create( pKlystron_->findByName("SysgenMR/NonIQCoefLatch") ) ),
  nonIQCoefOffset_(     IScalVal::create( pKlystron_->findByName("SysgenMR/NonIQCoefOffset") ) ),
  sysgenMajorVersion_(  IScalVal_RO::create( pKlystron_->findByName("SysgenMR/MajorVersion") ) ),
  sysgenMinorVersion_(  IScalVal_RO::create( pKlystron_->findByName("SysgenMR/MinorVersion") ) ),
  fpgaVersion_(         IScalVal_RO::create(      p->findByName("AmcCarrierCore/AxiVersion/FpgaVersion") ) ),
  measuredTrigPeriod_(  IScalVal_RO::create( pKlystron_->findByName("SysgenMR/MeasuredTrigPeriod") ) ),
  trigCounter_(         IScalVal_RO::create( pKlystron_->findByName("SysgenMR/TriggerCounter") ) ),
  msgNoDelay_(          IScalVal::create( p->findByName("AmcCarrierCore/AmcCarrierTiming/TimingFrameRx/MsgNoDelay") ) ),
  trigHwArm_(           ICommand::create( p->findByName("AppTop/DaqMuxV2[1]/ArmHwTrigger") ) ),
  waveform0Init_(        ICommand::create( p->findByName("AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[0]/WaveformEngineBuffers/Initialize") ) ),
  waveform1Init_(        ICommand::create( p->findByName("AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine[1]/WaveformEngineBuffers/Initialize") ) ),
  daqSize_(           IScalVal::create( p->findByName("AppTop/DaqMuxV2/DataBufferSize") ) ),
  daqBufStartAddr_(           IScalVal::create( p->findByName("AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine/WaveformEngineBuffers/StartAddr") ) ),
  daqBufEndAddr_(           IScalVal::create( p->findByName("AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine/WaveformEngineBuffers/EndAddr") ) ),

//  debugPacketCnt_(            IScalVal_RO::create( p->findByName("AmcCarrierCore/SwRssiServer[1]/ValidCnt") ) ),
  debugTrgCnt0_(         IScalVal_RO::create( p->findByName("AppTop/DaqMuxV2[0]/TrigCount") ) ),
  debugTrgCnt1_(         IScalVal_RO::create( p->findByName("AppTop/DaqMuxV2[1]/TrigCount") ) ),
  
  atten0inBay0_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfDownConvert/AttHMC624[0]/SetValue") ) ),
  atten1inBay0_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfDownConvert/AttHMC624[1]/SetValue") ) ),
  atten2inBay0_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfDownConvert/AttHMC624[2]/SetValue") ) ),
  atten3inBay0_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfDownConvert/AttHMC624[3]/SetValue") ) ),
  atten4inBay0_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfDownConvert/AttHMC624[4]/SetValue") ) ),
  atten5inBay0_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfDownConvert/AttHMC624[5]/SetValue") ) ),
  
  atten0inBay1_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfUpConvert/AttHMC624[0]/SetValue") ) ),
  atten1inBay1_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfUpConvert/AttHMC624[1]/SetValue") ) ),
  atten2inBay1_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfUpConvert/AttHMC624[2]/SetValue") ) ),
  atten3inBay1_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfUpConvert/AttHMC624[3]/SetValue") ) ),
  //atten4inBay1_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfUpConvert/AttHMC624[4]/SetValue") ) ),   // bay1 doesn't have 4 and 5
  //atten5inBay1_(         IScalVal::create( pKlystron_->findByName("AmcMrLlrfUpConvert/AttHMC624[5]/SetValue") ) ),   // bay1 doens't have 4 and 5
  
  bay0Temp0MSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfDownConvert/Adt7420[0]/TempMSByte") ) ),
  bay0Temp0LSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfDownConvert/Adt7420[0]/TempLSByte") ) ),
  bay0Temp1MSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfDownConvert/Adt7420[1]/TempMSByte") ) ),
  bay0Temp1LSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfDownConvert/Adt7420[1]/TempLSByte") ) ),
  bay0Temp2MSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfDownConvert/Adt7420[2]/TempMSByte") ) ),
  bay0Temp2LSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfDownConvert/Adt7420[2]/TempLSByte") ) ),
  bay0Temp3MSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfDownConvert/Adt7420[3]/TempMSByte") ) ),
  bay0Temp3LSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfDownConvert/Adt7420[3]/TempLSByte") ) ),
  
  bay1Temp0MSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfUpConvert/Adt7420[0]/TempMSByte") ) ),
  bay1Temp0LSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfUpConvert/Adt7420[0]/TempLSByte") ) ),
  bay1Temp1MSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfUpConvert/Adt7420[1]/TempMSByte") ) ),
  bay1Temp1LSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfUpConvert/Adt7420[1]/TempLSByte") ) ),
  bay1Temp2MSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfUpConvert/Adt7420[2]/TempMSByte") ) ),
  bay1Temp2LSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfUpConvert/Adt7420[2]/TempLSByte") ) ),
  bay1Temp3MSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfUpConvert/Adt7420[3]/TempMSByte") ) ),
  bay1Temp3LSB_(         IScalVal_RO::create(pKlystron_->findByName("AmcMrLlrfUpConvert/Adt7420[3]/TempLSByte") ) ),
  
  intPhaseError_(        IScalVal_RO::create(pKlystron_->findByName("SysgenMR/IntPhaseError") ) ),    // phase error between ereference and feedback channel
  ckPhaseSet_(           IScalVal::create(pKlystron_->findByName("SysgenMR/CKPhaseSet") ) ),
  
  rtmStatus_(            IScalVal_RO::create( pKlystron_->findByName("RtmRfInterlock/Status") ) ),
  rtmFirmwareVersion_(   IScalVal_RO::create( pKlystron_->findByName("RtmRfInterlock/FirmwareVersion") ) ),
  rtmSystemId_(          IScalVal_RO::create( pKlystron_->findByName("RtmRfInterlock/SystemId") ) ),
  rtmSubType_(           IScalVal_RO::create( pKlystron_->findByName("RtmRfInterlock/SubType") ) ),
  rtmFirmwareDate_(      IScalVal_RO::create( pKlystron_->findByName("RtmRfInterlock/FirmwareDate") ) ),
  rtmKlyWiperRegA_(      IScalVal::create(pKlystron_->findByName("RtmRfInterlock/KlyWiperRegA") ) ),
  rtmKlyWiperRegB_(      IScalVal::create(pKlystron_->findByName("RtmRfInterlock/KlyWiperRegB") ) ),
  rtmKlyNVRegA_(         IScalVal::create(pKlystron_->findByName("RtmRfInterlock/KlyNVRegA") ) ),
  rtmKlyNVRegB_(         IScalVal::create(pKlystron_->findByName("RtmRfInterlock/KlyNVRegB") ) ),
  rtmModWiperRegA_(      IScalVal::create(pKlystron_->findByName("RtmRfInterlock/ModWiperRegA") ) ),
  rtmModWiperRegB_(      IScalVal::create(pKlystron_->findByName("RtmRfInterlock/ModWiperRegB") ) ),
  rtmModNVRegA_(         IScalVal::create(pKlystron_->findByName("RtmRfInterlock/ModNVRegA") ) ),
  rtmModNVRegB_(         IScalVal::create(pKlystron_->findByName("RtmRfInterlock/ModNVRegB") ) ),
  rtmCfgRegister_(       IScalVal::create(pKlystron_->findByName("RtmRfInterlock/CfgRegister") ) ),
  rtmFaultOutStatus_(    IScalVal_RO::create(pKlystron_->findByName("RtmRfInterlock/FaultOut") ) ),
  rtmAdcLockedStatus_(   IScalVal_RO::create(pKlystron_->findByName("RtmRfInterlock/AdcLocked") ) ),
  rtmRFOffStatus_(       IScalVal_RO::create(pKlystron_->findByName("RtmRfInterlock/RfOff") ) ),
  rtmAdcIn_(             IScalVal_RO::create(pKlystron_->findByName("RtmRfInterlock/AdcIn") ) ),
  rtmMode_(              IScalVal::create(pKlystron_->findByName("RtmRfInterlock/Mode") ) ),
  rtmTuneSled_(          IScalVal::create(pKlystron_->findByName("RtmRfInterlock/TuneSled") ) ),
  rtmDetuneSled_(        IScalVal::create(pKlystron_->findByName("RtmRfInterlock/DetuneSled") ) ),
  rtmAdcBufferBeamIV_(   IScalVal_RO::create(pKlystron_->findByName("RtmRfInterlock/RtmAdcBuffer[0]/MemoryArray") ) ),
  rtmAdcBufferFwdRef_(   IScalVal_RO::create(pKlystron_->findByName("RtmRfInterlock/RtmAdcBuffer[1]/MemoryArray") ) ),
  rtmRearm_Cmd_(         ICommand::create(pKlystron_->findByName("RtmRfInterlock/RearmTrigger") ) ),
  rtmSwTrigger_Cmd_(     ICommand::create(pKlystron_->findByName("RtmRfInterlock/SwTrigger") ) ),
  rtmClearFault_Cmd_(    ICommand::create(pKlystron_->findByName("RtmRfInterlock/ClearFault") ) )

//  stream0_(             IStream::create(      p->findByName("Stream0") ) ),
//  stream1_(             IStream::create(      p->findByName("Stream1") ) ),
//  stream2_(             IStream::create(      p->findByName("Stream2") ) ),
//  stream3_(             IStream::create(      p->findByName("Stream3") ) ),
//  stream4_(             IStream::create(      p->findByName("Stream4") ) ),
//  stream5_(             IStream::create(      p->findByName("Stream5") ) ),
//  stream6_(             IStream::create(      p->findByName("Stream6") ) ),
//  stream7_(             IStream::create(      p->findByName("Stream7") ) )
{

    debugStreamSelect_[STREAM_0] = debugStream0Select_;
    debugStreamSelect_[STREAM_1] = debugStream1Select_;
    debugStreamSelect_[STREAM_2] = debugStream2Select_;
    debugStreamSelect_[STREAM_3] = debugStream3Select_;
    debugStreamSelect_[STREAM_4] = debugStream4Select_;
    debugStreamSelect_[STREAM_5] = debugStream5Select_;
    
    
    atten_[0] = atten0inBay0_;
    atten_[1] = atten1inBay0_;
    atten_[2] = atten2inBay0_;
    atten_[3] = atten3inBay0_;
    atten_[4] = atten4inBay0_;
    atten_[5] = atten5inBay0_;
    
    atten_[6] = atten0inBay1_;
    atten_[7] = atten1inBay1_;
    atten_[8] = atten2inBay1_;
    atten_[9] = atten3inBay1_;
    atten_[10] = atten4inBay1_;
    atten_[11] = atten5inBay1_;


    tempBytes_[0] = bay0Temp0MSB_;
    tempBytes_[1] = bay0Temp0LSB_;
    tempBytes_[2] = bay0Temp1MSB_;
    tempBytes_[3] = bay0Temp1LSB_;
    tempBytes_[4] = bay0Temp2MSB_;
    tempBytes_[5] = bay0Temp2LSB_;
    tempBytes_[6] = bay0Temp3MSB_;
    tempBytes_[7] = bay0Temp3LSB_;
    
    tempBytes_[8] = bay1Temp0MSB_;
    tempBytes_[9] = bay1Temp0LSB_;
    tempBytes_[10] = bay1Temp1MSB_;
    tempBytes_[11] = bay1Temp1LSB_;
    tempBytes_[12] = bay1Temp2MSB_;
    tempBytes_[13] = bay1Temp2LSB_;
    tempBytes_[14] = bay1Temp3MSB_;
    tempBytes_[15] = bay1Temp3LSB_;
    
     
}

void CKlystronFwAdapt::reset()
{

}

void CKlystronFwAdapt::setLO_KP(uint32_t KP)
{
    try {
        LO_KP_->setVal(KP);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setLO_KI(uint32_t KI)
{
    try {
        LO_KI_->setVal(KI);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setCLK_KP(uint32_t KP)
{
    try {
        CLK_KP_->setVal(KP);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setCLK_KI(uint32_t KI)
{
    try {
        CLK_KI_->setVal(KI);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setADPLL_KP(uint32_t KP)
{
    try {
        ADPLL_KP_->setVal(KP);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setADPLL_KI(uint32_t KI)
{
    try {
        ADPLL_KI_->setVal(KI);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::OutputEnable(bool enable)
{
    try {
        OutputEnable_->setVal((uint32_t) enable);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}
void CKlystronFwAdapt::CWOutputEnable(bool enable)
{
    try {
        CWModeEnable_->setVal((uint32_t) enable);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setDebugStreamSelect(StreamId sid, StreamSel ssel)
{
    try {
        debugStreamSelect_[sid]->setVal(ssel);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}
/**
 * Set the external trigger (ACC/STDBY/SPARE) delay
 * Input:
 *   value_ns           : Delay value in ns
 *   freq_MHz           : EVR frequency in MHz
 */
 
void CKlystronFwAdapt::setTrigMode(TrigMode mode)
{
    try {
        trigMode_->setVal( (uint32_t) mode );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setTrigPolarity(Trigger trig, TrigPolarity polarity)
{
    return;
}

void CKlystronFwAdapt::setTrigDelay(Trigger trig, double value_ns, double freq_MHz)
{
    return;
}

void CKlystronFwAdapt::setTrigWidth(Trigger trig, double value_ns, double freq_MHz)
{
    return;
}

void CKlystronFwAdapt::setTrigOpCode(Trigger trig, std::vector<int> opCodes)
{
    return;
}


void CKlystronFwAdapt::setDaqSize( uint32_t buf_size )
{
	uint32_t nelms = (buf_size % 4096) ? (buf_size + 4096-(buf_size%4096)):buf_size;


	std::vector<uint32_t> startAddr(8, 0);
	std::vector<uint32_t> endAddr(8, 0);
	for ( unsigned i = 0; i < 8; i++ ) {
		startAddr[i] = 0 + ((uint32_t) 0x20000000)*i;
		endAddr[i] = 0 + ((uint32_t) 0x20000000)*i + nelms;
	}
	try {
		daqSize_->setVal( nelms/4 );
		daqBufStartAddr_->setVal( (uint32_t *) &startAddr[0], 8 );
		daqBufEndAddr_->setVal( (uint32_t *) &endAddr[0], 8 );
	} catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;

	}
}
/**
 * Select the reference and feedback channel. The higher 16 bits for reference channel and the lower 16 bits for feedback channel
 * Input:
 *   refCh              : The channel ID of the reference signal
 *   fbkCh              : The channel ID of the feedback signal
 */ 
void CKlystronFwAdapt::selectRefFbkChannel(uint32_t refCh, uint32_t fbkCh)
{
    /* writeRegister */  
    try {
        referenceChannel_->setVal( refCh );
        feedbackChannel_->setVal( fbkCh );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

/**
 * Set the reference phase set point for reference tracking. The input is in degree and will be nomalized +/-1 and
 *   coded by a 16-bit data with 13 bit of fraction
 * Input:
 *   phaSP_deg : phase setpoint in degree, +/- 180
 */
void CKlystronFwAdapt::setRefPhaSP(double phaSP_deg)
{
    int32_t pha = (int32_t)((phaSP_deg/180.0) * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_PHS_FRACTION));
    try {
        referencePhaseSet_->setVal( (uint32_t*) &pha, 1 );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

/**
 * Set the rotation coefficients for the feedback signal. The higher 16 bits is for cos and the lower 16 bits for sin. They all
 *   have 30 bits of fraction
 * Input:
 *   scale              : Scale factor of the feedback signal (should be in the range of [-1, 1])
 *   phase_deg       : Rotation angle (radian) of the feedback siganl
 */
void  CKlystronFwAdapt::setSWFeedbackCorrection(Timeslot timeslot, double amplitude_norm, double phase_deg)
{
    int32_t cs   = (int32_t)(amplitude_norm * cos( phase_deg * PI/180.0 ) * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_COR_COEF_FRACTION));
    int32_t sn   = (int32_t)(amplitude_norm * sin( phase_deg * -PI/180.0 ) * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_COR_COEF_FRACTION));    
    /* writeRegister */  
    try {
    
      switch (timeslot) {
          case ONE:
              ICorrSoftware_TS1_->setVal( (uint32_t*) &cs, 1 );
              QCorrSoftware_TS1_->setVal( (uint32_t*) &sn, 1 );
              break;
          case TWO:
              ICorrSoftware_TS2_->setVal( (uint32_t*) &cs, 1 );
              QCorrSoftware_TS2_->setVal( (uint32_t*) &sn, 1 );
              break;
          case THREE:
              ICorrSoftware_TS3_->setVal( (uint32_t*) &cs, 1 );
              QCorrSoftware_TS3_->setVal( (uint32_t*) &sn, 1 );
              break;
          case FOUR:
              ICorrSoftware_TS4_->setVal( (uint32_t*) &cs, 1 );
              QCorrSoftware_TS4_->setVal( (uint32_t*) &sn, 1 );
              break;
          case FIVE:
              ICorrSoftware_TS5_->setVal( (uint32_t*) &cs, 1 );
              QCorrSoftware_TS5_->setVal( (uint32_t*) &sn, 1 );
              break;
          case SIX:
              ICorrSoftware_TS6_->setVal( (uint32_t*) &cs, 1 );
              QCorrSoftware_TS6_->setVal( (uint32_t*) &sn, 1 );
              break;
      }
  
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

/**
 * Get the current rotation coefficients for the feedback signal. The higher 16 bits is for cos and the lower 16 bits for sin. They all
 *   have 30 bits of fraction
 * Input:
 *   scale              : Scale factor of the feedback signal (should be in the range of [-1, 1])
 *   phase_deg       : Rotation angle (radian) of the feedback siganl
 */
void  CKlystronFwAdapt::getSWFeedbackCorrection(Timeslot timeslot, double *amplitude_norm, double *phase_deg)
{    
    int32_t cs = 0;
    int32_t sn = 0;
    double cs_scale;
    double sn_scale;
    /* readRegisters */  
    try {
        switch(timeslot) {
            case ONE:
                ICorrSoftware_TS1_->getVal( (uint32_t*) &cs );
                QCorrSoftware_TS1_->getVal( (uint32_t*) &sn );
                break;
            case TWO:
                ICorrSoftware_TS2_->getVal( (uint32_t*) &cs );
                QCorrSoftware_TS2_->getVal( (uint32_t*) &sn );
                break;
            case THREE:
                ICorrSoftware_TS3_->getVal( (uint32_t*) &cs );
                QCorrSoftware_TS3_->getVal( (uint32_t*) &sn );
                break;
            case FOUR:
                ICorrSoftware_TS4_->getVal( (uint32_t*) &cs );
                QCorrSoftware_TS4_->getVal( (uint32_t*) &sn );
                break;
            case FIVE:
                ICorrSoftware_TS5_->getVal( (uint32_t*) &cs );
                QCorrSoftware_TS5_->getVal( (uint32_t*) &sn );
                break;
            case SIX:
                ICorrSoftware_TS6_->getVal( (uint32_t*) &cs );
                QCorrSoftware_TS6_->getVal( (uint32_t*) &sn );
                break;
        }
        
	cs_scale = cs * pow(2, -FWC_COMMON_PLATFORM_IQFB_CONST_COR_COEF_FRACTION);
	sn_scale = sn * pow(2, -FWC_COMMON_PLATFORM_IQFB_CONST_COR_COEF_FRACTION);
	*phase_deg = atan2( sn_scale, cs_scale ) * 180.0/PI;
	*amplitude_norm = sqrt( (cs_scale)*(cs_scale) + sn_scale*sn_scale );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::enableTimeslotCorrection(bool enable)
{
    try {
	enableTimeslotCorrection_->setVal( (uint32_t) enable );
    } catch (CPSWError &e ) {
	fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
	throw e;
    }
}

/**
 * Set the feed forward values
 * Input:
 *   phase                 : Feedforwad value in degrees, +/- 180
 *   amplitude             : Feedforward value, normalized 
 */
void  CKlystronFwAdapt::setFeedforward(double phase, double amplitude)
{
    /* FPGA treats phase as normalized +/- 1 */
    int32_t p = (int32_t)((phase/180.0) * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_GAIN_FRACTION));
    int32_t a = (int32_t)(amplitude * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_GAIN_FRACTION));
    /* writeRegister */ 
    try {
        feedforwardP_->setVal( (uint32_t*) &p, 1 );
        feedforwardA_->setVal( (uint32_t*) &a, 1 );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

/**
 * Set the gain for the feedback
 * Input:
 *   phase              : Gain value
 *   amplitude          : Gain value
 */
void  CKlystronFwAdapt::setGain(double phase, double amplitude)
{
    int32_t p = (int32_t)(phase * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_GAIN_FRACTION));
    int32_t a = (int32_t)(amplitude * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_GAIN_FRACTION));
    /* writeRegister */  
    try {
        gainP_->setVal( (uint32_t*) &p, 1 );
        gainA_->setVal( (uint32_t*) &a, 1 );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}


/**
 * Set the feedback correction limits
 */
void  CKlystronFwAdapt::setFeedbackCorrLimits(double phase, double amplitude)
{
    int32_t p = (int32_t)(phase * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_GAIN_FRACTION));
    int32_t a = (int32_t)(amplitude * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_GAIN_FRACTION));
    /* writeRegister */   
    try {
        feedbackCorrLimitP_->setVal( (uint32_t*)&p, 1 );
        feedbackCorrLimitA_->setVal( (uint32_t*)&a, 1 );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

/**
 * Set the time window for the integration
 * Input:
 *   value_ns           : Time value in ns
 *   freq_MHz           : Sampling frequency in MHz
 */
void  CKlystronFwAdapt::setIntgStart(double value_ns, double freq_MHz)
{
    uint32_t data = (uint32_t)(value_ns * ( freq_MHz / 64.0 ) / 1000.0);
    /* writeRegister */ 
    try {
        integrationStart_->setVal( data );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void  CKlystronFwAdapt::setIntgEnd(  double value_ns, double freq_MHz)
{
    uint32_t data = (uint32_t)(value_ns * ( freq_MHz / 64.0 ) / 1000.0);
    /* writeRegister */  
    try {
        integrationEnd_->setVal( data );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

/**
 * Set the time window for applying correction
 * Input:
 *   value_ns           : Time value in ns
 *   freq_MHz           : Sampling frequency in MHz
 */
void  CKlystronFwAdapt::setApplStart(double value_ns, double freq_MHz)
{
    uint32_t data = (uint32_t)(value_ns * freq_MHz / 1000.0);
    /* writeRegister */ 
    try {
        applyCorrStart_->setVal( data );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void  CKlystronFwAdapt::setApplEnd(  double value_ns, double freq_MHz)
{
    uint32_t data = (uint32_t)(value_ns * freq_MHz / 1000.0);
    /* writeRegister */ 
    try {
        applyCorrEnd_->setVal( data );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

/**
 * Set the offset of the DAC
 * Input:
 *   offset             : Offset in digits
 */
void  CKlystronFwAdapt::setDACOffset(int16_t offset)
{
    /* writeRegister */   
    try {
        dacOffset_->setVal( offset );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

/**
 * Set the limits for DAC output signal
 *   limit              : Limit value in digits
 */
void  CKlystronFwAdapt::setAmpLimit( int16_t high, int16_t low)
{
    /* writeRegister */   
    try {
        dacOutLimitHi_->setVal( high );
        dacOutLimitLow_->setVal( low );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
  
}

/**
 * Set the I/Q set point table.
 * Input:
 *   pno                : Number of the points
 *   ISPTable           : set point table for I
 *   QSPTable           : set point table for Q
 */
void  CKlystronFwAdapt::setIQSPTable(uint32_t pno, double *ISPTable, double *QSPTable)
{
    uint32_t i;
    std::vector<int16_t> Idata( FWC_COMMON_PLATFORM_IQFB_CONST_SP_TAB_BUF_DEPTH, 0 );
    std::vector<int16_t> Qdata( FWC_COMMON_PLATFORM_IQFB_CONST_SP_TAB_BUF_DEPTH, 0 );

    pno = pno > FWC_COMMON_PLATFORM_IQFB_CONST_SP_TAB_BUF_DEPTH ? FWC_COMMON_PLATFORM_IQFB_CONST_SP_TAB_BUF_DEPTH : pno;    
 
    for (i = 0; i < pno; i ++) {
    /* Make up the data */
        Idata[i] = (int16_t)((*(ISPTable + i)) * pow(2, 14) );
        Qdata[i] = (int16_t)((*(QSPTable + i)) * pow(2, 14) ); 
    }
    try {
        setpointTableI_->setVal( (uint16_t *) &(Idata[0]), FWC_COMMON_PLATFORM_IQFB_CONST_SP_TAB_BUF_DEPTH );
        setpointTableQ_->setVal( (uint16_t *) &(Qdata[0]), FWC_COMMON_PLATFORM_IQFB_CONST_SP_TAB_BUF_DEPTH );
        setpointTableNelms_->setVal( pno );
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }

}

/** 
 * Set the non-IQ coefficient index offset value (will be valid only you press the apply button)
 */
void  CKlystronFwAdapt::setNonIQCoefOffset(uint32_t offset)
{
    nonIQCoefOffset_->setVal( offset );  
}

/* Fimrware readings */
void CKlystronFwAdapt::getAppFwInfo(uint32_t *firmwareName, uint32_t *majorVer, uint32_t *minorVer, uint32_t *buildNum)
{
    /* read register var_version */
    sysgenMajorVersion_->getVal( majorVer );
    sysgenMinorVersion_->getVal( minorVer );
    fpgaVersion_->getVal( buildNum );
}

void  CKlystronFwAdapt::getRaceConditionFlags(uint32_t *accFlags, uint32_t *stdbyFlags, uint32_t *spareFlags)
{
  
}

void CKlystronFwAdapt::getADCValid(bool *valid)
{
    *valid = true;
}

void  CKlystronFwAdapt::getPulseCounter(uint32_t *pulseCnt)
{
    trigCounter_->getVal( pulseCnt ); 
}

void  CKlystronFwAdapt::getMeaTrigPeriod(double *value_ms, double freq_MHz)
{
    uint32_t u32;
    measuredTrigPeriod_->getVal( &u32 );

    *value_ms = u32 / ( freq_MHz * 1000.0 );
}

void  CKlystronFwAdapt::getNonIQCoefCur(uint32_t *cur)
{
    nonIQCoefCurr_->getVal( cur );
}



void  CKlystronFwAdapt::getFwStatus(uint32_t *platformStatus, uint32_t *RFCtrlStatus)
{
  
}


void CKlystronFwAdapt::msgNoDelay(uint32_t no_delay)
{
    uint32_t zero(0), one(1);

    if(no_delay) msgNoDelay_->setVal(&one);
    else         msgNoDelay_->setVal(&zero);

}

void CKlystronFwAdapt::armDaq()
{
    //waveformInit_->setVal( (uint64_t) 1 );
    //waveformInit_->setVal( (uint64_t) 0 );
    trigHwArm_->execute();
}

void CKlystronFwAdapt::initBuf(void)
{

//    waveformInit_->setVal( (uint64_t) 0 );
//    waveformInit_->setVal( (uint64_t) 1 );
    waveform0Init_->execute();
    waveform1Init_->execute();
}


void CKlystronFwAdapt::loadConfigFromYamlFile( const char *filename, const char *yaml_dir)
{
    YAML::Node config( CYamlFieldFactoryBase::loadPreprocessedYamlFile( filename, yaml_dir ) );
    p_->loadConfigFromYaml( config );
}

void CKlystronFwAdapt::dumpConfigToYamlFile( const char *filename, const char *yaml_dir)
{
    YAML::Node config;
    p_->dumpConfigToYaml( config );

    YAML::Emitter emit;
    emit << config;

    std::fstream strm( filename, std::fstream::out );
    strm << emit.c_str() << "\n";
}


KlystronFw IKlystronFw::create(Path p)
{
    //KlystronFwAdapt rval = IEntryAdapt::check_interface<KlystronFwAdapt, Entry>(p);
    //return rval;
    return IEntryAdapt::check_interface<KlystronFwAdapt, DevImpl>( p );
}


//

void CKlystronFwAdapt::getDebugPacketCnt(uint32_t *dbgCnt)
{
    try {
//        debugPacketCnt_->getVal(dbgCnt);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::getDebugTrgCnt(uint32_t *dbgCnt0, uint32_t *dbgCnt1)
{
    try{
      debugTrgCnt0_->getVal(dbgCnt0);
      debugTrgCnt1_->getVal(dbgCnt1);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}


void CKlystronFwAdapt::setAtten(uint32_t att, int bay, int chn)
{
    int index = bay*6 + chn;
    if(index >= 10) return; // bay1 chn 4 and chn 5 are not available.
    try {
        atten_[index]->setVal(att);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setAtten(uint32_t att, int index)
{
    if(index > 9) return;  // bay1 chn 4 and chn5 are not available.
    try {
        atten_[index]->setVal(att);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }

}

void CKlystronFwAdapt::getTemp(int bay, int sensor, double *temp)
{
    int index = bay*4 + sensor;
    uint8_t MSB, LSB;
    try {
        tempBytes_[index*2]->getVal(&MSB);
        tempBytes_[index*2+1]->getVal(&LSB);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
    
    if (MSB & 0x80) { // if sign bit is set, negative
        *temp = ( (MSB<<8) + LSB - 0x10000 ) / 128.0;
    }
    else { // else positive
        *temp = ( (MSB<<8) + LSB ) / 128.0;
    }

}

void CKlystronFwAdapt::getTemp(int index, double *temp)
{
    uint8_t MSB, LSB;
    try {
        tempBytes_[index*2]->getVal(&MSB);
        tempBytes_[index*2+1]->getVal(&LSB);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
    
    if (MSB & 0x80) { // if sign bit is set, negative
        *temp = ( (MSB<<8) + LSB - 0x10000 ) / 128.0;
    }
    else { // else positive
        *temp = ( (MSB<<8) + LSB ) / 128.0;
    }

}

void CKlystronFwAdapt::getIntPhaseError(int32_t *phase)
{
    try {
        intPhaseError_->getVal((uint32_t*)phase);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setCKPhase(uint32_t ckPhase)
{
    try {
        ckPhaseSet_->setVal(ckPhase);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
} 


void CKlystronFwAdapt::getRtmStatus(uint32_t *rtmStatus)
{
    try {
        rtmStatus_->getVal(rtmStatus);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}


void CKlystronFwAdapt::getRtmFirmwareVersion(uint32_t *rtmFirmwareVersion)
{
    try {
        rtmFirmwareVersion_->getVal(rtmFirmwareVersion);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}


void CKlystronFwAdapt::getRtmSystemId(char *systemIdString)
{
    uint16_t id[4];
    char *c;
    int i;

    try {
        rtmSystemId_->getVal(id, 4);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }

    c = (char*) id;

    for(i=0;i<4; i++) {   // swapped byte order for string
        *(systemIdString +(i*2)) = *(c+(i*2+1));
        *(systemIdString +(i*2+1)) = *(c+(i*2));
    }

    *(systemIdString + (i*2)) = '\0';
    
}


void CKlystronFwAdapt::getRtmSubType(char *subTypeString)
{
    uint16_t stype[4];
    char *c;
    int i;

    try {
        rtmSubType_->getVal(stype, 4);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }

    c = (char *) stype;

    for(i=0; i<4; i++) {    // swapped byte order for string
        *(subTypeString+(i*2)) = *(c+(i*2+1));
        *(subTypeString+(i*2+1)) = *(c+(i*2));
    }
    *(subTypeString +(i*2)) = '\0';
}


void CKlystronFwAdapt::getRtmFirmwareDate(char *firmwareDateString)
{
    uint16_t sdate[4];
    char *c;
    int i;
    
    try {
        rtmFirmwareDate_->getVal(sdate,4);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }

    c = (char*) sdate;

    for(i=0; i<4; i++) {    // swapped byte order for string
        *(firmwareDateString + (i*2)) = *(c+(i*2+1));
        *(firmwareDateString + (i*2+1)) = *(c+(i*2));
    }

    *(firmwareDateString+(i*2)) = '\0';

}

void CKlystronFwAdapt::setRtmKlyWiperRegA(uint32_t reg)
{
    try {
        rtmKlyWiperRegA_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmKlyWiperRegB(uint32_t reg)
{
    try {
        rtmKlyWiperRegB_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmKlyNVRegA(uint32_t reg)
{

    try {
        rtmKlyNVRegA_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmKlyNVRegB(uint32_t reg)
{
    try {
        rtmKlyNVRegB_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmModWiperRegA(uint32_t reg)
{
    try {
        rtmModWiperRegA_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmModWiperRegB(uint32_t reg)
{
    try {
        rtmModWiperRegB_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmModNVRegA(uint32_t reg)
{
    try {
        rtmModNVRegA_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmModNVRegB(uint32_t reg)
{
    try {
        rtmModNVRegB_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmCfgRegister(uint32_t reg)
{
    try {
        rtmCfgRegister_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::getRtmFaultOutStatus(uint32_t *rtmFaultOutStatus)
{
    try {
        rtmFaultOutStatus_->getVal(rtmFaultOutStatus);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::getRtmAdcLockedStatus(uint32_t *rtmAdcLockedStatus)
{    
    try {
        rtmAdcLockedStatus_->getVal(rtmAdcLockedStatus);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::getRtmRFOffStatus(uint32_t *rtmRFOffStatus)
{
    try {
        rtmRFOffStatus_->getVal(rtmRFOffStatus);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::getRtmAdcIn(uint32_t v[])
{
    try {
        rtmAdcIn_->getVal(v,4);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmMode(uint32_t reg)
{
    try {
        rtmMode_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::setRtmDesiredSled(bool tune)
{
    try {
        if(tune) {
            rtmDetuneSled_->setVal( (uint64_t) 0);
            rtmTuneSled_->setVal( (uint64_t) 1);
        }
        else {
            rtmTuneSled_->setVal( (uint64_t) 0);
            rtmDetuneSled_->setVal( (uint64_t) 1);
        }
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::getRtmFastAdcBufferBeamCurrentVoltage(uint32_t v[])
{
    try {
        rtmAdcBufferBeamIV_->getVal(v, (unsigned) 0x200);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::getRtmFastAdcBufferForwardReflect(uint32_t v[])
{
    try {
        rtmAdcBufferFwdRef_->getVal(v, (unsigned) 0x200);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::cmdRtmRearm(void)
{
    try {
        rtmRearm_Cmd_->execute();
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::cmdRtmSwTrigger(void)
{
    try {
        rtmSwTrigger_Cmd_->execute();
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlystronFwAdapt::cmdRtmClearFault(void)
{
    try {
        rtmClearFault_Cmd_->execute();
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}
