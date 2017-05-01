#include "KlysMrFw.h"

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


class CKlysMrFwAdapt;
typedef shared_ptr<CKlysMrFwAdapt> KlysMrFwAdapt;

class CKlysMrFwAdapt : public IKlysMrFw, public IEntryAdapt {
protected:
    Path pKlys_;
/* put ScalVals, etc. here */

    ScalVal    trigMode_;
    
    TrigPulse  trigKlysAccel_;
    TrigPulse  trigKlysStandby_;
    TrigPulse  trigModAccel_;
    TrigPulse  trigModStandby_;
    TrigPulse  trigRtmDaqAccel_;
    TrigPulse  trigRtmDaqStandby_;
    TrigPulse  trigSsbAccel_;
    TrigPulse  trigSsbStandby_;
    
    ScalVal    OutputEnable_; 
    ScalVal    CWModeEnable_; 
    ScalVal    referenceChannel_; 
    ScalVal    feedbackChannel_;
    ScalVal    referencePhaseSet_; 
    ScalVal    feedbackRotI_;
    ScalVal    feedbackRotQ_;
    ScalVal    ICorrSoftware_;
    ScalVal    QCorrSoftware_;
    ScalVal    ICorrSoftwareTimeslot4_;
    ScalVal    QCorrSoftwareTimeslot4_;
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
    ScalVal    trigHwArm_;
    ScalVal    waveformInit_;
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

    std::map<Trigger, TrigPulse> triggers_;

    std::map<StreamId, ScalVal> debugStreamSelect_;
    
    std::map<int, ScalVal> atten_;
    
    
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
    ScalVal_RO   rtmAdcBufferBeamIV_;
    ScalVal_RO   rtmAdcBufferFwdRef_;
    Command      rtmRearm_Cmd_;
    Command      rtmSwTrigger_Cmd_;
    Command      rtmClearFault_Cmd_;


public:
        CKlysMrFwAdapt(Key &k, Path p, shared_ptr<const CEntryImpl> ie);

public:
    virtual void reset();
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

    virtual void armDaq();
    virtual void initBuf(void);

    virtual void loadConfigFromYamlFile( const char *filename, const char *yaml_dir = 0 );
    virtual void dumpConfigToYamlFile(   const char *filename, const char *yaml_dir = 0 );


    virtual void getDebugPacketCnt(uint32_t *dbgCnt);
    virtual void getDebugTrgCnt(uint32_t *dbgCnt0, uint32_t *dbgCnt1);
    
    virtual void setAtten(uint32_t att, int bay, int chn);
    virtual void setAtten(uint32_t att, int index);
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
    virtual void getRtmFastAdcBufferBeamCurrentVoltage(uint32_t v[]);
    virtual void getRtmFastAdcBufferForwardReflect(uint32_t v[]);
    virtual void cmdRtmRearm(void);
    virtual void cmdRtmSwTrigger(void);
    virtual void cmdRtmClearFault(void);
};

CKlysMrFwAdapt::CKlysMrFwAdapt(Key &k, Path p, shared_ptr<const CEntryImpl> ie) : 
  IEntryAdapt(k, p, ie),
  pKlys_(                                      p->findByName("AmcCarrierMrEth/AmcCarrierMrKlysApp") ),
  
  trigMode_          (  IScalVal::create( pKlys_->findByName("TimingCore/Mode") ) ),
  trigKlysAccel_(       ITrigPulse::create( pKlys_->findByName("TimingCore/TrigKlysAccel") ) ),
  trigKlysStandby_(     ITrigPulse::create( pKlys_->findByName("TimingCore/TrigKlysStdby") ) ),
  trigModAccel_(        ITrigPulse::create( pKlys_->findByName("TimingCore/TrigRtmModAccel") ) ),
  trigModStandby_(      ITrigPulse::create( pKlys_->findByName("TimingCore/TrigRtmModStdby") ) ),
  trigRtmDaqAccel_(     ITrigPulse::create( pKlys_->findByName("TimingCore/TrigRtmDaqAccel") ) ),
  trigRtmDaqStandby_(   ITrigPulse::create( pKlys_->findByName("TimingCore/TrigRtmDaqStdby") ) ),
  trigSsbAccel_(        ITrigPulse::create( pKlys_->findByName("TimingCore/TrigRtmSsbAccel") ) ),
  trigSsbStandby_(      ITrigPulse::create( pKlys_->findByName("TimingCore/TrigRtmSsbStdby") ) ),
  OutputEnable_(        IScalVal::create( pKlys_->findByName("SysgenMR/OutputEnable") ) ),
  CWModeEnable_(        IScalVal::create( pKlys_->findByName("SysgenMR/CWModeEnable") ) ),
  referenceChannel_(    IScalVal::create( pKlys_->findByName("SysgenMR/ReferenceChannel") ) ),
  feedbackChannel_(     IScalVal::create( pKlys_->findByName("SysgenMR/FeedbackChannel") ) ),
  referencePhaseSet_(   IScalVal::create( pKlys_->findByName("SysgenMR/ReferencePhaseSet") ) ),
  feedbackRotI_(        IScalVal::create( pKlys_->findByName("SysgenMR/FeedbackRotI") ) ),
  feedbackRotQ_(        IScalVal::create( pKlys_->findByName("SysgenMR/FeedbackRotQ") ) ),
  ICorrSoftware_(       IScalVal::create( pKlys_->findByName("SysgenMR/ICorrSoftware") ) ),
  QCorrSoftware_(       IScalVal::create( pKlys_->findByName("SysgenMR/QCorrSoftware") ) ),
  ICorrSoftwareTimeslot4_(IScalVal::create( pKlys_->findByName("SysgenMR/ICorrSoftware_TS4") ) ),
  QCorrSoftwareTimeslot4_(IScalVal::create( pKlys_->findByName("SysgenMR/QCorrSoftware_TS4") ) ),
  enableTimeslotCorrection_(IScalVal::create( pKlys_->findByName("SysgenMR/EnableTimeslotCorrection") ) ),
  feedforwardP_(        IScalVal::create( pKlys_->findByName("SysgenMR/FeedforwardP") ) ),
  feedforwardA_(        IScalVal::create( pKlys_->findByName("SysgenMR/FeedforwardA") ) ),
  gainP_(               IScalVal::create( pKlys_->findByName("SysgenMR/GainP") ) ),
  gainA_(               IScalVal::create( pKlys_->findByName("SysgenMR/GainA") ) ),
  feedbackCorrLimitP_(  IScalVal::create( pKlys_->findByName("SysgenMR/FeedbackCorrLimitP") ) ),
  feedbackCorrLimitA_(  IScalVal::create( pKlys_->findByName("SysgenMR/FeedbackCorrLimitA") ) ),
  integrationStart_(    IScalVal::create( pKlys_->findByName("SysgenMR/IntegrationStart") ) ),
  integrationEnd_(      IScalVal::create( pKlys_->findByName("SysgenMR/IntegrationEnd") ) ),
  applyCorrStart_(      IScalVal::create( pKlys_->findByName("SysgenMR/ApplyCorrStart") ) ),
  applyCorrEnd_(        IScalVal::create( pKlys_->findByName("SysgenMR/ApplyCorrEnd") ) ),
  dacOffset_(           IScalVal::create( pKlys_->findByName("SysgenMR/DACOffset") ) ),
  dacOutLimitHi_(       IScalVal::create( pKlys_->findByName("SysgenMR/DACLimitHi") ) ),
  dacOutLimitLow_(      IScalVal::create( pKlys_->findByName("SysgenMR/DACLimitLow") ) ),
  debugStream0Select_ ( IScalVal::create( pKlys_->findByName("SysgenMR/DebugStream0Select") ) ),
  debugStream1Select_ ( IScalVal::create( pKlys_->findByName("SysgenMR/DebugStream1Select") ) ),
  debugStream2Select_ ( IScalVal::create( pKlys_->findByName("SysgenMR/DebugStream2Select") ) ),
  debugStream3Select_ ( IScalVal::create( pKlys_->findByName("SysgenMR/DebugStream3Select") ) ),
  debugStream4Select_ ( IScalVal::create( pKlys_->findByName("SysgenMR/DebugStream4Select") ) ),
  debugStream5Select_ ( IScalVal::create( pKlys_->findByName("SysgenMR/DebugStream5Select") ) ),
  setpointTableI_(      IScalVal::create( pKlys_->findByName("SysGenWaveform_I/MemoryArray") ) ),
  setpointTableQ_(      IScalVal::create( pKlys_->findByName("SysGenWaveform_Q/MemoryArray") ) ),
  setpointTableNelms_(  IScalVal::create( pKlys_->findByName("SysgenMR/SetpointTableNelms") ) ),
  nonIQCoefCurr_(       IScalVal_RO::create( pKlys_->findByName("SysgenMR/NonIQCoefLatch") ) ),
  nonIQCoefOffset_(     IScalVal::create( pKlys_->findByName("SysgenMR/NonIQCoefOffset") ) ),
  sysgenMajorVersion_(  IScalVal_RO::create( pKlys_->findByName("SysgenMR/MajorVersion") ) ),
  sysgenMinorVersion_(  IScalVal_RO::create( pKlys_->findByName("SysgenMR/MinorVersion") ) ),
  fpgaVersion_(         IScalVal_RO::create(      p->findByName("AmcCarrierMrEth/AmcCarrierCore/AxiVersion/FpgaVersion") ) ),
  measuredTrigPeriod_(  IScalVal_RO::create( pKlys_->findByName("SysgenMR/MeasuredTrigPeriod") ) ),
  trigCounter_(         IScalVal_RO::create( pKlys_->findByName("SysgenMR/TriggerCounter") ) ),
  trigHwArm_(           IScalVal::create( pKlys_->findByName("DaqMuxV2[1]/TriggerHwArm") ) ),
  waveformInit_(        IScalVal::create( p->findByName("AmcCarrierMrEth/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine/WaveformEngineBuffers/Init") ) ),
  daqSize_(           IScalVal::create( pKlys_->findByName("DaqMuxV2/DataBufferSize") ) ),
  daqBufStartAddr_(           IScalVal::create( p->findByName("AmcCarrierMrEth/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine/WaveformEngineBuffers/StartAddr") ) ),
  daqBufEndAddr_(           IScalVal::create( p->findByName("AmcCarrierMrEth/AmcCarrierCore/AmcCarrierBsa/BsaWaveformEngine/WaveformEngineBuffers/EndAddr") ) ),



  debugPacketCnt_(            IScalVal_RO::create( p->findByName("AmcCarrierMrEth/AmcCarrierCore/RssiServerSw[1]/ValidCnt") ) ),
  debugTrgCnt0_(         IScalVal_RO::create( p->findByName("AmcCarrierMrEth/AmcCarrierMrKlysApp/DaqMuxV2[0]/TrigCount") ) ),
  debugTrgCnt1_(         IScalVal_RO::create( p->findByName("AmcCarrierMrEth/AmcCarrierMrKlysApp/DaqMuxV2[1]/TrigCount") ) ),
  
  atten0inBay0_(         IScalVal::create( pKlys_->findByName("AmcBay0Core/AttHMC624[0]/SetValue") ) ),
  atten1inBay0_(         IScalVal::create( pKlys_->findByName("AmcBay0Core/AttHMC624[1]/SetValue") ) ),
  atten2inBay0_(         IScalVal::create( pKlys_->findByName("AmcBay0Core/AttHMC624[2]/SetValue") ) ),
  atten3inBay0_(         IScalVal::create( pKlys_->findByName("AmcBay0Core/AttHMC624[3]/SetValue") ) ),
  atten4inBay0_(         IScalVal::create( pKlys_->findByName("AmcBay0Core/AttHMC624[4]/SetValue") ) ),
  atten5inBay0_(         IScalVal::create( pKlys_->findByName("AmcBay0Core/AttHMC624[5]/SetValue") ) ),
  
  atten0inBay1_(         IScalVal::create( pKlys_->findByName("AmcBay1Core/AttHMC624[0]/SetValue") ) ),
  atten1inBay1_(         IScalVal::create( pKlys_->findByName("AmcBay1Core/AttHMC624[1]/SetValue") ) ),
  atten2inBay1_(         IScalVal::create( pKlys_->findByName("AmcBay1Core/AttHMC624[2]/SetValue") ) ),
  atten3inBay1_(         IScalVal::create( pKlys_->findByName("AmcBay1Core/AttHMC624[3]/SetValue") ) ),
  //atten4inBay1_(         IScalVal::create( pKlys_->findByName("AmcBay1Core/AttHMC624[4]/SetValue") ) ),   // bay1 doesn't have 4 and 5
  //atten5inBay1_(         IScalVal::create( pKlys_->findByName("AmcBay1Core/AttHMC624[5]/SetValue") ) ),   // bay1 doens't have 4 and 5
  
  intPhaseError_(        IScalVal_RO::create(pKlys_->findByName("SysgenMR/IntPhaseError") ) ),    // phase error between ereference and feedback channel
  ckPhaseSet_(           IScalVal::create(pKlys_->findByName("SysgenMR/CKPhaseSet") ) ),
  
  rtmStatus_(            IScalVal_RO::create( pKlys_->findByName("RtmKlysCore/Status") ) ),
  rtmFirmwareVersion_(   IScalVal_RO::create( pKlys_->findByName("RtmKlysCore/FirmwareVersion") ) ),
  rtmSystemId_(          IScalVal_RO::create( pKlys_->findByName("RtmKlysCore/SystemId") ) ),
  rtmSubType_(           IScalVal_RO::create( pKlys_->findByName("RtmKlysCore/SubType") ) ),
  rtmFirmwareDate_(      IScalVal_RO::create( pKlys_->findByName("RtmKlysCore/FirmwareDate") ) ),
  rtmKlyWiperRegA_(      IScalVal::create(pKlys_->findByName("RtmKlysCore/KlyWiperRegA") ) ),
  rtmKlyWiperRegB_(      IScalVal::create(pKlys_->findByName("RtmKlysCore/KlyWiperRegB") ) ),
  rtmKlyNVRegA_(         IScalVal::create(pKlys_->findByName("RtmKlysCore/KlyNVRegA") ) ),
  rtmKlyNVRegB_(         IScalVal::create(pKlys_->findByName("RtmKlysCore/KlyNVRegB") ) ),
  rtmModWiperRegA_(      IScalVal::create(pKlys_->findByName("RtmKlysCore/ModWiperRegA") ) ),
  rtmModWiperRegB_(      IScalVal::create(pKlys_->findByName("RtmKlysCore/ModWiperRegB") ) ),
  rtmModNVRegA_(         IScalVal::create(pKlys_->findByName("RtmKlysCore/ModNVRegA") ) ),
  rtmModNVRegB_(         IScalVal::create(pKlys_->findByName("RtmKlysCore/ModNVRegB") ) ),
  rtmCfgRegister_(       IScalVal::create(pKlys_->findByName("RtmKlysCore/CfgRegister") ) ), 
  rtmFaultOutStatus_(    IScalVal_RO::create(pKlys_->findByName("RtmKlysCore/FaultOut") ) ),
  rtmAdcLockedStatus_(   IScalVal_RO::create(pKlys_->findByName("RtmKlysCore/AdcLocked") ) ),
  rtmRFOffStatus_(       IScalVal_RO::create(pKlys_->findByName("RtmKlysCore/RfOff") ) ),
  rtmAdcIn_(             IScalVal_RO::create(pKlys_->findByName("RtmKlysCore/AdcIn") ) ),
  rtmMode_(              IScalVal::create(pKlys_->findByName("RtmKlysCore/Mode") ) ),
  rtmAdcBufferBeamIV_(   IScalVal_RO::create(pKlys_->findByName("RtmKlysCore/RtmAdcBuffer[0]/MemoryArray") ) ),
  rtmAdcBufferFwdRef_(   IScalVal_RO::create(pKlys_->findByName("RtmKlysCore/RtmAdcBuffer[1]/MemoryArray") ) ),
  rtmRearm_Cmd_(         ICommand::create(pKlys_->findByName("RtmKlysCore/RearmTrigger") ) ),
  rtmSwTrigger_Cmd_(     ICommand::create(pKlys_->findByName("RtmKlysCore/SwTrigger") ) ),
  rtmClearFault_Cmd_(    ICommand::create(pKlys_->findByName("RtmKlysCore/ClearFault") ) )

//  stream0_(             IStream::create(      p->findByName("Stream0") ) ),
//  stream1_(             IStream::create(      p->findByName("Stream1") ) ),
//  stream2_(             IStream::create(      p->findByName("Stream2") ) ),
//  stream3_(             IStream::create(      p->findByName("Stream3") ) ),
//  stream4_(             IStream::create(      p->findByName("Stream4") ) ),
//  stream5_(             IStream::create(      p->findByName("Stream5") ) ),
//  stream6_(             IStream::create(      p->findByName("Stream6") ) ),
//  stream7_(             IStream::create(      p->findByName("Stream7") ) )
{

    triggers_[ KLYS_ACCEL ] = trigKlysAccel_;
    triggers_[ KLYS_STDBY ] = trigKlysStandby_;
    triggers_[ MOD_ACCEL ]  = trigModAccel_;
    triggers_[ MOD_STDBY ]  = trigModStandby_;
    triggers_[ SSB_ACCEL ]  = trigSsbAccel_;
    triggers_[ SSB_STDBY ]  = trigSsbStandby_;
    triggers_[ RTM_ACCEL ]  = trigRtmDaqAccel_;
    triggers_[ RTM_STDBY ]  = trigRtmDaqStandby_;

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

    
     
}

void CKlysMrFwAdapt::reset()
{

}

void CKlysMrFwAdapt::OutputEnable(bool enable)
{
    try {
        OutputEnable_->setVal((uint32_t) enable);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}
void CKlysMrFwAdapt::CWOutputEnable(bool enable)
{
    try {
        CWModeEnable_->setVal((uint32_t) enable);
    } catch (CPSWError &e) {
        fprintf(stderr, "CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setDebugStreamSelect(StreamId sid, StreamSel ssel)
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
 
void CKlysMrFwAdapt::setTrigMode(TrigMode mode)
{
    try {
        trigMode_->setVal( (uint32_t) mode );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setTrigPolarity(Trigger trig, TrigPolarity polarity)
{
    try {
        triggers_[trig]->setTrigPolarity( (ITrigPulse::TrigPolarity) polarity );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setTrigDelay(Trigger trig, double value_ns, double freq_MHz)
{
    try {
        triggers_[trig]->setTrigDelay( value_ns, freq_MHz );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setTrigWidth(Trigger trig, double value_ns, double freq_MHz)
{
    try {
        triggers_[trig]->setTrigWidth( value_ns, freq_MHz );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setTrigOpCode(Trigger trig, std::vector<int> opCodes)
{

    try {
        triggers_[trig]->setTrigOpCode( opCodes );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}


void CKlysMrFwAdapt::setDaqSize( uint32_t buf_size )
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
void CKlysMrFwAdapt::selectRefFbkChannel(uint32_t refCh, uint32_t fbkCh)
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
void CKlysMrFwAdapt::setRefPhaSP(double phaSP_deg)
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
void  CKlysMrFwAdapt::setSWFeedbackCorrection(Timeslot timeslot, double amplitude_norm, double phase_deg)
{
    int32_t cs   = (int32_t)(amplitude_norm * cos( phase_deg * PI/180.0 ) * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_COR_COEF_FRACTION));
    int32_t sn   = (int32_t)(amplitude_norm * sin( phase_deg * PI/180.0 ) * pow(2, FWC_COMMON_PLATFORM_IQFB_CONST_COR_COEF_FRACTION));    
    /* writeRegister */  
    try {
	if ( timeslot == FOUR ) {
	    ICorrSoftwareTimeslot4_->setVal( (uint32_t*) &cs, 1 );
            QCorrSoftwareTimeslot4_->setVal( (uint32_t*) &sn, 1 );
	} else {
            ICorrSoftware_->setVal( (uint32_t*) &cs, 1 );
            QCorrSoftware_->setVal( (uint32_t*) &sn, 1 );
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
void  CKlysMrFwAdapt::getSWFeedbackCorrection(Timeslot timeslot, double *amplitude_norm, double *phase_deg)
{    
    int32_t cs = 0;
    int32_t sn = 0;
    double cs_scale;
    double sn_scale;
    /* readRegisters */  
    try {
	if ( timeslot == FOUR ) {
	    ICorrSoftwareTimeslot4_->getVal( (uint32_t*) &cs );
            QCorrSoftwareTimeslot4_->getVal( (uint32_t*) &sn );
	} else {
            ICorrSoftware_->getVal( (uint32_t*) &cs );
            QCorrSoftware_->getVal( (uint32_t*) &sn );
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

void CKlysMrFwAdapt::enableTimeslotCorrection(bool enable)
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
void  CKlysMrFwAdapt::setFeedforward(double phase, double amplitude)
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
void  CKlysMrFwAdapt::setGain(double phase, double amplitude)
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
void  CKlysMrFwAdapt::setFeedbackCorrLimits(double phase, double amplitude)
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
void  CKlysMrFwAdapt::setIntgStart(double value_ns, double freq_MHz)
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

void  CKlysMrFwAdapt::setIntgEnd(  double value_ns, double freq_MHz)
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
void  CKlysMrFwAdapt::setApplStart(double value_ns, double freq_MHz)
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

void  CKlysMrFwAdapt::setApplEnd(  double value_ns, double freq_MHz)
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
void  CKlysMrFwAdapt::setDACOffset(int16_t offset)
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
void  CKlysMrFwAdapt::setAmpLimit( int16_t high, int16_t low)
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
void  CKlysMrFwAdapt::setIQSPTable(uint32_t pno, double *ISPTable, double *QSPTable)
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
void  CKlysMrFwAdapt::setNonIQCoefOffset(uint32_t offset)
{
    nonIQCoefOffset_->setVal( offset );  
}

/* Fimrware readings */
void CKlysMrFwAdapt::getAppFwInfo(uint32_t *firmwareName, uint32_t *majorVer, uint32_t *minorVer, uint32_t *buildNum)
{
    /* read register var_version */
    sysgenMajorVersion_->getVal( majorVer );
    sysgenMinorVersion_->getVal( minorVer );
    fpgaVersion_->getVal( buildNum );
}

void  CKlysMrFwAdapt::getRaceConditionFlags(uint32_t *accFlags, uint32_t *stdbyFlags, uint32_t *spareFlags)
{
  
}

void CKlysMrFwAdapt::getADCValid(bool *valid)
{
    *valid = true;
}

void  CKlysMrFwAdapt::getPulseCounter(uint32_t *pulseCnt)
{
    trigCounter_->getVal( pulseCnt ); 
}

void  CKlysMrFwAdapt::getMeaTrigPeriod(double *value_ms, double freq_MHz)
{
    uint32_t u32;
    measuredTrigPeriod_->getVal( &u32 );

    *value_ms = u32 / ( freq_MHz * 1000.0 );
}

void  CKlysMrFwAdapt::getNonIQCoefCur(uint32_t *cur)
{
    nonIQCoefCurr_->getVal( cur );
}



void  CKlysMrFwAdapt::getFwStatus(uint32_t *platformStatus, uint32_t *RFCtrlStatus)
{
  
}



void CKlysMrFwAdapt::armDaq()
{
//    waveformInit_->setVal( (uint64_t) 1 );
//    waveformInit_->setVal( (uint64_t) 0 );
    trigHwArm_->setVal( (uint64_t) 1 );
    trigHwArm_->setVal( (uint64_t) 0 );
}

void CKlysMrFwAdapt::initBuf(void)
{

    waveformInit_->setVal( (uint64_t) 1 );
    waveformInit_->setVal( (uint64_t) 0 );

}


void CKlysMrFwAdapt::loadConfigFromYamlFile( const char *filename, const char *yaml_dir)
{
    YAML::Node config( CYamlFieldFactoryBase::loadPreprocessedYamlFile( filename, yaml_dir ) );
    p_->loadConfigFromYaml( config );
}

void CKlysMrFwAdapt::dumpConfigToYamlFile( const char *filename, const char *yaml_dir)
{
    YAML::Node config;
    p_->dumpConfigToYaml( config );

    YAML::Emitter emit;
    emit << config;

    std::fstream strm( filename, std::fstream::out );
    strm << emit.c_str() << "\n";
}


KlysMrFw IKlysMrFw::create(Path p)
{
    //KlysMrFwAdapt rval = IEntryAdapt::check_interface<KlysMrFwAdapt, Entry>(p);
    //return rval;
    return IEntryAdapt::check_interface<KlysMrFwAdapt, DevImpl>( p );
}


//

void CKlysMrFwAdapt::getDebugPacketCnt(uint32_t *dbgCnt)
{
    try {
        debugPacketCnt_->getVal(dbgCnt);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::getDebugTrgCnt(uint32_t *dbgCnt0, uint32_t *dbgCnt1)
{
    try{
      debugTrgCnt0_->getVal(dbgCnt0);
      debugTrgCnt1_->getVal(dbgCnt1);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}


void CKlysMrFwAdapt::setAtten(uint32_t att, int bay, int chn)
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

void CKlysMrFwAdapt::setAtten(uint32_t att, int index)
{
    if(index > 9) return;  // bay1 chn 4 and chn5 are not available.
    try {
        atten_[index]->setVal(att);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }

}

void CKlysMrFwAdapt::getIntPhaseError(int32_t *phase)
{
    try {
        intPhaseError_->getVal((uint32_t*)phase);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setCKPhase(uint32_t ckPhase)
{
    try {
        ckPhaseSet_->setVal(ckPhase);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
} 


void CKlysMrFwAdapt::getRtmStatus(uint32_t *rtmStatus)
{
    try {
        rtmStatus_->getVal(rtmStatus);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}


void CKlysMrFwAdapt::getRtmFirmwareVersion(uint32_t *rtmFirmwareVersion)
{
    try {
        rtmFirmwareVersion_->getVal(rtmFirmwareVersion);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}


void CKlysMrFwAdapt::getRtmSystemId(char *systemIdString)
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


void CKlysMrFwAdapt::getRtmSubType(char *subTypeString)
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


void CKlysMrFwAdapt::getRtmFirmwareDate(char *firmwareDateString)
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

void CKlysMrFwAdapt::setRtmKlyWiperRegA(uint32_t reg)
{
    try {
        rtmKlyWiperRegA_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setRtmKlyWiperRegB(uint32_t reg)
{
    try {
        rtmKlyWiperRegB_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setRtmKlyNVRegA(uint32_t reg)
{

    try {
        rtmKlyNVRegA_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setRtmKlyNVRegB(uint32_t reg)
{
    try {
        rtmKlyNVRegB_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setRtmModWiperRegA(uint32_t reg)
{
    try {
        rtmModWiperRegA_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setRtmModWiperRegB(uint32_t reg)
{
    try {
        rtmModWiperRegB_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setRtmModNVRegA(uint32_t reg)
{
    try {
        rtmModNVRegA_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setRtmModNVRegB(uint32_t reg)
{
    try {
        rtmModNVRegB_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setRtmCfgRegister(uint32_t reg)
{
    try {
        rtmCfgRegister_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::getRtmFaultOutStatus(uint32_t *rtmFaultOutStatus)
{
    try {
        rtmFaultOutStatus_->getVal(rtmFaultOutStatus);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::getRtmAdcLockedStatus(uint32_t *rtmAdcLockedStatus)
{    
    try {
        rtmAdcLockedStatus_->getVal(rtmAdcLockedStatus);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::getRtmRFOffStatus(uint32_t *rtmRFOffStatus)
{
    try {
        rtmRFOffStatus_->getVal(rtmRFOffStatus);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::getRtmAdcIn(uint32_t v[])
{
    try {
        rtmAdcIn_->getVal(v,4);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::setRtmMode(uint32_t reg)
{
    try {
        rtmMode_->setVal(reg);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::getRtmFastAdcBufferBeamCurrentVoltage(uint32_t v[])
{
    try {
        rtmAdcBufferBeamIV_->getVal(v, (unsigned) 0x200);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::getRtmFastAdcBufferForwardReflect(uint32_t v[])
{
    try {
        rtmAdcBufferFwdRef_->getVal(v, (unsigned) 0x200);
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::cmdRtmRearm(void)
{
    try {
        rtmRearm_Cmd_->execute();
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::cmdRtmSwTrigger(void)
{
    try {
        rtmSwTrigger_Cmd_->execute();
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CKlysMrFwAdapt::cmdRtmClearFault(void)
{
    try {
        rtmClearFault_Cmd_->execute();
    } catch( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}