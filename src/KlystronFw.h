#ifndef _KLYSTRON_FW_H
#define _KLYSTRON_FW_H

#include <cpsw_api_user.h>
#include <cpsw_api_builder.h>

class IKlystronFw;
typedef shared_ptr<IKlystronFw> KlystronFw;

class IKlystronFw : public virtual IEntry {
public:
    static KlystronFw create(Path p);
public:
    typedef enum Trigger { LLRF_ACCEL =0, LLRF_STDBY=1, MOD_ACCEL=2, MOD_STDBY=3, SSB_ACCEL=4, SSB_STDBY=5, RTM_ACCEL=6, RTM_STDBY=7} Trigger;
    typedef enum TrigMode { DISABLE=0, ACCEL=1, STANDBY=2, ACCEL_OR_STANDBY=3 } TrigMode;
    typedef enum TrigPolarity { NORMAL =0, INVERTED=1} TrigPolarity;
    typedef enum Timeslot{ ONE = 1, TWO = 2, THREE = 3, FOUR = 4, FIVE = 5, SIX = 6 } Timeslot;
    
    typedef enum StremID { STREAM_0 = 0, STREAM_1 = 1, STREAM_2 =2, STREAM_3=3, STREAM_4=4, STREAM_5=5 } StreamId;
    typedef enum StreamSel { BAY0CH0 = 0, BAY0CH1=1, BAY0CH2=2, BAY0CH3=3, BAY0CH4=4, BAY0CH5=5, BAY1CH0=6, BAY1CH1=7, BAY1CH2=8, BAY2CH3=9, BAY2CH4=10, BAY2CH5=11, DAC_OUTPUT=12, PHASE_ERR=13 } StreamSel;
    
    virtual void getJesdStatusValidCnt(uint32_t *cnt, int jesd_idx, int cnt_idx) = 0;
    
    virtual void reset()                      = 0;

    virtual void setLO_KP(uint32_t KP)        = 0;
    virtual void setLO_KI(uint32_t KI)        = 0;
    virtual void setCLK_KP(uint32_t KP)       = 0;
    virtual void setCLK_KI(uint32_t KI)       = 0;
    virtual void setADPLL_KP(uint32_t KP)     = 0;
    virtual void setADPLL_KI(uint32_t KI)     = 0;
    
    virtual void getLoPllPhase(uint32_t *phase)            = 0;
    virtual void getLoPllAmplitude(uint32_t *amplitude)    = 0;
    virtual void getLoPllLocked(uint32_t *lockStatus)      = 0;
    virtual void getClockPllPhase(uint32_t *phase)         = 0;
    virtual void getClockPllAmplitude(uint32_t *amplitude) = 0;
    virtual void getClockPllLocked(uint32_t *lockStatus)   = 0;
    virtual void getAdPllPhase(uint32_t *phase)            = 0;
    virtual void getAdPllAmplitude(uint32_t *amplitude)    = 0;
    virtual void getAdPllLocked(uint32_t *lockStatus)      = 0;
    
    virtual void getLoPllLossSignal(uint32_t *loss)        = 0;
    virtual void getClockPllLossSignal(uint32_t *loss)     = 0;
    virtual void getAdPllLossSignal(uint32_t *loss)        = 0;
    virtual void getLoPllUnlockCounter(uint32_t *count)    = 0;
    virtual void getClockPllUnlockCounter(uint32_t *count) = 0;
    virtual void getAdPllUnlockCounter(uint32_t *count)    = 0;

    virtual void OutputEnable(bool enable)    = 0; /* Set this to 1 at startup */
    virtual void CWOutputEnable(bool enable)  = 0; 
    virtual void setDebugStreamSelect(StreamId sid, StreamSel ssel) = 0;


    virtual void setTrigMode         (TrigMode mode) = 0;    
    virtual void setTrigPolarity     (Trigger trig, TrigPolarity polarity) = 0;
    virtual void setTrigDelay        (Trigger trig, double value_ns, double freq_MHz = 119.0) = 0;
    virtual void setTrigWidth        (Trigger trig, double value_ns, double freq_MHz = 119.0) = 0;
    virtual void setTrigOpCode       (Trigger trig, std::vector<int> opCodes)                 = 0;

    virtual void setDaqSize( uint32_t buf_size ) = 0;  /* setup DAQ size, byte size, will be rounded up to 4k blocks */

    virtual void selectRefFbkChannel(uint32_t refCh, uint32_t fbkCh) = 0;
    
    virtual void setRefPhaSP(double phaSP_deg) = 0;
    
    virtual void  setSWFeedbackCorrection(Timeslot timeslot, double amplitude_norm, double phase_deg ) = 0;  /* scale and rotate the baseband signal */
    virtual void  getSWFeedbackCorrection(Timeslot timeslot, double *amplitude_norm, double *phase_deg ) = 0;  /* get current settings for phase/amp correct */
    virtual void  enableTimeslotCorrection( bool enable = true ) = 0;

/* FW feeback control */
    virtual void  setFeedforward(double phase, double amplitude) = 0;                  /* set the feedforward value for phase */
    virtual void  setGain(double phase, double amplitude) = 0;                         /* set the gain value for phase */
    virtual void  setFeedbackCorrLimits(double phase, double amplitdue) = 0;

    virtual void  setIntgStart(double value_ns, double freq_MHz = 357.0) = 0;            /* start time for integration, relative to the acc trigger */
    virtual void  setIntgEnd(  double value_ns, double freq_MHz = 357.0) = 0;             

/* For FW + SW feedback */
    virtual void  setApplStart(double value_ns, double freq_MHz = 357.0) = 0;            /* start time for applying the correction, relative to the acc trigger */
    virtual void  setApplEnd(  double value_ns, double freq_MHz = 357.0) = 0;             


    virtual void  setDACOffset(  int16_t offset) = 0;                       /* set the DAC offset */
    virtual void  setAmpLimit(   int16_t high, int16_t low) = 0;            /* set the output limit */

    /* pno will set RF pulse lenghth, data @ 357 MPSP*/
    /* table depth is 4096 */
    virtual void  setIQSPTable       (uint32_t pno, double *ISPTable, double *QSPTable) = 0;                        /* set the set point table */

    virtual void  setNonIQCoefOffset(uint32_t offset) = 0;

/* Fimrware readings */
    virtual void  getAppFwInfo(uint32_t *firmwareName, uint32_t *majorVer, uint32_t *minorVer, uint32_t *buildNum) = 0;

    virtual void  getRaceConditionFlags(uint32_t *accFlags, uint32_t *stdbyFlags, uint32_t *spareFlags) = 0;

    virtual void  getADCValid(bool *valid) = 0;

    virtual void  getPulseCounter(uint32_t *pulseCnt) = 0;
    virtual void  getMeaTrigPeriod(double *value_ms, double freq_MHz = 119.0) = 0;       /* get the measurement trigger period */

    virtual void  getNonIQCoefCur(uint32_t *cur) = 0;

    virtual void  getFwStatus(uint32_t *platformStatus, uint32_t *RFCtrlStatus) = 0;

//    virtual uint32_t  getAllStreamData(uint32_t pno,                            /* data from DRAM */
//                                                            uint8_t *data0, uint8_t *data1,            
//                                                            uint8_t *data2, uint8_t *data3,
//                                                            uint8_t *data4, uint8_t *data5,
//                                                            uint8_t *data6, uint8_t *data7) = 0;

    virtual void msgNoDelay(uint32_t no_delay) = 0;
    virtual void armDaq()      = 0;  /* rearm the DAQ (and async messages) */
    virtual void initBuf(void) = 0;

    virtual void loadConfigFromYamlFile( const char *filename, const char *yaml_dir = 0 ) = 0;
    virtual void dumpConfigToYamlFile(   const char *filename, const char *yaml_dir = 0 ) = 0;
//
    virtual void getDebugPacketCnt(uint32_t *dbgCnt) = 0;
    virtual void getDebugTrgCnt(uint32_t *dbgCnt0, uint32_t *dbgCnt1) = 0;
    virtual void setAtten(uint32_t att, int bay, int chn) = 0;
    virtual void setAtten(uint32_t att, int index) = 0;
    virtual void getTemp(int bay, int sensor, double *temp) = 0;
    virtual void getTemp(int index, double *temp) = 0;
    virtual void getIntPhaseError(int32_t *phase) = 0;
    virtual void setCKPhase(uint32_t ckPhase) = 0;


    virtual void getRtmStatus(uint32_t *rtmStatus) = 0;
    virtual void getRtmFirmwareVersion(uint32_t *rtmFirmwareVersion) = 0;
    virtual void getRtmSystemId(char * rtmSystemIdString) = 0;
    virtual void getRtmSubType(char *rtmSubTypeString) = 0;
    virtual void getRtmFirmwareDate(char *rtmFirmwareDateString) = 0;
    virtual void setRtmKlyWiperRegA(uint32_t v) = 0;
    virtual void setRtmKlyWiperRegB(uint32_t v) = 0;
    virtual void setRtmKlyNVRegA(uint32_t v) = 0;
    virtual void setRtmKlyNVRegB(uint32_t v) = 0;
    virtual void setRtmModWiperRegA(uint32_t v) = 0;
    virtual void setRtmModWiperRegB(uint32_t v) = 0;
    virtual void setRtmModNVRegA(uint32_t v) = 0;
    virtual void setRtmModNVRegB(uint32_t v) = 0;
    virtual void setRtmCfgRegister(uint32_t v) = 0;
    virtual void getRtmFaultOutStatus(uint32_t *rtmFaultOutStatus) = 0;
    virtual void getRtmAdcLockedStatus(uint32_t *rtmAdcLockedStatus) = 0;
    virtual void getRtmRFOffStatus(uint32_t *rtmRFOffStatus) = 0;
    virtual void getRtmAdcIn(uint32_t v[]) = 0;
    virtual void setRtmMode(uint32_t v) = 0;
    virtual void setRtmDesiredSled(bool tune) = 0;
    virtual void getRtmFastAdcBufferBeamCurrentVoltage(uint32_t v[]) = 0;
    virtual void getRtmFastAdcBufferForwardReflect(uint32_t v[]) =0;
    virtual void cmdRtmRearm(void) = 0;
    virtual void cmdRtmSwTrigger(void) = 0;
    virtual void cmdRtmClearFault(void) = 0;
    
};

#endif
