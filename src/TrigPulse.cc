#include "TrigPulse.h"

#include <cpsw_sval.h>
#include <cpsw_entry_adapt.h>
#include <cpsw_hub.h>
#include <fstream>
#include <sstream>
#include <math.h>

class CTrigPulseAdapt;
typedef shared_ptr<CTrigPulseAdapt> TrigPulseAdapt;

class CTrigPulseAdapt : public ITrigPulse, public IEntryAdapt {
protected:
    ScalVal    trigDelay_;
    ScalVal    trigWidth_;
    ScalVal    trigOpCode_;
    ScalVal    trigPolarity_;

public:
        CTrigPulseAdapt(Key &k, Path p, shared_ptr<const CEntryImpl> ie);

public:
    virtual void setTrigDelay     (double value_ns, double freq_MHz = 119.0);
    virtual void setTrigWidth     (double value_ns, double freq_MHz = 119.0);
    virtual void setTrigOpCode    (std::vector<int> opCodes);
    virtual void setTrigPolarity  (TrigPolarity polarity);
};

CTrigPulseAdapt::CTrigPulseAdapt(Key &k, Path p, shared_ptr<const CEntryImpl> ie) : 
  IEntryAdapt(k, p, ie),
  trigDelay_(    IScalVal::create( p_->findByName("PulseDelay") ) ),
  trigWidth_(    IScalVal::create( p_->findByName("PulseWidth") ) ),
  trigOpCode_(   IScalVal::create( p_->findByName("OpCodeMask") ) ),
  trigPolarity_( IScalVal::create( p_->findByName("PulsePolarity") ) )
{
}

TrigPulse ITrigPulse::create(Path p)
{
    //TrigPulseAdapt rval = IEntryAdapt::check_interface<TrigPulseAdapt, Entry>(p);
    //return rval;
    return IEntryAdapt::check_interface<TrigPulseAdapt, DevImpl>( p );
}


/**
 * Set the external trigger (ACC/STDBY/SPARE) delay
 * Input:
 *   value_ns           : Delay value in ns
 *   freq_MHz           : EVR frequency in MHz
 */
void CTrigPulseAdapt::setTrigDelay(double value_ns, double freq_MHz)
{
    uint32_t data = (uint32_t)(value_ns * freq_MHz / 1000.0);
    try {
        trigDelay_->setVal( data );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CTrigPulseAdapt::setTrigWidth(double value_ns, double freq_MHz)
{
    uint32_t data = (uint32_t)(value_ns * freq_MHz / 1000.0);
    try {
        trigWidth_->setVal( data );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CTrigPulseAdapt::setTrigOpCode(std::vector<int> opCodes)
{
    /* OpCodes is an array (8) of 32-bit word scal vals */
    const int maskSize = 8;
    std::vector<uint32_t> mask(maskSize, 0);

    /* Build the mask vector */
    for ( std::vector<int>::iterator it = opCodes.begin(); it != opCodes.end(); ++it ) {
        int quotient  = (*it) / 32;
        int remainder = (*it) % 32;

        uint32_t value = pow(2, remainder); 
        mask[quotient] |= value;
    }

    try {
        trigOpCode_->setVal( (uint32_t *) &(mask[0]), maskSize );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}

void CTrigPulseAdapt::setTrigPolarity(TrigPolarity polarity)
{
    try {
        trigPolarity_->setVal( polarity );
    } catch ( CPSWError &e ) {
        fprintf(stderr,"CPSW Error: %s\n", e.getInfo().c_str());
        throw e;
    }
}
