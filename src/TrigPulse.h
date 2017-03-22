#ifndef _TRIG_PULSE_H
#define _TRIG_PULSE_H

#include <cpsw_api_user.h>
#include <cpsw_api_builder.h>

class ITrigPulse;
typedef shared_ptr<ITrigPulse> TrigPulse;

class ITrigPulse : public virtual IEntry {
public:
    static TrigPulse create(Path p);
public:
    typedef enum TrigPolarity
    {
        NORMAL = 0,
        INVERTED = 1,
    } TrigPolarity;
    virtual void setTrigDelay        (double value_ns, double freq_MHz = 119.0) = 0;
    virtual void setTrigWidth        (double value_ns, double freq_MHz = 119.0) = 0;
    virtual void setTrigOpCode       (std::vector<int> opCodes)                 = 0;
    virtual void setTrigPolarity     (TrigPolarity polarity)                        = 0;
};

#endif
