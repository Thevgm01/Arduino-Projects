#ifndef Polls_h
#define Polls_h

#include "Arduino.h"

class Polls {
    public:
        Polls(unsigned long updateDelay);
        
        virtual bool update();
        static void setMillis(unsigned long millis);
        unsigned long getLengthMillis();
    
    protected:        
        unsigned long startMillis{0};
        unsigned long lengthMillis{0};
    
        unsigned long getPollElapsedMillis();
        unsigned long getPollRemainingMillis();
        bool checkPoll();
        void resetPoll();
        bool isActive();
        void setPollLength(const unsigned long millis);
};

#endif