#pragma once

#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<frc::PIDController>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, frc::PIDController* value);
        void enable() override;
        void disable() override;
        
    private:
        void coreSend() override;
        void coreEdit() override;
        frc::PIDController* value_;
        nt::GenericEntry* entry_[3]; //[P, I, D]
};