#pragma once

#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<SwervePose::ModulePose>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, SwervePose::ModulePose* value);
        void enable(frc::ShuffleboardTab* tab) override;
        void disable() override;
        
    private:
        void send() override;
        void edit() override;
        SwervePose::ModulePose value_;
        nt::GenericEntry* entry_[3]; //[P, I, D]
};