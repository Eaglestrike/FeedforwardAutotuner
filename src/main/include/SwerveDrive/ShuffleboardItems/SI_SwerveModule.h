#pragma once

#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<SwervePose::ModulePose>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, SwervePose::ModulePose* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        SwervePose::ModulePose* value_;
        nt::GenericEntry* entry_[2]; //[Ang, Speed]
};