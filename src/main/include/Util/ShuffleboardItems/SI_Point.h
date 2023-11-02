#pragma once

#include "Util/Point.h"
#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<Point>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, Point* value);
        void enable() override;
        void disable() override;
        
    private:
        void coreSend() override;
        void coreEdit() override;

        Point* value_;
        nt::GenericEntry* entry_[2]; //[x, y]
};