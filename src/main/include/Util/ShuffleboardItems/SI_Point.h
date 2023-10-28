#pragma once

#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<Point>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, Point* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        Point* value_;
        nt::GenericEntry* entry_[2]; //[x, y]
};