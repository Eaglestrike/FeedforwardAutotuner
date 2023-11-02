#pragma once

#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<double> : public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, double* value);
        bool itemHasChanged() override;
        void enable() override;
        void disable() override;
    private:
        void coreSend() override;
        void coreEdit() override;
        double prevVal_;
        double* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<bool>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, bool* value);
        bool itemHasChanged() override;
        void enable() override;
        void disable() override;
    private:
        void coreSend() override;
        void coreEdit() override;
        bool prevVal_;
        bool* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<int>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, int* value);
        bool itemHasChanged() override;
        void enable() override;
        void disable() override;
    private:
        void coreSend() override;
        void coreEdit() override;
        int prevVal_;
        int* value_;
        nt::GenericEntry* entry_;
};