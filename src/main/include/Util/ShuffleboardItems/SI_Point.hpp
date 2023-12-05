#pragma once

#include "Util/Point.h"
#include "ShuffleboardSender/ShuffleboardItem.h"

template<> class ShuffleboardItem<Point>: public BaseShuffleboardItem{
    public:
        ShuffleboardItem(ItemData data, Point* value):
            BaseShuffleboardItem(data)
        {
            value_ = value;

            if(data_.tab){
                frc::ShuffleboardLayout* pointLayout = ShuffleboardHelper::createList(data);
                entry_[0] = pointLayout->Add("X", value->getX()).GetEntry();   
                entry_[1] = pointLayout->Add("Y", value->getY()).GetEntry();
            }
        }

        bool itemHasChanged() override{
            Point newVal = *value_;
            bool hasChanged = (prevVal_ != newVal);
            prevVal_ = newVal;
            return hasChanged;
        }

        void enable(frc::ShuffleboardTab* tab) override{
            for(auto &component :tab->GetComponents()){
                if(component->GetTitle() == data_.name){
                    return;
                }
            }
            data_.tab = tab;
            frc::ShuffleboardLayout* pointLayout = ShuffleboardHelper::createList(data_);
            entry_[0] = pointLayout->Add("X", value_->getX()).GetEntry();   
            entry_[1] = pointLayout->Add("Y", value_->getY()).GetEntry();
        }

        void disable() override{

        }
        
    private:
        void send() override{
            entry_[0]->SetDouble(value_->getX());
            entry_[1]->SetDouble(value_->getY());
        }

        void edit() override{
            value_->setX(entry_[0]->GetDouble(value_->getX()));
            value_->setY(entry_[1]->GetDouble(value_->getY()));
        }

        Point prevVal_;
        Point* value_;
        nt::GenericEntry* entry_[2]; //[x, y]
};