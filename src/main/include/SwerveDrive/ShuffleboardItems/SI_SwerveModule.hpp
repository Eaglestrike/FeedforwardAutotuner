#pragma once

#include "Util/Point.h"
#include "ShuffleboardSender/ShuffleboardItem.h"
#include "SwerveDrive/SwervePose.h"

template<> class ShuffleboardItem<SwervePose::ModulePose>: public BaseShuffleboardItem{
    public:
        ShuffleboardItem(ItemData data, SwervePose::ModulePose* value):
            BaseShuffleboardItem(data)
        {
            value_ = value;

            if(data_.tab){
                frc::ShuffleboardLayout* pointLayout = ShuffleboardHelper::createList(data);
                entry_[0] = pointLayout->Add("Vel", value->speed).GetEntry();   
                entry_[1] = pointLayout->Add("Ang", value->ang).GetEntry();
            }
        }

        bool itemHasChanged() override{
            bool hasChanged = (prevVal_.speed != value_->speed) || (prevVal_.ang != value_->ang);
            prevVal_ = *value_;
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
            entry_[0] = pointLayout->Add("Vel", value_->speed).GetEntry();   
            entry_[1] = pointLayout->Add("Ang", value_->ang).GetEntry();
        }

        void disable() override{

        }
        
    private:
        void send() override{
            entry_[0]->SetDouble(value_->speed);
            entry_[1]->SetDouble(value_->ang);
        }

        void edit() override{
            value_->speed = entry_[0]->GetDouble(value_->speed);
            value_->speed = entry_[1]->GetDouble(value_->ang);
        }

        SwervePose::ModulePose prevVal_;
        SwervePose::ModulePose* value_;
        nt::GenericEntry* entry_[2]; //[vel, ang]
};