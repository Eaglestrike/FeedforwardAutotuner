#include "ShuffleboardSender/ShuffleboardSender.h"

ShuffleboardSender::ShuffleboardSender(std::string name):
name_(name)
{
}

void ShuffleboardSender::Initialize(bool edit){
    edit_ = edit;
    enabled_ = true;
    if(!initialized_){
        tab_ = &frc::Shuffleboard::GetTab(name_);
        initialized_ = true;
    }
}

void ShuffleboardSender::update(){
    if(!initialized_){
        return;
    }
    if(edit_){
        for(ShuffleboardItemInterface* item : items_){
            item->edit();
        }
    }
    for(ShuffleboardItemInterface* item : items_){
        item->send();
    }
};