#include "ShuffleboardSender\ShuffleboardSender.h"

ShuffleboardSender::ShuffleboardSender(std::string name):
    name_(name),
    enabled_(false)
{
    enabled_ = true;
    tab_ = &frc::Shuffleboard::GetTab(name_);
}

void ShuffleboardSender::update(bool edit){
    for(ShuffleboardItemInterface* item : items_){
        item->update(edit);
    }
};

void ShuffleboardSender::enable(){
    enabled_ = true;
    for(ShuffleboardItemInterface* item : items_){
        item->enable();
    }
}

void ShuffleboardSender::disable(){
    enabled_ = false;
    for(ShuffleboardItemInterface* item : items_){
        item->disable();
    }
}