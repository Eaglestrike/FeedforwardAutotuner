#include "ShuffleboardSender/ShuffleboardButton.h"

ShuffleboardButton::ShuffleboardButton(ItemData data, std::function<void()> callback):
    ShuffleboardItemInterface(data),
    callback_(callback)
{
    entry_ = ShuffleboardHelper::createItem(data_, false, frc::BuiltInWidgets::kToggleButton);
}

void ShuffleboardButton::update(bool update){
    if(update){
        if(entry_->GetBoolean(false)){
            callback_();
            entry_->SetBoolean(false);
        }
    }
}

void ShuffleboardButton::enable(){
    if(!entry_->Exists()){
        entry_ = ShuffleboardHelper::createItem(data_, false, frc::BuiltInWidgets::kToggleButton);
    }
};
void ShuffleboardButton::disable(){
    entry_->Unpublish();
};