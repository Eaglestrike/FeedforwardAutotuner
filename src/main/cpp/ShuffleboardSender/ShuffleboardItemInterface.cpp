#include "ShuffleboardSender/ShuffleboardItemInterface.h"

ShuffleboardItemInterface::ShuffleboardItemInterface(ItemData data):
    data_(data)
{
}

void ShuffleboardItemInterface::update(bool edit){
    if(itemHasChanged()){
        coreSend();
    }
    if(data_.edit && edit){
        coreEdit();
    }
}

bool ShuffleboardItemInterface::itemHasChanged(){
    return true;
}