#include "util/ShuffleboardItems/SI_Point.h"

ShuffleboardItem<Point>::ShuffleboardItem(ItemData data, Point* value):
    ShuffleboardItemInterface(data)
{
    value_ = value;

    frc::ShuffleboardLayout* poseLayout = ShuffleboardHelper::createList(data);
    entry_[0] = poseLayout->Add("X", value->getX()).GetEntry();   
    entry_[1] = poseLayout->Add("Y", value->getY()).GetEntry(); 
};

void ShuffleboardItem<Point>::coreSend(){
    entry_[0]->SetDouble(value_->getX());
    entry_[1]->SetDouble(value_->getY());
}

void ShuffleboardItem<Point>::coreEdit(){
    value_->setX(entry_[0]->GetDouble(value_->getX()));
    value_->setY(entry_[1]->GetDouble(value_->getY()));
}

void ShuffleboardItem<Point>::enable(){
    if((!entry_[0]->Exists()) || (!entry_[1]->Exists())){
        frc::ShuffleboardLayout* poseLayout = ShuffleboardHelper::createList(data_);
        entry_[0] = poseLayout->Add("X", value_->getX()).GetEntry();   
        entry_[1] = poseLayout->Add("Y", value_->getY()).GetEntry(); 
    }
};

void ShuffleboardItem<Point>::disable(){
    entry_[0]->Unpublish();
    entry_[1]->Unpublish();
};