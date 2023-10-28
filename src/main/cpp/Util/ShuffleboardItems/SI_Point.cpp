#include "util/ShuffleboardItems/SI_Point.h"

ShuffleboardItem<Point>::ShuffleboardItem(ItemData data, Point* value):
    edit_(data.edit)
{
    value_ = value;

    frc::ShuffleboardLayout* poseLayout;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        poseLayout = &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                                    .WithSize(data.width, data.height)
                                                    .WithPosition(data.positionX, data.positionY)
                                                    .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
    }
    else{
        poseLayout = &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                                    .WithSize(data.width, data.height)
                                                    .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
    }
    entry_[0] = poseLayout->Add("X", value->getX()).GetEntry();   
    entry_[1] = poseLayout->Add("Y", value->getY()).GetEntry(); 
};

void ShuffleboardItem<Point>::send(){
    entry_[0]->SetDouble(value_->getX());
    entry_[1]->SetDouble(value_->getY());
}

void ShuffleboardItem<Point>::edit(){
    if(!edit_)return;
    value_->setX(entry_[0]->GetDouble(value_->getX()));
    value_->setY(entry_[1]->GetDouble(value_->getY()));
}