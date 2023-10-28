#include "SwerveDrive\ShuffleboardItems\SI_SwerveModule.h"

#include "Util\GeometryHelper.h"
using namespace GeometryHelper;

ShuffleboardItem<SwervePose::ModulePose>::ShuffleboardItem(ItemData data, SwervePose::ModulePose* value):
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
    entry_[0] = poseLayout->Add("Ang", toDeg(value->ang)).GetEntry();   
    entry_[1] = poseLayout->Add("Vel", value->speed).GetEntry(); 
}

void ShuffleboardItem<SwervePose::ModulePose>::send(){
    entry_[0]->SetDouble(toDeg(value_->ang));
    entry_[1]->SetDouble(value_->speed);
}

void ShuffleboardItem<SwervePose::ModulePose>::edit(){
    if(!edit_)return;
    value_->ang = entry_[0]->GetDouble(toRad(value_->ang));
    value_->speed = entry_[1]->GetDouble(value_->speed);
}