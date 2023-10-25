#pragma once

#include "ShuffleboardSender/ShuffleboardItem.h"
#include "SwerveDrive/SwervePose.h"
#include "Geometry/GeometryHelper.h"

using namespace GeometryHelper;

//Constructor
ShuffleboardItem<double>::ShuffleboardItem(ItemData data, double* value):
    edit_(data.edit)
{
    value_ = value;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        entry_ = data.tab->Add(data.name, *value)
                                .WithSize(data.width, data.height)
                                .WithPosition(data.positionX, data.positionY)
                                .GetEntry();
    }
    else{
        entry_ = data.tab->Add(data.name, *value)
                                .WithSize(data.width, data.height)
                                .GetEntry();
    }
}

ShuffleboardItem<bool>::ShuffleboardItem(ItemData data, bool* value):
    edit_(data.edit)
{
    value_ = value;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        entry_ = data.tab->Add(data.name, *value)
                                .WithSize(data.width, data.height)
                                .WithPosition(data.positionX, data.positionY)
                                .GetEntry();
    }
    else{
        entry_ = data.tab->Add(data.name, *value)
                                .WithSize(data.width, data.height)
                                .GetEntry();
    }
}

ShuffleboardItem<int>::ShuffleboardItem(ItemData data, int* value):
    edit_(data.edit)
{
    value_ = value;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        entry_ = data.tab->Add(data.name, *value)
                                .WithSize(data.width, data.height)
                                .WithPosition(data.positionX, data.positionY)
                                .GetEntry();
    }
    else{
        entry_ = data.tab->Add(data.name, *value)
                                .WithSize(data.width, data.height)
                                .GetEntry();
    }
}

ShuffleboardItem<units::volt_t>::ShuffleboardItem(ItemData data, units::volt_t* value):
    edit_(data.edit)
{
    value_ = value;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        entry_ = data.tab->Add(data.name, value->value())
                                .WithSize(data.width, data.height)
                                .WithPosition(data.positionX, data.positionY)
                                .GetEntry();
    }
    else{
        entry_ = data.tab->Add(data.name, value->value())
                                .WithSize(data.width, data.height)
                                .GetEntry();
    }
}

ShuffleboardItem<frc::PIDController>::ShuffleboardItem(ItemData data, frc::PIDController* value):
    edit_(data.edit)
{
    value_ = value;

    frc::ShuffleboardLayout* pidLayout;
    if((data.positionX >= 0) && (data.positionY >= 0)){
        pidLayout = &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                                    .WithSize(data.width, data.height)
                                                    .WithPosition(data.positionX, data.positionY)
                                                    .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
    }
    else{
        pidLayout = &data.tab->GetLayout(data.name, frc::BuiltInLayouts::kList)
                                                    .WithSize(data.width, data.height)
                                                    .WithProperties({std::make_pair("Label Position", nt::Value::MakeString("Right"))});
    }
    entry_[0] = pidLayout->Add("P", value->GetP()).GetEntry();   
    entry_[1] = pidLayout->Add("I", value->GetI()).GetEntry();
    entry_[2] = pidLayout->Add("D", value->GetD()).GetEntry(); 
};

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
};

ShuffleboardItem<SwervePose::Pose>::ShuffleboardItem(ItemData data, SwervePose::Pose* value):
    edit_(data.edit)
{
    value_ = value;

    field_.SetRobotPose(units::meter_t{value->pos.getX()}, units::meter_t{value->pos.getY()}, {units::radian_t{value->ang}});
    frc::SmartDashboard::PutData("Field", &field_);
};

//Send
void ShuffleboardItem<double>::send(){
    entry_->SetDouble(*value_);
}
void ShuffleboardItem<bool>::send(){
    entry_->SetBoolean(*value_);
}
void ShuffleboardItem<int>::send(){
    entry_->SetInteger(*value_);
}
void ShuffleboardItem<units::volt_t>::send(){
    entry_->SetDouble(value_->value());
}
void ShuffleboardItem<frc::PIDController>::send(){
    entry_[0]->SetDouble(value_->GetP());
    entry_[1]->SetDouble(value_->GetI());
    entry_[2]->SetDouble(value_->GetD());
}
void ShuffleboardItem<Point>::send(){
    entry_[0]->SetDouble(value_->getX());
    entry_[1]->SetDouble(value_->getY());
}
void ShuffleboardItem<SwervePose::ModulePose>::send(){
    entry_[0]->SetDouble(toDeg(value_->ang));
    entry_[1]->SetDouble(value_->speed);
}
void ShuffleboardItem<SwervePose::Pose>::send(){
    field_.SetRobotPose(units::meter_t{value_->pos.getX()}, units::meter_t{value_->pos.getY()}, {units::radian_t{value_->ang}});
}

//Edit
void ShuffleboardItem<double>::edit(){
    if(!edit_)return;
    *value_ = entry_->GetDouble(*value_);
}
void ShuffleboardItem<bool>::edit(){
    if(!edit_)return;
    *value_ = entry_->GetBoolean(*value_);
}
void ShuffleboardItem<int>::edit(){
    if(!edit_)return;
    *value_ = entry_->GetInteger(*value_);
}

void ShuffleboardItem<units::volt_t>::edit(){
    if(!edit_)return;
    *value_ = units::volt_t{entry_->GetDouble(value_->value())};
}

void ShuffleboardItem<frc::PIDController>::edit(){
    if(!edit_)return;
    value_->SetP(entry_[0]->GetDouble(value_->GetP()));
    value_->SetI(entry_[1]->GetDouble(value_->GetI()));
    value_->SetD(entry_[2]->GetDouble(value_->GetD()));
}

void ShuffleboardItem<Point>::edit(){
    if(!edit_)return;
    value_->setX(entry_[0]->GetDouble(value_->getX()));
    value_->setY(entry_[1]->GetDouble(value_->getY()));
}

void ShuffleboardItem<SwervePose::ModulePose>::edit(){
    if(!edit_)return;
    value_->ang = entry_[0]->GetDouble(toRad(value_->ang));
    value_->speed = entry_[1]->GetDouble(value_->speed);
}

void ShuffleboardItem<SwervePose::Pose>::edit(){
    if(!edit_)return;
}