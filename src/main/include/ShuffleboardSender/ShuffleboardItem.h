#pragma once

#include <vector>

#include <iostream>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ComplexWidget.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/PIDCommand.h>
#include <units/voltage.h>
#include "SwerveDrive/SwervePose.h"
#include "Geometry/Point.h"

class ShuffleboardItemInterface{
    public:
        struct ItemData{
            std::string name;
            frc::ShuffleboardTab* tab;
            bool edit = false;
            int width = 1;
            int height = 1;
            int positionX = -1;
            int positionY = -1;
        };
        virtual void send() = 0;
        virtual void edit() = 0;
};

/**
 * Class to send objects to shuffleboard
 * 
 * Default only works for nt::Value
*/
template <typename T>
class ShuffleboardItem : public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, T* value);
        
        void send() override {
            entry_->Set(*value_);
        }

        void edit() override {
            if(!edit_)return;
            *value_ = entry_->Get();
        }

    private:
        bool edit_;

        T* value_;

        nt::GenericEntry* entry_;
};

template <typename T> ShuffleboardItem<T>::ShuffleboardItem(ItemData data, T* value):
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
        entry_ = data.tab->Add(data.name, *value).
                                    WithSize(data.width, data.height)
                                    .GetEntry();
    }
};

//Specialization
//Extra items go here:
/**
 * double
 * bool
 * int
 * units::volt_t
 * frc::PIDController
 * SwervePose::ModulePose
*/

template<> class ShuffleboardItem<double> : public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, double* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        double* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<bool>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, bool* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        bool* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<int>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, int* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        int* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<units::volt_t>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, units::volt_t* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        units::volt_t* value_;
        nt::GenericEntry* entry_;
};

template<> class ShuffleboardItem<frc::PIDController>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, frc::PIDController* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        frc::PIDController* value_;
        nt::GenericEntry* entry_[3]; //[P, I, D]
};

template<> class ShuffleboardItem<Point>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, Point* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        Point* value_;
        nt::GenericEntry* entry_[2]; //[x, y]
};

template<> class ShuffleboardItem<SwervePose::ModulePose>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, SwervePose::ModulePose* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        SwervePose::ModulePose* value_;
        nt::GenericEntry* entry_[2]; //[Ang, Speed]
};

template<> class ShuffleboardItem<SwervePose::Pose>: public ShuffleboardItemInterface{
    public:
        ShuffleboardItem(ItemData data, SwervePose::Pose* value);
        void send() override;
        void edit() override;
    private:
        bool edit_;
        SwervePose::Pose* value_;
        frc::Field2d field_;
        //frc::ComplexWidget* fieldWidget_;
        nt::GenericEntry* entry_[9]; //[x, y, vx, vy, ax, ay, ang, angVel, angAcc]
};