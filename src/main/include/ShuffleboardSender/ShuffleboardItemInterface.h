#pragma once

#include <string>

#include <frc/shuffleboard/Shuffleboard.h>

class ShuffleboardItemInterface{
    public:
        /**
         * Pose Struct:
         * {width, height, x, y}
         * Coordinates start at 0, 0
         * -1 coordinates is default placement
        */
        struct ShuffleboardPose{
            int width = 1;
            int height = 1;
            int positionX = -1;
            int positionY = -1;
        };

        struct ItemData{
            std::string name;
            frc::ShuffleboardTab* tab;
            bool edit;
            ShuffleboardPose pose;
        };
        
        ShuffleboardItemInterface(ItemData data);

        void update(bool edit);
        virtual bool itemHasChanged(); //If value in code has changed

        virtual void enable() = 0;
        virtual void disable() = 0;

    protected:
        virtual void coreSend() = 0;
        virtual void coreEdit() = 0;
        
        ItemData data_;
};