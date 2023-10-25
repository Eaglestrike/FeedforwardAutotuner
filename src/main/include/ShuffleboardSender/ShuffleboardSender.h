#pragma once

#include <vector>
#include <string>

#include <frc/shuffleboard/Shuffleboard.h>

#include <units/voltage.h>

#include "ShuffleboardItem.h"

/**
 * Class to send many variables to Shuffleboard and edit them
*/
class ShuffleboardSender{
    public:
        /**
         * Creates a tab with name
        */
        ShuffleboardSender(std::string name);

        /**
         * Initializes the tab
        */
        void Initialize(bool edit = false);
        bool isInitialized(){return initialized_;}

        /**
         * Pair a value on shuffleboard to the code
        */
        template <typename T> void add(ShuffleboardItem<T>* item){
            items_.push_back(item);
        }

        template <typename T> void add(std::string name, T* o, bool edit = false){
            items_.push_back(new ShuffleboardItem({name, tab_, edit}, o));
        }

        struct ShuffleboardPose{
            int width = 1;
            int height = 1;
            int positionX = -1;
            int positionY = -1;
        };
        /**
         * Pose Struct:
         * {width, height, x, y}
        */
        template <typename T> void add(std::string name, T* o, ShuffleboardPose pose, bool edit = false){
            items_.push_back(new ShuffleboardItem({name, tab_, edit, pose.width, pose.height, pose.positionX, pose.positionY}, o));
        }
        
        /**
         * Updates variables by reading and configuring, and then sending the data
        */
        void update();

        /***
         * If the sender's edit is disabled, all items will not be able to be edited
        */
        void disable() {enabled_ = false;}
        void enable(bool edit = false) {enabled_ = true; edit_ = edit;}

    private:
        std::string name_;
        bool initialized_ = false;
        bool edit_ = false;
        bool enabled_ = false;
        frc::ShuffleboardTab* tab_;
        std::vector<ShuffleboardItemInterface*> items_;
};
