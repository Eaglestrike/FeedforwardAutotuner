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
         * Pair a value on shuffleboard to the code
        */
        template <typename T> void add(ShuffleboardItem<T>* item){
            items_.push_back(item);
        }

        template <typename T> void add(std::string name, T* o, bool edit = false){
            items_.push_back(new ShuffleboardItem({name, tab_, edit}, o));
        }
        
        /**
         * Adds an item with the position and size of order {width, height, x, y}
         * Coordinates start at 0, 0
        */
        template <typename T> void add(std::string name, T* o, ShuffleboardItemInterface::ShuffleboardPose pose, bool edit = false){
            items_.push_back(new ShuffleboardItem({name, tab_, edit, pose}, o));
        }
        
        /**
         * Updates variables by reading and configuring, and then sending the data
         * 
         * edit boolean enable editing
        */
        void update(bool edit);

        void enable();
        void disable();

    private:
        std::string name_;
        
        bool enabled_ = false;
        
        frc::ShuffleboardTab* tab_;
        std::vector<ShuffleboardItemInterface*> items_;
};
