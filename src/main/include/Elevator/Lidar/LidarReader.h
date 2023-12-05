#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <frc/SerialPort.h>
#include <frc/Timer.h>

#include "Util/Mechanism.h"

namespace LidarReaderConstants{
    const char REQ[] = {0x59}; //Request write char
    const char RES = 0x59; //Respond read char

    const int BAUD_RATE = 115200; //Speed of communication
    const frc::SerialPort::Port LIDAR_PORT = frc::SerialPort::kMXP;

    const double RESPONSE_TIME = 1.0; //Maximum time to respond for valid data

    const double VALID_DATA_TIME = 3.0; //Time in which data is valid

    const unsigned char NO_READ = 255; //Value if lidar doesn't see anything

    const double DEFAULT_POSITION = 30.0; //cm
};

class LidarReader : public Mechanism{
    public:
        // for thread only
        struct LidarDataAtomic{
           std::atomic<double> conePos; //cm
           std::atomic<double> cubePos; //cm
           std::atomic<bool> hasCone;
           std::atomic<bool> hasCube;
           std::atomic<bool> isValid;
           std::atomic<double> readTime; //Time in which data was recorded
        };

        // for outside access
        struct LidarData {
            double conePos;
            double cubePos;
            bool hasCone;
            bool hasCube;
            bool isValid;
            double readTime;
        };

        LidarReader(bool enable = true, bool shuffleboard = false);
        void RequestData();

        void setAutoRequest(bool autoRequest);

        LidarData getData();
        double getConePos();
        double getCubePos();
        bool hasCone();
        bool hasCube();
        bool validData();
        double getRecordedTime();

    private:
        void CoreInit() override;
        void CoreShuffleboardInit() override;
        void CoreShuffleboardPeriodic() override;

        void PeriodicLoop();

        void readPortData(unsigned char (&readData)[8], int& readIndex);
        void storeData(const unsigned char data[4]);
        bool isValidData(const unsigned char data[4]);
        void findOffset(unsigned char (&readData)[8], int& readIndex);

        std::thread thread_;

        std::atomic<bool> autoRequest_;

        frc::SerialPort port_;
        std::atomic<double> reqTime_; //Time since last request
        std::atomic<bool> isRequesting_; //If currently there is a call

        LidarDataAtomic data_;
};