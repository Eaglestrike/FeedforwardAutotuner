#include "Elevator/Lidar/LidarReader.h"

#include <iostream>
#include <algorithm>

#include "frc/smartdashboard/SmartDashboard.h"

/// @brief Constructor
LidarReader::LidarReader(bool enabled, bool shuffleboard):
    Mechanism("Lidar", enabled, shuffleboard),
    port_(LidarReaderConstants::BAUD_RATE, LidarReaderConstants::LIDAR_PORT),
    isRequesting_(false)
{
    frc::SmartDashboard::PutBoolean("Lidar Stale", true);
    double time = frc::Timer::GetFPGATimestamp().value();
    reqTime_.store(time);
    data_.conePos.store(LidarReaderConstants::DEFAULT_POSITION);
    data_.cubePos.store(LidarReaderConstants::DEFAULT_POSITION);
    data_.hasCone.store(false);
    data_.hasCube.store(false);
    data_.isValid.store(false);
    data_.readTime.store(time);

    port_.SetTimeout(0_s);
    port_.EnableTermination('\n');
}

/// @brief Requests Lidar for data
void LidarReader::RequestData(){
    if(isRequesting_.load()){
        return;
    }
    port_.Write(LidarReaderConstants::REQ, 1);
    reqTime_.store(frc::Timer::GetFPGATimestamp().value());
    isRequesting_.store(true);
}

// @brief Init function
void LidarReader::CoreInit(){
    thread_ = std::thread([this]{this->PeriodicLoop();});
    thread_.detach();
}


/// @brief Should be called in perioidic - reads port and stores data
void LidarReader::PeriodicLoop(){
    unsigned char readData[8]; //[4 old values (for checks/adjustments), RES, cone, cube, check]
    int readIndex = 0; //Number of bytes currently read

    while(true){
        double time = frc::Timer::GetFPGATimestamp().value();
        if(autoRequest_.load()){
            RequestData();
        }

        //Check how stale data is
        if(time - data_.readTime.load() > LidarReaderConstants::VALID_DATA_TIME){
            data_.isValid.store(false);
        }

        //If not requesting, do nothing
        if(!isRequesting_.load()){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        //Check if it is responding in time
        if(time - reqTime_ > LidarReaderConstants::RESPONSE_TIME){
            isRequesting_ = false;
            port_.Reset();
            std::cout<<"Failed Requesting Lidar Data"<<std::endl;
        }
        readPortData(readData, readIndex);
    }
}

/// @brief Sets the autorequest config
/// @param autoRequest requests data if invalid data
void LidarReader::setAutoRequest(bool autoRequest){
    autoRequest_ = autoRequest;
}

/// @brief Reads the data from the port
void LidarReader::readPortData(unsigned char (&readData)[8], int& readIndex){
    //Check buffer size
    int bufferSize = port_.GetBytesReceived();
    if(bufferSize == 0){
        return;
    }
    
    //Read Data
    if(bufferSize > 1000){ //If disconnected, port gets filled
        std::cout<<"Reset Port"<<std::endl;
        port_.Reset();
        return;
    }

    char clearBuffer[256];
    while (bufferSize > 8){ //Clear port
        int cleared = port_.Read(clearBuffer, std::min(256, bufferSize - 8));
        bufferSize -= cleared;
    }

    char readBuffer[8]; //Reads to this char array
    int count = port_.Read(readBuffer, 8);

    //Verify and store data
    for(int i = 0; i < count; i++){
        readData[readIndex + 4] = readBuffer[i]; //Stores in last 4 bytes of array
        readIndex++;
        if(readIndex == 4){ //Has read 4 bytes
            readIndex = 0;
            isRequesting_.store(false);

            //Check recorded data is good
            if(isValidData(readData + 4)){
                storeData(readData + 4);
            }
            else{
                findOffset(readData, readIndex);
            }

            //Shifts the array by 4 to left
            std::copy(readData+4, readData+8, readData);
        }
    }
}

/// @brief stores the data
void LidarReader::storeData(const unsigned char data[4]){
    data_.hasCone.store((data[1] != LidarReaderConstants::NO_READ) && (data[1] < 31));
    data_.hasCube.store((data[2] != LidarReaderConstants::NO_READ));
    data_.conePos.store(data_.hasCone.load()? ((double)data[1]) : LidarReaderConstants::DEFAULT_POSITION);
    data_.cubePos.store(data_.hasCube.load()? ((double)data[2]) : LidarReaderConstants::DEFAULT_POSITION);
    data_.isValid.store(true);
    data_.readTime.store(frc::Timer::GetFPGATimestamp().value());
}

/// @brief Checks if data is valid via the check sum and response key
/// @param data 4 chars
/// @return boolean
bool LidarReader::isValidData(const unsigned char data[4]){
    //Respond invalid
    if(data[0] != LidarReaderConstants::RES){
        return false;
    }
    //Check sum
    if(((data[0] + data[1] + data[2]) % 256) != data[3]){
        return false;
    }
    return true;
}

/// @brief finds the offset, then changes the readindex
void LidarReader::findOffset(unsigned char (&readData)[8], int& readIndex){
    //Check all possible offsets
    for(int off = 1; off < 4; off++){
        if(isValidData(readData + off)){
            storeData(readData + off);
            //Shift data left
            std::copy(readData + off, readData + 8, readData);
            readIndex = off+1;
            return;
        }
    }
}

/// @brief Gets the current data
/// @return LidarData struct with all info
LidarReader::LidarData LidarReader::getData(){
    LidarData newData;
    newData.conePos = data_.conePos.load();
    newData.cubePos = data_.cubePos.load();
    newData.hasCone = data_.hasCone.load();
    newData.hasCube = (data_.hasCube.load());
    newData.isValid = (data_.isValid.load());
    newData.readTime = (data_.readTime.load());
    return newData;
}

/// @brief Gets the cone position in cm
/// @return cm
double LidarReader::getConePos(){
    return data_.conePos;
}

/// @brief Gets the cube position in cm
/// @return cm
double LidarReader::getCubePos(){
    return data_.cubePos;
}

/// @brief returns if the lidar senses a cone
/// @return if the lidar senses a cone
bool LidarReader::hasCone(){
    return data_.hasCone.load();
}

/// @brief returns if the lidar senses a cube
/// @return if the lidar senses a cube
bool LidarReader::hasCube(){
    return data_.hasCube.load();
}

/// @brief returns if the data is valid (collected within some time)
/// @return if data is valid
bool LidarReader::validData(){
    return data_.isValid.load();
}

/// @brief Returns when the data was recorded
/// @return seconds
double LidarReader::getRecordedTime(){
    return data_.readTime.load();
}

void LidarReader::CoreShuffleboardInit(){
    frc::SmartDashboard::PutBoolean("Lidar Stale", !data_.isValid.load());
    frc::SmartDashboard::PutNumber("Port read size", port_.GetBytesReceived());

    frc::SmartDashboard::PutBoolean("Get Data", false);
    frc::SmartDashboard::PutBoolean("Auto Request", false);
    frc::SmartDashboard::PutNumber("Cone Pos", getConePos());
    frc::SmartDashboard::PutNumber("Cube Pos", getCubePos());
    frc::SmartDashboard::PutBoolean("Has Cube", hasCube());
    frc::SmartDashboard::PutBoolean("Has Cone", hasCone());
    frc::SmartDashboard::PutBoolean("Is Requesting", isRequesting_.load());
    frc::SmartDashboard::PutNumber("Read Time", data_.readTime.load());
}

void LidarReader::CoreShuffleboardPeriodic(){
    frc::SmartDashboard::PutBoolean("Lidar Stale", !data_.isValid.load());
    frc::SmartDashboard::PutNumber("Port read size", port_.GetBytesReceived());

    setAutoRequest(frc::SmartDashboard::GetBoolean("Auto Request", false));
    if(frc::SmartDashboard::GetBoolean("Get Data", false)){
        RequestData();
        frc::SmartDashboard::PutBoolean("Get Data", false);
    }
    frc::SmartDashboard::PutNumber("Cone Pos", getConePos());
    frc::SmartDashboard::PutNumber("Cube Pos", getCubePos());
    frc::SmartDashboard::PutBoolean("Has Cube", hasCube());
    frc::SmartDashboard::PutBoolean("Has Cone", hasCone());
    frc::SmartDashboard::PutBoolean("Is Requesting", isRequesting_.load());
    frc::SmartDashboard::PutNumber("Read Time", data_.readTime.load());
}