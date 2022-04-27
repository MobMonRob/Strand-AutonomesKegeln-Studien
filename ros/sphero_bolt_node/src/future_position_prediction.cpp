
#include "FuturePositionPrediction.h"
#include <cmath>
#include <ros/ros.h>

void FuturePositionPrediction::add(BufferedPosition position) {
    buffer.push_back(position);
    while (buffer.size() > bufferSize)
        buffer.pop_front();
}

size_t FuturePositionPrediction::getBufferedPosition() {
    size_t offset = 4;
    while(offset < buffer.size() && 
        buffer[buffer.size() - offset].ballFound == false)
        offset++;

    auto result = buffer[offset];
    if (!result.ballFound)
        throw NoComparablePositionsInBufferException();
    else {
        return offset;
    }
}


pcl::PointXYZ FuturePositionPrediction::predictPosition(pcl::PointXYZ currentPosition) {
    if (buffer.size() < 4)
        return currentPosition;
    
    size_t offset = 4;
    while(offset < buffer.size() && 
        buffer[buffer.size() - offset].ballFound == false)
        offset++;
    
    try {
        size_t offset = getBufferedPosition();
        auto comparedPosition = buffer[offset];
        double time = offset * 0.04;
         
        float xVelocity = (currentPosition.x - comparedPosition.position.x) / time;
        float yVelocity = (currentPosition.y - comparedPosition.position.y) / time;
        pcl::PointXYZ futurePosition(currentPosition.x + xVelocity * time, currentPosition.y + yVelocity * time, currentPosition.z); 

        double absoluteVelocity = sqrtf(powf(xVelocity, 2.0) + powf(yVelocity, 2.0)); 
        ROS_INFO_STREAM("Ball velocity: " << absoluteVelocity);
        ROS_INFO_STREAM("Predicted position" << futurePosition.x << " y: " << futurePosition.y << " z: " << futurePosition.z);
        return futurePosition;

    } catch (std::exception& e) {
        return currentPosition;
    }
    
}