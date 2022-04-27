#ifndef FUTUREPOSITIONPREDICTION_H
#define FUTUREPOSITIONPREDICTION_H
#include <deque>
#include <pcl/point_types.h>
#include <exception>

struct BufferedPosition {
    pcl::PointXYZ position;
    bool ballFound = false;
};

class FuturePositionPrediction {
    private:
        int scanFrequency;
        const size_t bufferSize = 10;
        const size_t compareOffset = 4;
        std::deque<BufferedPosition> buffer;

        size_t getBufferedPosition();

        struct NoComparablePositionsInBufferException : public std::exception {
            const char * what () const throw()
            {
                return "No ball was found in the buffered positions";
            }
        };

    public:

        FuturePositionPrediction(int scnFrequency) : scanFrequency(scnFrequency) {};
        void add(BufferedPosition);
        pcl::PointXYZ predictPosition(pcl::PointXYZ currentPosition);
};


#endif