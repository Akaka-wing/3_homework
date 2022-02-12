#pragma once
#include "SeamlessCloneing.h"

class MixingCloneing :
    public SeamlessCloneing
{
public:
    MixingCloneing(cv::Mat source, cv::Mat destination, cv::Point start_source, cv::Point end_source, cv::Point start_destination)
        : SeamlessCloneing(source, destination, start_source, end_source, start_destination)
    {}
    ~MixingCloneing();

    void Cloneing();
};

