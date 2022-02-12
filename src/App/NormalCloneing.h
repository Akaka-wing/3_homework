#pragma once
#include "SeamlessCloneing.h"
class NormalCloneing :
    public SeamlessCloneing
{
public:
    NormalCloneing(cv::Mat source, cv::Mat destination, cv::Point start_source, cv::Point end_source, cv::Point start_destination);
    ~NormalCloneing();

    void Cloneing();
};

