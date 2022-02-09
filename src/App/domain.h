#pragma once

#include <QWidget>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

using namespace std;

class domain
{
public:
	domain();
	~domain();

private:
	vector<vector<int>> A;
};

