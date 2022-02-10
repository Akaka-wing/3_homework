#pragma once

#include <QWidget>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <eigen/dense>
#include <eigen/sparse>
#include <Eigen/SparseCholesky>
#include <iostream>
#include <vector>

//using namespace std;
using namespace Eigen;

class SeamlessCloneing
{
public:
	SeamlessCloneing();
	~SeamlessCloneing();

	SeamlessCloneing(cv::Mat source, cv::Mat destination, QPoint start, QPoint end);
	void GetPointMatrix();
	void GetIsChosenMatrix();
	int  GetOrder(cv::Point point);
	void GetNeighborhood();
	
	
	void GetTriplet();
	void GetSparseMatrix();
	void Cloneing();											//set right-side of the equation	
	int Fix(double data);
	void FillImage();											//fill desination image with the solution

	cv::Mat GetImage();
public:
	typedef struct {
		cv::Point Axes;
		int order;
	}PointType;

	typedef struct {
		std::vector<cv::Point> interior_points;
		std::vector<cv::Point> boundary_points;
	}Neighborhood;

private:
	cv::Mat image_source;										//The cut one
	cv::Mat image_destination;									//The original background

	int order_total = 0;
	VectorXd r, g, b;
	cv::Point point_start, point_end;

	SimplicialLDLT<SparseMatrix<double>> solver;
	std::vector<Triplet<double>> A_triplet;

	std::vector<std::vector<bool>> is_chosen_matrix;
	std::vector<std::vector<PointType>> point_matrix;						//position of selected points of source image
	std::vector<std::vector<Neighborhood>> neighborhood_matrix;			//neighborhoods of selected points
	
	
};

