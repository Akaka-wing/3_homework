//updata of 2022.2.11 22:59
//need to list:
//警告也给他消掉
//画的红线给他标出来，想法是不给它定位point_end,只给point_start,这样是不是只有一个红点这样子
//modify the point_matrix from the destination posiotion to source position
//paste mode: normal cloneing, importing cloneing,  or Mixing cloneing
//把这个类拆成三个小类
//add polygon painting


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

	SeamlessCloneing(cv::Mat source, cv::Mat destination, cv::Point start_source, cv::Point end_source, cv::Point start_destination);
	
	void GetPointMatrix();
	void GetNeighborhood();

	void GetTriplet();
	void GetSparseMatrix();
	
	virtual void Cloneing() = 0;											//set right-side of the equation	
	void MixingCloneing();

	cv::Mat GetImage();

protected:
	void getischosenmatrix();
	int getorder(cv::Point point);
	cv::Point Translate(cv::Point destination_point);			//point translate from destination to source position
	bool compare(cv::Vec3b p_s, cv::Vec3b q_s, cv::Vec3b p_d, cv::Vec3b q_d);
	unsigned char fix(double data);
	void imagefill(VectorXd r, VectorXd g, VectorXd b);											//fill desination image with the solution


public:
	typedef struct {
		cv::Point Axes;
		int order;
	}PointType;

	typedef struct {
		std::vector<cv::Point> interior_points;
		std::vector<cv::Point> boundary_points;
	}Neighborhood;

protected:
	cv::Mat image_source;										//source image
	cv::Mat image_destination;									//The original background

	int order_total = 0;
	cv::Point source_start, source_end;
	cv::Point destination_start;

	SimplicialLDLT<SparseMatrix<double>> solver;
	std::vector<Triplet<double>> A_triplet;

	std::vector<std::vector<bool>> is_chosen_matrix;
	std::vector<std::vector<PointType>> point_matrix;						//position of selected points of source image
	std::vector<std::vector<Neighborhood>> neighborhood_matrix;			//neighborhoods of selected points
	
	
};

