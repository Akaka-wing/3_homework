#include "SeamlessCloneing.h"

using std::vector;

SeamlessCloneing::SeamlessCloneing() {

}

SeamlessCloneing::~SeamlessCloneing() {

}

SeamlessCloneing::SeamlessCloneing(cv::Mat source, cv::Mat destination, cv::Point start_source, cv::Point end_source, cv::Point start_destination)
{
	source_start = start_source;
	source_end = end_source;

	destination_start = start_destination;

	if (!point_matrix.empty())
		point_matrix.clear();
	if (!A_triplet.empty())
		A_triplet.clear();

	image_destination = destination;
	image_source = source;

	GetPointMatrix();
	GetNeighborhood();

	GetTriplet();
	GetSparseMatrix();
}

void SeamlessCloneing::GetPointMatrix() {
	int ord = 0;

	int start_x = destination_start.x;
	int end_x = destination_start.x + source_end.x - source_start.x;
	int start_y = destination_start.y;
	int end_y = destination_start.y + source_end.y - source_start.y;

	for (size_t i = start_y; i <= end_y; i++)
	{
		vector<PointType> point_list;
		point_list.clear();
		for (size_t j = start_x; j <= end_x; j++)
		{
			PointType point_tmp;
			point_tmp.Axes = cv::Point(j, i);
			point_tmp.order = ord;
			point_list.push_back(point_tmp);

			ord++;
		}

		point_matrix.push_back(point_list);
	}
	order_total = ord;						//set of order_total
}


void SeamlessCloneing::GetNeighborhood() {
	for (size_t i = 0; i < point_matrix.size(); i++)
	{
		neighborhood_matrix.push_back(vector<Neighborhood>());
		for (size_t j = 0; j < point_matrix[i].size(); j++)
		{
			neighborhood_matrix[i].push_back(Neighborhood());
			vector<cv::Point> interior_list;
			vector<cv::Point> boundary_list;

			//consider the four neighborhoods of the chosen point
			int x = point_matrix[i][j].Axes.x;
			int y = point_matrix[i][j].Axes.y;
			
			//left_neighborhood
			if (j > 0 && point_matrix[i][j - 1].Axes.x == x - 1)
				interior_list.push_back(point_matrix[i][j - 1].Axes);
			else
				boundary_list.push_back(cv::Point(x - 1, y));

			//right_neighborhood
			if (j < point_matrix[i].size() - 1 && point_matrix[i][j + 1].Axes.x == x + 1)
				interior_list.push_back(point_matrix[i][j + 1].Axes);
			else
				boundary_list.push_back(cv::Point(x + 1, y));

			//up_neighborhood
			size_t k = 0;
			for (k = 0; i > 0 && k < point_matrix[i - 1].size(); k++)
				if (point_matrix[i - 1][k].Axes.x == x)
				{ 
					interior_list.push_back(point_matrix[i - 1][k].Axes);
					break;
				}	
			if (i == 0 || k == point_matrix[i - 1].size())
				boundary_list.push_back(cv::Point(x, y - 1));

			//down_neighborhood
			for(k = 0; i < point_matrix.size() - 1 && k < point_matrix[i + 1].size(); k++)
				if (point_matrix[i + 1][k].Axes.x == x)
				{
					interior_list.push_back(point_matrix[i + 1][k].Axes);
					break;
				}
			if (i == point_matrix.size() - 1 || k == point_matrix[i + 1].size())
				boundary_list.push_back(cv::Point(x, y + 1));

			neighborhood_matrix[i][j].interior_points = interior_list;
			neighborhood_matrix[i][j].boundary_points = boundary_list;
		}

	}
}



void SeamlessCloneing::GetTriplet() 
{

	int ord = 0;

	for (size_t i = 0; i < point_matrix.size(); i++) 
	{
		for (size_t j = 0; j < point_matrix[i].size(); j++)
		{
			A_triplet.push_back(Triplet<double>(ord, ord, 4));

			for (size_t k = 0; k < neighborhood_matrix[i][j].interior_points.size(); k++)
			{
				cv::Point point_tmp = neighborhood_matrix[i][j].interior_points[k];
				
				A_triplet.push_back(Triplet<double>(ord, getorder(point_tmp), -1));
			}

			ord++;
		}
	}
	order_total = ord;
}
void SeamlessCloneing::GetSparseMatrix() 
{
	SparseMatrix<double> sparse_matrix(order_total, order_total);
	sparse_matrix.setFromTriplets(A_triplet.begin(), A_triplet.end());
	solver.compute(sparse_matrix);
}

cv::Mat SeamlessCloneing::GetImage()
{
	return image_destination;
}

int SeamlessCloneing::getorder(cv::Point point)
{
	size_t row = point.y - point_matrix[0][0].Axes.y;
	int ord = 0;
	for (size_t i = 0; i < row; i++)
	{
		ord += point_matrix[i].size();
	}

	for (size_t col = 0; col < point_matrix[row].size(); col++)
	{
		if (point_matrix[row][col].Axes.x == point.x)
		{
			return ord;
		}
		ord++;
	}
	return -1;
}

cv::Point SeamlessCloneing::Translate(cv::Point destination_point)
{
	cv::Point point_tmp;
	point_tmp.x = destination_point.x + source_start.x - destination_start.x;
	point_tmp.y = destination_point.y + source_start.y - destination_start.y;
	return point_tmp;
}


bool SeamlessCloneing::compare(cv::Vec3b p_s, cv::Vec3b q_s, cv::Vec3b p_d, cv::Vec3b q_d)
{
	double dist_s = pow(p_s[0] - q_s[0], 2) + pow(p_s[1] - q_s[1], 2) + pow(p_s[2] - q_s[2], 2);
	double dist_d = pow(p_d[0] - q_d[0], 2) + pow(p_d[1] - q_d[1], 2) + pow(p_d[2] - q_d[2], 2);

	if (dist_s > dist_d)
		return true;
	else
		return false;
}

unsigned char SeamlessCloneing::fix(double data)
{
	if (data > 255)
		return 255;
	if (data < 0)
		return 0;
	return unsigned char(data);
}

void SeamlessCloneing::imagefill(VectorXd r, VectorXd g, VectorXd b)
{
	int ord_tmp = 0;

	for (size_t i = 0; i < point_matrix.size(); i++)
	{	
		for (size_t j = 0; j < point_matrix[i].size(); j++)
		{
			int x = point_matrix[i][j].Axes.x;
			int y = point_matrix[i][j].Axes.y;

			image_destination.at<cv::Vec3b>(y, x)[0] = fix(r(ord_tmp));
			image_destination.at<cv::Vec3b>(y, x)[1] = fix(g(ord_tmp));
			image_destination.at<cv::Vec3b>(y, x)[2] = fix(b(ord_tmp));

			ord_tmp++;
		}
	}
}

void SeamlessCloneing::getischosenmatrix()
{
	for (size_t i = 0; i < image_destination.rows; i++)
	{
		vector<bool> bool_list_tmp;

		for (size_t j = 0; j < image_destination.cols; j++)
		{
			if (j > source_start.x && j < source_end.x && i > source_start.y && i < source_end.y)
				bool_list_tmp.push_back(true);
			else
				bool_list_tmp.push_back(false);

		}
		is_chosen_matrix.push_back(bool_list_tmp);
	}
}
