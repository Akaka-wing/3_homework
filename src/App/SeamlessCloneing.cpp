#include "SeamlessCloneing.h"

using std::vector;

SeamlessCloneing::SeamlessCloneing() {

}

SeamlessCloneing::~SeamlessCloneing() {

}

SeamlessCloneing::SeamlessCloneing(cv::Mat source, cv::Mat destination, QPoint start_source, QPoint end_source, QPoint start_destination)
{
	source_start.x = start_source.rx();
	source_start.y = start_source.ry();
	source_end.x = end_source.rx();
	source_end.y = end_source.ry();

	destination_start.x = start_destination.rx();
	destination_start.y = start_destination.ry();

	if (!point_matrix.empty())
		point_matrix.clear();
	if (!A_triplet.empty())
		A_triplet.clear();

	image_destination = destination;
	image_source = source;

	GetPointMatrix();
	GetNeighborhood();

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

void SeamlessCloneing::GetIsChosenMatrix()
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

int SeamlessCloneing::GetOrder(cv::Point point)
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
				
				A_triplet.push_back(Triplet<double>(ord, GetOrder(point_tmp), -1));
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

cv::Point SeamlessCloneing::Translate(cv::Point destination_point)
{
	cv::Point point_tmp;
	point_tmp.x = destination_point.x + source_start.x - destination_start.x;
	point_tmp.y = destination_point.y + source_start.y - destination_start.y;
	return point_tmp;
}
void SeamlessCloneing::ImportingCloneing() 
{
	VectorXd br = VectorXd::Zero(order_total);
	VectorXd bg = VectorXd::Zero(order_total);
	VectorXd bb = VectorXd::Zero(order_total);

	int ord = 0;
	for (size_t i = 0; i < point_matrix.size(); i++)
	{
		for (size_t j = 0; j < point_matrix[i].size(); j++)
		{
			for (size_t k = 0; k < neighborhood_matrix[i][j].boundary_points.size(); k++)
			{
				cv::Vec3b q_destination_data = image_destination.at<cv::Vec3b>(neighborhood_matrix[i][j].boundary_points[k]);
				cv::Vec3b q_source_data = image_source.at<cv::Vec3b>(Translate(neighborhood_matrix[i][j].boundary_points[k]));
				cv::Vec3b p_source_data = image_source.at<cv::Vec3b>(Translate(point_matrix[i][j].Axes));
				
				br(ord) += p_source_data[0] - q_source_data[0];
				bg(ord) += p_source_data[1] - q_source_data[1];
				bb(ord) += p_source_data[2] - q_source_data[2];

				br(ord) += q_destination_data[0];
				bg(ord) += q_destination_data[1];
				bb(ord) += q_destination_data[2];
			}
			for (size_t k = 0; k < neighborhood_matrix[i][j].interior_points.size(); k++)
			{
				cv::Vec3b q_source_data = image_source.at<cv::Vec3b>(Translate(neighborhood_matrix[i][j].interior_points[k]));
				cv::Vec3b p_source_data = image_source.at<cv::Vec3b>(Translate(point_matrix[i][j].Axes));

				br(ord) += p_source_data[0] - q_source_data[0];
				bg(ord) += p_source_data[1] - q_source_data[1];
				bb(ord) += p_source_data[2] - q_source_data[2];
			
			}
			ord++;
		}

	}

	if (solver.info() != Success)
	{
		std::cout << "Error:can't solve the equation" << std::endl;
		return;
	}
	VectorXd r = solver.solve(br);
	VectorXd g = solver.solve(bg);
	VectorXd b = solver.solve(bb);

	FillImage(r,g,b);
	////consider new and old pixels
	//int ord_tmp = 0;

	//for (int i = 0; i < point_matrix.size(); i++)
	//{
	//	for (int j = 0; j < point_matrix[i].size(); j++)
	//	{
	//		int x = point_matrix[i][j].Axes.x;
	//		int y = point_matrix[i][j].Axes.y;

	//		std::cout << image_destination.at<cv::Vec3b>(y, x)[0] << " : " << Fix(r(ord_tmp)) << "    ";
	//		std::cout << image_destination.at<cv::Vec3b>(y, x)[1] << " : " << Fix(g(ord_tmp)) << "    ";
	//		std::cout << image_destination.at<cv::Vec3b>(y, x)[2] << " : " << Fix(b(ord_tmp)) << "    ";

	//		ord_tmp++;
	//	}
	//	std::cout << std::endl << std::endl;
	//}
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

void SeamlessCloneing::MixingCloneing()
{
	VectorXd br = VectorXd::Zero(order_total);
	VectorXd bg = VectorXd::Zero(order_total);
	VectorXd bb = VectorXd::Zero(order_total);

	int ord = 0;
	for (size_t i = 0; i < point_matrix.size(); i++)
	{
		for (size_t j = 0; j < point_matrix[i].size(); j++)
		{
			for (size_t k = 0; k < neighborhood_matrix[i][j].boundary_points.size(); k++)
			{
				cv::Vec3b q_destination_data = image_destination.at<cv::Vec3b>(neighborhood_matrix[i][j].boundary_points[k]);
				cv::Vec3b p_destination_data = image_destination.at<cv::Vec3b>(point_matrix[i][j].Axes);
				cv::Vec3b q_source_data = image_source.at<cv::Vec3b>(Translate(neighborhood_matrix[i][j].boundary_points[k]));
				cv::Vec3b p_source_data = image_source.at<cv::Vec3b>(Translate(point_matrix[i][j].Axes));

				if (compare(p_source_data, q_source_data, p_destination_data, q_destination_data))
				{
					br(ord) += p_source_data[0] - q_source_data[0];
					bg(ord) += p_source_data[1] - q_source_data[1];
					bb(ord) += p_source_data[2] - q_source_data[2];
				}
				else
				{
					br(ord) += p_destination_data[0] - q_destination_data[0];
					bg(ord) += p_destination_data[1] - q_destination_data[1];
					bb(ord) += p_destination_data[2] - q_destination_data[2];
				}

				br(ord) += q_destination_data[0];
				bg(ord) += q_destination_data[1];
				bb(ord) += q_destination_data[2];
			}
			for (size_t k = 0; k < neighborhood_matrix[i][j].interior_points.size(); k++)
			{
				cv::Vec3b q_destination_data = image_destination.at<cv::Vec3b>(neighborhood_matrix[i][j].interior_points[k]);
				cv::Vec3b p_destination_data = image_destination.at<cv::Vec3b>(point_matrix[i][j].Axes);
				cv::Vec3b q_source_data = image_source.at<cv::Vec3b>(Translate(neighborhood_matrix[i][j].interior_points[k]));
				cv::Vec3b p_source_data = image_source.at<cv::Vec3b>(Translate(point_matrix[i][j].Axes));

				if (compare(p_source_data, q_source_data, p_destination_data, q_destination_data))
				{
					br(ord) += p_source_data[0] - q_source_data[0];
					bg(ord) += p_source_data[1] - q_source_data[1];
					bb(ord) += p_source_data[2] - q_source_data[2];
				}
				else
				{
					br(ord) += p_destination_data[0] - q_destination_data[0];
					bg(ord) += p_destination_data[1] - q_destination_data[1];
					bb(ord) += p_destination_data[2] - q_destination_data[2];
				}

			}
			ord++;
		}

	}

	if (solver.info() != Success)
	{
		std::cout << "Error:can't solve the equation" << std::endl;
		return;
	}
	VectorXd r = solver.solve(br);
	VectorXd g = solver.solve(bg);
	VectorXd b = solver.solve(bb);

	FillImage(r, g, b);
}

unsigned char SeamlessCloneing::fix(double data)
{
	if (data > 255)
		return 255;
	if (data < 0)
		return 0;
	return unsigned char(data);
}

void SeamlessCloneing::FillImage(VectorXd r, VectorXd g, VectorXd b)
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

cv::Mat SeamlessCloneing::GetImage() 
{
	return image_destination;
}