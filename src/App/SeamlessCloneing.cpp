#include "SeamlessCloneing.h"

using std::vector;

SeamlessCloneing::SeamlessCloneing() {

}

SeamlessCloneing::~SeamlessCloneing() {

}

SeamlessCloneing::SeamlessCloneing(cv::Mat source, cv::Mat destination, QPoint start, QPoint end) 
{
	point_start.x = start.rx();
	point_start.y = start.ry();
	point_end.x = end.rx();
	point_end.y = end.ry();

	if (!point_matrix.empty())
		point_matrix.clear();
	if (!A_triplet.empty())
		A_triplet.clear();

	image_destination = destination;
	image_source = source;

	GetPointMatrix();
	GetNeighborhood();

	r = VectorXd::Zero(order_total);
	g = VectorXd::Zero(order_total);
	b = VectorXd::Zero(order_total);
}

void SeamlessCloneing::GetPointMatrix() {
	int ord = 0;

	int start_x = point_start.x;
	int end_x = point_end.x;
	int start_y = point_start.y;
	int end_y = point_end.y;

	for (int i = start_y; i <= end_y; i++)
	{
		vector<PointType> point_list;
		point_list.clear();
		for (int j = start_x; j <= end_x; j++)
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
	for (int i = 0; i < image_destination.rows; i++)
	{
		vector<bool> bool_list_tmp;
		
		for (int j = 0; j < image_destination.cols; j++)
		{
			if (j > point_start.x && j < point_end.x && i > point_start.y && i < point_end.y)
				bool_list_tmp.push_back(true);
			else
				bool_list_tmp.push_back(false);

		}
		is_chosen_matrix.push_back(bool_list_tmp);
	}
}

void SeamlessCloneing::GetNeighborhood() {
	for (int i = 0; i < point_matrix.size(); i++)
	{
		neighborhood_matrix.push_back(vector<Neighborhood>());
		for (int j = 0; j < point_matrix[i].size(); j++)
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
			int k = 0;
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
	int row = point.y - point_matrix[0][0].Axes.y;
	int ord = 0;
	for (int i = 0; i < row; i++)
	{
		ord += point_matrix[i].size();
	}
	
	for (int col = 0; col < point_matrix[row].size(); col++)
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

	for (int i = 0; i < point_matrix.size(); i++) 
	{
		for (int j = 0; j < point_matrix[i].size(); j++)
		{
			A_triplet.push_back(Triplet<double>(ord, ord, 4));

			for (int k = 0; k < neighborhood_matrix[i][j].interior_points.size(); k++)
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


void SeamlessCloneing::Cloneing() 
{
	VectorXd br = VectorXd::Zero(order_total);
	VectorXd bg = VectorXd::Zero(order_total);
	VectorXd bb = VectorXd::Zero(order_total);

	int ord = 0;
	for (int i = 0; i < point_matrix.size(); i++)
	{
		for (int j = 0; j < point_matrix[i].size(); j++)
		{
			for (int k = 0; k < neighborhood_matrix[i][j].boundary_points.size(); k++)
			{
				cv::Vec3b q_destination_data = image_destination.at<cv::Vec3b>(neighborhood_matrix[i][j].boundary_points[k]);
				cv::Vec3b q_source_data = image_source.at<cv::Vec3b>(neighborhood_matrix[i][j].boundary_points[k]);
				cv::Vec3b p_source_data = image_source.at<cv::Vec3b>(point_matrix[i][j].Axes);
				
				br(ord) += p_source_data[0] - q_source_data[0];
				bg(ord) += p_source_data[1] - q_source_data[1];
				bb(ord) += p_source_data[2] - q_source_data[2];

				br(ord) += q_destination_data[0];
				bg(ord) += q_destination_data[1];
				bb(ord) += q_destination_data[2];
			}
			for (int k = 0; k < neighborhood_matrix[i][j].interior_points.size(); k++)
			{
				cv::Vec3b q_source_data = image_source.at<cv::Vec3b>(neighborhood_matrix[i][j].interior_points[k]);
				cv::Vec3b p_source_data = image_source.at<cv::Vec3b>(point_matrix[i][j].Axes);

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
	r = solver.solve(br);
	g = solver.solve(bg);
	b = solver.solve(bb);
}

int SeamlessCloneing::Fix(double data)
{
	if (data > 253)
		return 255;
	if (data < 2)
		return 0;
	return int(data);
}

void SeamlessCloneing::FillImage() 
{
	int ord_tmp = 0;

	for (int i = 0; i < point_matrix.size(); i++) 
	{	
		for (int j = 0; j < point_matrix[i].size(); j++)
		{
			int x = point_matrix[i][j].Axes.x;
			int y = point_matrix[i][j].Axes.y;

			image_destination.at<cv::Vec3b>(y, x)[0] = Fix(r(ord_tmp));
			image_destination.at<cv::Vec3b>(y, x)[1] = Fix(g(ord_tmp));
			image_destination.at<cv::Vec3b>(y, x)[2] = Fix(b(ord_tmp));

			ord_tmp++;
		}
	}
}

cv::Mat SeamlessCloneing::GetImage() 
{
	return image_destination;
}