#include "MixingCloneing.h"

MixingCloneing::~MixingCloneing()
{

}

void MixingCloneing::Cloneing()
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

	imagefill(r, g, b);
}