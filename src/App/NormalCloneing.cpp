#include "NormalCloneing.h"

NormalCloneing::NormalCloneing(cv::Mat source, cv::Mat destination, cv::Point start_source, cv::Point end_source, cv::Point start_destination)
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
	//GetNeighborhood();

}

NormalCloneing::~NormalCloneing()
{

}

void NormalCloneing::Cloneing()
{

	// Restore image
	//	*(image_) = *(image_backup_);

	// Paste
	for (size_t i = 0; i < point_matrix.size(); i++)
	{
		for (size_t j = 0; j < point_matrix[i].size(); j++)
		{
			image_destination.at<cv::Vec3b>(point_matrix[i][j].Axes) = image_source.at<cv::Vec3b>(Translate(point_matrix[i][j].Axes));
		}
	}

}