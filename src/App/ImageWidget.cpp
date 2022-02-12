#include "ImageWidget.h"


//using std::cout;
//using std::endl;

ImageWidget::ImageWidget(ChildWindow* relatewindow)
{
	//image_ = new QImage();
	//image_backup_ = new QImage();

	clone_status_ = kkNone;
	draw_status_ = kNone;
	is_choosing_ = false;
	is_pasting_ = false;

	point_start_ = QPoint(0, 0);
	point_end_ = QPoint(0, 0);

	source_window_ = NULL;
}

ImageWidget::~ImageWidget(void)
{
}

int ImageWidget::ImageWidth()
{
	//return image_->width();
	return image_mat_.cols;
}

int ImageWidget::ImageHeight()
{
	//return image_->height();
	return image_mat_.rows;
}

void ImageWidget::set_clone_status_to_normal()
{
	clone_status_ = kNormal;
}

void ImageWidget::set_clone_status_to_importing()
{
	clone_status_ = kImporting;
}

void ImageWidget::set_clone_status_to_mixing()
{
	clone_status_ = kMixing;
}

void ImageWidget::set_draw_status_to_choose()
{
	draw_status_ = kChoose;	
}

void ImageWidget::set_draw_status_to_paste()
{
	draw_status_ = kPaste;
}

cv::Mat ImageWidget::image()
{
	return image_mat_;
}

void ImageWidget::set_source_window(ChildWindow* childwindow)
{
	source_window_ = childwindow;
}

void ImageWidget::paintEvent(QPaintEvent* paintevent)
{
	QPainter painter;
	painter.begin(this);

	// Draw background
	painter.setBrush(Qt::lightGray);
	QRect back_rect(0, 0, width(), height());
	painter.drawRect(back_rect);

	// Draw image
	//QRect rect = QRect(0, 0, image_->width(), image_->height());
	//painter.drawImage(rect, *image_);
	QImage image_show = QImage((unsigned char*)(image_mat_.data), image_mat_.cols, image_mat_.rows, image_mat_.step, QImage::Format_RGB888);
	QRect rect = QRect(0, 0, image_show.width(), image_show.height());
	painter.drawImage(rect, image_show);

	// Draw choose region
	painter.setBrush(Qt::NoBrush);
	painter.setPen(Qt::red);
	painter.drawRect(point_start_.x(), point_start_.y(),
		point_end_.x() - point_start_.x(), point_end_.y() - point_start_.y());

	painter.end();
}

void ImageWidget::mousePressEvent(QMouseEvent* mouseevent)
{
	if (Qt::LeftButton == mouseevent->button())
	{
		switch (draw_status_)
		{
		case kChoose:
		{
			is_choosing_ = true;
			point_start_ = point_end_ = mouseevent->pos();
			break;
		}
		case kPaste:
		{
			is_pasting_ = true;

			// Start point in object image
			int xpos = mouseevent->pos().rx();
			int ypos = mouseevent->pos().ry();

			// Start point in source image
			int xsourcepos = source_window_->imagewidget_->point_start_.rx();
			int ysourcepos = source_window_->imagewidget_->point_start_.ry();

			// Width and Height of rectangle region
			int w = source_window_->imagewidget_->point_end_.rx()
				- source_window_->imagewidget_->point_start_.rx() + 1;
			int h = source_window_->imagewidget_->point_end_.ry()
				- source_window_->imagewidget_->point_start_.ry() + 1;

			//set point_start_ && point_end_
			/*point_start_ = mouseevent->pos();
			point_end_.rx() = point_start_.rx() + w - 1;
			point_end_.ry() = point_start_.ry() + h - 1;

			std::cout << "selected size: " << point_end_.rx() - point_start_.rx() << " x " << point_end_.ry() - point_start_.ry() << std::endl;
			std::cout << "selected position: " << std::endl;
			std::cout << "(" << point_start_.rx() << " , " << point_start_.ry() << ")" << std::endl;
			std::cout << "(" << point_end_.rx() << " , " << point_end_.ry() << ")" << std::endl;*/

			if ((xpos > 0) && (ypos > 0) && (xpos + w < image_mat_.cols) && (ypos + h < image_mat_.rows))
			{
				cv::Mat image_tmp_ = image_mat_.clone();
				
				cv::Point point_start_source_ = cv::Point(xsourcepos, ysourcepos);
				cv::Point point_end_source_ = cv::Point(xsourcepos + w - 1, ysourcepos + h - 1);

				switch (clone_status_)
				{
				case kNormal:
				{
					NormalCloneing* normal_clone = new NormalCloneing(source_window_->imagewidget_->image(), image_tmp_, point_start_source_, point_end_source_, cv::Point(xpos, ypos));
					normal_clone->Cloneing();
					image_mat_ = normal_clone->GetImage();
					break;
				}
				case kImporting:
				{
					ImportingCloneing* normal_clone = new ImportingCloneing(source_window_->imagewidget_->image(), image_tmp_, point_start_source_, point_end_source_, cv::Point(xpos, ypos));
					normal_clone->Cloneing();
					image_mat_ = normal_clone->GetImage();
					break;
				}
				case kMixing:
				{
					MixingCloneing* normal_clone = new MixingCloneing(source_window_->imagewidget_->image(), image_tmp_, point_start_source_, point_end_source_, cv::Point(xpos, ypos));
					normal_clone->Cloneing();
					image_mat_ = normal_clone->GetImage();
					break;
				}
				default:
					break;
				}
			}
			
			update();
		}
		default:
			break;
		}
	}
}

void ImageWidget::mouseMoveEvent(QMouseEvent* mouseevent)
{
	switch (draw_status_)
	{
	case kChoose:
		// Store point position for rectangle region
		if (is_choosing_)
		{
			point_end_ = mouseevent->pos();
		}
		break;

	case kPaste:
	{
		// Paste rectangle region to object image
		if (is_pasting_)
		{
			// Start point in object image
			int xpos = mouseevent->pos().rx();
			int ypos = mouseevent->pos().ry();

			// Start point in source image
			int xsourcepos = source_window_->imagewidget_->point_start_.rx();
			int ysourcepos = source_window_->imagewidget_->point_start_.ry();

			// Width and Height of rectangle region
			int w = source_window_->imagewidget_->point_end_.rx()
				- source_window_->imagewidget_->point_start_.rx() + 1;
			int h = source_window_->imagewidget_->point_end_.ry()
				- source_window_->imagewidget_->point_start_.ry() + 1;

			// Paste
			if ((xpos > 0) && (ypos > 0) && (xpos + w < image_mat_.cols) && (ypos + h < image_mat_.rows))
			{
				// Restore image 
				image_mat_ = image_mat_backup_.clone();
				//*(image_) = *(image_backup_);

				// Paste
				//image_mat_ = image_mat_.clone();
				for (int i = 0; i < w; i++)
				{
					for (int j = 0; j < h; j++)
					{
						image_mat_.at<cv::Vec3b>(ypos + j, xpos + i) = source_window_->imagewidget_->image().at<cv::Vec3b>(ysourcepos + j, xsourcepos + i);
					}
				}
			}
		}
		update();
	}
	default:
		break;
	}

	update();
}

void ImageWidget::mouseReleaseEvent(QMouseEvent* mouseevent)
{
	switch (draw_status_)
	{
	case kChoose:
		if (is_choosing_)
		{
			point_end_ = mouseevent->pos();
			std::cout << "selected size: " << point_end_.rx() - point_start_.rx() << " x " << point_end_.ry() - point_start_.ry() << std::endl;
			std::cout << "selected position: " << std::endl;
			std::cout << "(" << point_start_.rx() << " , " << point_start_.ry() << ")" << std::endl;
			std::cout << "(" << point_end_.rx() << " , " << point_end_.ry() << ")" << std::endl;

			is_choosing_ = false;
			draw_status_ = kNone;
		}
	case kPaste:
		if (is_pasting_)
		{
			is_pasting_ = false;
			draw_status_ = kNone;
			clone_status_ = kkNone;
		}

	default:
		break;
	}

	update();
}

void ImageWidget::Open(QString filename)
{	
	//Open File
	//QString fileName = QFileDialog::getOpenFileName(this, tr("Read Image"), ".", tr("Images(*.bmp *.png *.jpg)"));
	
	//Load file
	if (!filename.isEmpty())
	{
		image_mat_ = cv::imread(filename.toLatin1().data());
		cv::cvtColor(image_mat_, image_mat_, cv::COLOR_BGR2RGB);
		image_mat_backup_ = image_mat_.clone();
	}

	/*if (!filename.isEmpty())
	{
		image_->load(filename);
		*(image_backup_) = *(image_);
	}*/

	//	setFixedSize(image_->width(), image_->height());
	//	relate_window_->setWindowFlags(Qt::Dialog);
	//	relate_window_->setFixedSize(QSize(image_->width(), image_->height()));
	//	relate_window_->setWindowFlags(Qt::SubWindow);

		//image_->invertPixels(QImage::InvertRgb);
		//*(image_) = image_->mirrored(true, true);
		//*(image_) = image_->rgbSwapped();
	std::cout << "image size: " << image_mat_.cols << ' ' << image_mat_.rows << std::endl;
	update();
}

void ImageWidget::Save()
{
	SaveAs();
}

void ImageWidget::SaveAs()
{
	QString filename = QFileDialog::getSaveFileName(this, tr("Save Image"), ".", tr("Images(*.bmp *.png *.jpg)"));
	if (filename.isNull())
	{
		return;
	}

	cv::Mat image_save;
	cv::cvtColor(image_mat_, image_save, cv::COLOR_BGR2RGB);
	cv::imwrite(filename.toLatin1().data(), image_save);
	//image_->save(filename);
}



void ImageWidget::Invert()
{
	if (image_mat_.empty())
		return;
	cv::MatIterator_<cv::Vec3b> iter, iterend;
	for (iter = image_mat_.begin<cv::Vec3b>(), iterend = image_mat_.end<cv::Vec3b>(); iter != iterend; iter++) 
	{
		(*iter)[0] = 255 - (*iter)[0];
		(*iter)[1] = 255 - (*iter)[1];
		(*iter)[2] = 255 - (*iter)[2];
	}

	/*for (int i = 0; i < image_mat_cols; i++)
	{
		for (int j = 0; j < image_->height(); j++)
		{
			QRgb color = image_->pixel(i, j);
			image_->setPixel(i, j, qRgb(255 - qRed(color), 255 - qGreen(color), 255 - qBlue(color)));
		}
	}*/

	// equivalent member function of class QImage
	// image_->invertPixels(QImage::InvertRgb);
	update();
}

void ImageWidget::Mirror(bool ishorizontal, bool isvertical)
{
	if (image_mat_.empty())
		return;
	cv::Mat image_tmp_mat_ = image_mat_.clone();
	int width = image_mat_.cols;
	int height = image_mat_.rows;

	if (ishorizontal)
	{
		if (isvertical)
		{
			for (int i = 0; i < width; i++)
			{
				for (int j = 0; j < height; j++)
				{
					image_mat_.at<cv::Vec3b>(j, i) = image_tmp_mat_.at<cv::Vec3b>(height - 1 - j, width - 1 - i);
					//image_->setPixel(i, j, image_tmp.pixel(width - 1 - i, height - 1 - j));

				}
			}
		}
		else
		{
			for (int i = 0; i < width; i++)
			{
				for (int j = 0; j < height; j++)
				{
					image_mat_.at<cv::Vec3b>(j, i) = image_tmp_mat_.at<cv::Vec3b>(height - 1 - j, i);
					//image_->setPixel(i, j, image_tmp.pixel(i, height - 1 - j));
				}
			}
		}

	}
	else
	{
		if (isvertical)
		{
			for (int i = 0; i < width; i++)
			{
				for (int j = 0; j < height; j++)
				{
					image_mat_.at<cv::Vec3b>(j, i) = image_tmp_mat_.at<cv::Vec3b>(j, width - 1 - i);
					//image_->setPixel(i, j, image_tmp.pixel(width - 1 - i, j));
					std::cout << image_mat_.at<cv::Vec3b>(j, i)[0] << "   " << image_mat_.at<cv::Vec3b>(j, i)[1] << "   " << image_mat_.at<cv::Vec3b>(j, i)[2] << std::endl;
				}
			}
		}
	}

	// equivalent member function of class QImage
	//*(image_) = image_->mirrored(true, true);
	update();
}

void ImageWidget::TurnGray()
{
	if (image_mat_.empty())
		return;
	cv::MatIterator_<cv::Vec3b> iter, iterend;
	for (iter = image_mat_.begin<cv::Vec3b>(), iterend = image_mat_.end<cv::Vec3b>(); iter != iterend; iter++)
	{
		int tmp = ((*iter)[0] + (*iter)[1] + (*iter)[2]) / 3;
		(*iter)[0] = tmp;
		(*iter)[1] = tmp;
		(*iter)[2] = tmp;
	}
	/*for (int i = 0; i < image_mat_.cols; i++)
	{
		for (int j = 0; j < image_mat_.rows; j++)
		{
			QRgb color = image_->pixel(i, j);
			int gray_value = (qRed(color) + qGreen(color) + qBlue(color)) / 3;
			image_->setPixel(i, j, qRgb(gray_value, gray_value, gray_value));
		}
	}*/

	update();
}

void ImageWidget::Restore()
{
	image_mat_ = image_mat_backup_.clone();
	point_start_ = point_end_ = QPoint(0, 0);
	update();
}
