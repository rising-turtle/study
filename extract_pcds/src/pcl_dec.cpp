#include "pcl_dec.h"


cloud_t* createXYZPointCloud( const cv::Mat& depth_img, const cv::Mat& rgb_img)
{
	cloud_t* cloud (new cloud_t() );
	cloud->is_dense         = false; //single point of view, 2d rasterized NaN where no depth value was found

	//xtion 
	float fx = 1./ 577.24;//cam_info->K[0]; //(cloud->width >> 1) - 0.5f;
	float fy = 1./ 580.48;//cam_info->K[4]; //(cloud->width >> 1) - 0.5f;
	float cx = 302.49;//cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
	float cy = 198.71;//cam_info->K[5]; //(cloud->width >> 1) - 0.5f;
	int data_skip_step = 1;
	if(depth_img.rows % data_skip_step != 0 || depth_img.cols % data_skip_step != 0){
		printf("The parameter cloud_creation_skip_step is not a divisor of the depth image dimensions. This will most likely crash the program!\n");
	}
	cloud->height = ceil(depth_img.rows / static_cast<float>(data_skip_step));
	cloud->width = ceil(depth_img.cols / static_cast<float>(data_skip_step));
	int pixel_data_size = 3;
	bool encoding_bgr = true;
	//Assume BGR
	//char red_idx = 2, green_idx = 1, blue_idx = 0;
	//Assume RGB
	char red_idx = 0, green_idx = 1, blue_idx = 2;
	if(rgb_img.type() == CV_8UC1) pixel_data_size = 1;
	else if(encoding_bgr) { red_idx = 2; blue_idx = 0; }

	unsigned int color_row_step, color_pix_step, depth_pix_step, depth_row_step;
	color_pix_step = pixel_data_size * (rgb_img.cols / cloud->width);
	color_row_step = pixel_data_size * (rgb_img.rows / cloud->height -1 ) * rgb_img.cols;
	depth_pix_step = (depth_img.cols / cloud->width);
	depth_row_step = (depth_img.rows / cloud->height -1 ) * depth_img.cols;

	cloud->points.resize (cloud->height * cloud->width);

	// depth_img already has the desired dimensions, but rgb_img may be higher res.
	int color_idx = 0 * color_pix_step - 0 * color_row_step, depth_idx = 0; //FIXME: Hack for hard-coded calibration of color to depth
	double depth_scaling = 1.;
	float max_depth = -1;//MAX_XTION_RANGE;
	float min_depth = -1;//MIN_XTION_RANGE;
	if(max_depth < 0.0) max_depth = std::numeric_limits<float>::infinity();

	cloud_t::iterator pt_iter = cloud->begin();
	for (int v = 0; v < (int)rgb_img.rows; v += data_skip_step, color_idx += color_row_step, depth_idx += depth_row_step)
	{
		for (int u = 0; u < (int)rgb_img.cols; u += data_skip_step, color_idx += color_pix_step, depth_idx += depth_pix_step, ++pt_iter)
		{
			if(pt_iter == cloud->end()){
				break;
			}
			point_t& pt = *pt_iter;
			if(u < 0 || v < 0 || u >= depth_img.cols || v >= depth_img.rows){
				pt.x = std::numeric_limits<float>::quiet_NaN();
				pt.y = std::numeric_limits<float>::quiet_NaN();
				pt.z = std::numeric_limits<float>::quiet_NaN();
				continue;
			}

			float Z = depth_img.at<float>(depth_idx) * depth_scaling;

			// Check for invalid measurements
			if (!(Z >= min_depth)) //Should also be trigger on NaN//std::isnan (Z))
			{
				pt.x = (u - cx) * 1.0 * fx; //FIXME: better solution as to act as at 1meter?
				pt.y = (v - cy) * 1.0 * fy;
				pt.z = std::numeric_limits<float>::quiet_NaN();
			}
			else // Fill in XYZ
			{
				pt.x = (u - cx) * Z * fx;
				pt.y = (v - cy) * Z * fy;
				pt.z = Z;
			}
		}
	}
}

// depth_img must be 32fc1
color_cloud_t* createXYZRGBPointCloud (const cv::Mat& depth_img, const cv::Mat& rgb_img)
{
	color_cloud_t* cloud (new color_cloud_t() );
	cloud->is_dense         = false; //single point of view, 2d rasterized NaN where no depth value was found

	//xtion 
	float fx = 1./ 577.24;//cam_info->K[0]; //(cloud->width >> 1) - 0.5f;
	float fy = 1./ 580.48;//cam_info->K[4]; //(cloud->width >> 1) - 0.5f;
	float cx = 302.49;//cam_info->K[2]; //(cloud->width >> 1) - 0.5f;
	float cy = 198.71;//cam_info->K[5]; //(cloud->width >> 1) - 0.5f;
	int data_skip_step = 1;
	if(depth_img.rows % data_skip_step != 0 || depth_img.cols % data_skip_step != 0){
		printf("The parameter cloud_creation_skip_step is not a divisor of the depth image dimensions. This will most likely crash the program!\n");
	}
	cloud->height = ceil(depth_img.rows / static_cast<float>(data_skip_step));
	cloud->width = ceil(depth_img.cols / static_cast<float>(data_skip_step));
	int pixel_data_size = 3;
	bool encoding_bgr = true;
	//Assume BGR
	//char red_idx = 2, green_idx = 1, blue_idx = 0;
	//Assume RGB
	char red_idx = 0, green_idx = 1, blue_idx = 2;
	if(rgb_img.type() == CV_8UC1) pixel_data_size = 1;
	else if(encoding_bgr) { red_idx = 2; blue_idx = 0; }

	unsigned int color_row_step, color_pix_step, depth_pix_step, depth_row_step;
	color_pix_step = pixel_data_size * (rgb_img.cols / cloud->width);
	color_row_step = pixel_data_size * (rgb_img.rows / cloud->height -1 ) * rgb_img.cols;
	depth_pix_step = (depth_img.cols / cloud->width);
	depth_row_step = (depth_img.rows / cloud->height -1 ) * depth_img.cols;

	cloud->points.resize (cloud->height * cloud->width);

	// depth_img already has the desired dimensions, but rgb_img may be higher res.
	int color_idx = 0 * color_pix_step - 0 * color_row_step, depth_idx = 0; //FIXME: Hack for hard-coded calibration of color to depth
	double depth_scaling = 1.;
	float max_depth = -1;//MAX_XTION_RANGE;
	float min_depth = -1;//MIN_XTION_RANGE;
	if(max_depth < 0.0) max_depth = std::numeric_limits<float>::infinity();

	color_cloud_t::iterator pt_iter = cloud->begin();
	for (int v = 0; v < (int)rgb_img.rows; v += data_skip_step, color_idx += color_row_step, depth_idx += depth_row_step)
	{
		for (int u = 0; u < (int)rgb_img.cols; u += data_skip_step, color_idx += color_pix_step, depth_idx += depth_pix_step, ++pt_iter)
		{
			if(pt_iter == cloud->end()){
				break;
			}
			color_point_t& pt = *pt_iter;
			if(u < 0 || v < 0 || u >= depth_img.cols || v >= depth_img.rows){
				pt.x = std::numeric_limits<float>::quiet_NaN();
				pt.y = std::numeric_limits<float>::quiet_NaN();
				pt.z = std::numeric_limits<float>::quiet_NaN();
				continue;
			}

			float Z = depth_img.at<float>(depth_idx) * depth_scaling;

			// Check for invalid measurements
			if (!(Z >= min_depth)) //Should also be trigger on NaN//std::isnan (Z))
			{
				pt.x = (u - cx) * 1.0 * fx; //FIXME: better solution as to act as at 1meter?
				pt.y = (v - cy) * 1.0 * fy;
				pt.z = std::numeric_limits<float>::quiet_NaN();
			}
			else // Fill in XYZ
			{
				pt.x = (u - cx) * Z * fx;
				pt.y = (v - cy) * Z * fy;
				pt.z = Z;
			}
			// Fill in color
			RGBValue color;
			if(color_idx > 0 && color_idx < rgb_img.total()*color_pix_step){ //Only necessary because of the color_idx offset
				if(pixel_data_size == 3){
					color.Red   = rgb_img.at<uint8_t>(color_idx + red_idx);
					color.Green = rgb_img.at<uint8_t>(color_idx + green_idx);
					color.Blue  = rgb_img.at<uint8_t>(color_idx + blue_idx);
				} else {
					color.Red   = color.Green = color.Blue  = rgb_img.at<uint8_t>(color_idx);
				}
				color.Alpha = 0;
				pt.rgb = color.float_value;
			}
		}
	}

	return cloud;
}

