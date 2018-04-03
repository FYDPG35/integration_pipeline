#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/photo/photo.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/filters/bilateral.h>

using namespace cv;
using namespace std;

typedef pcl::PointXYZI PointT;


#define NUM_COLS 5000.0f

#define ASCII 0
#define BINARY 1
#define BINARY_COMPRESSED 2

struct DataPoint {
    double x;
    double y;
    double z;
};

class PointCloud{
    private:

    public:
        double min_x;
        double min_y;
        double max_x;
        double max_y;
        double min_z;
        double max_z;
        int num_rows;
        int num_cols;
        double resolution;
        std::vector<DataPoint> points;
        std::vector<DataPoint> output_points;
        Mat image;
        Mat num_points;
        Mat mask;

    PointCloud(): min_x(9999.0), min_y(99999.0), max_x(0.0), max_y(0.0), min_z(99999.0), max_z(0.0), num_rows(0), num_cols(0) {}

    void add(double x, double y, double z) {
        DataPoint point = {.x = x, .y = y, .z = z};
        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(min_y, y);
        min_z = std::min(min_z, z);
        max_z = std::max(max_z, z);
        // cout << "Minimum x: " << min_x << std::endl;
        // cout << "Minimum y: " << min_y << std::endl;
        // cout << "Maximum x: " << max_x << std::endl;
        // cout << "Maximum y: " << max_y << std::endl;
        points.push_back(point);
    }

    void point_to_image(DataPoint &point) {
        int i = (point.x - min_x)/(max_x - min_x)*num_rows;
        int j = (point.y - min_y)/(max_y - min_y)*num_cols;
        int num = num_points.at<double>(i,j);
        double point_intensity = 255*(point.z - min_z)/(max_z - min_z); //mapping to an 8 bit intensity
        double intensity = (point_intensity + image.at<double>(i,j)*num)/(num+1); // Take average        
        image.at<double>(i,j) = intensity; // New intensity
        num_points.at<double>(i,j) = num + 1; // increasing count by 1
        if (i != num_rows && j != num_cols) {
            mask.at<double>(i,j) = 0.0f; // Not creating a mask there

        }
    }

    void generate_image( void ) {
        resolution = (max_x - min_x)/NUM_COLS;
        num_rows = (max_x - min_x)/resolution;
        num_cols = (max_y - min_y)/resolution;
        std::cout << "number of rows: " << num_rows << std::endl;
        std::cout << "number of cols: " << num_cols << std::endl;
        Mat data(num_rows, num_cols, CV_64FC1, cvScalar(0.));
        data.copyTo(image); // Create image Mat object
        data.copyTo(num_points); // Keep track of number of entries
        Mat data_mask(num_rows, num_cols, CV_64FC1, cvScalar(1.));
        data_mask.copyTo(mask); // Create mask Mat object

        for (int i=0; i < points.size(); i++) {
            point_to_image(points[i]);
        }
    }

    void fill_image( void ) {
        cout << "Converting" << std::endl;
        Mat output_image, mask_8UC1, image_8UC1;
        image.convertTo(image_8UC1, CV_8UC1); // Converting for inpainting (8-bit 1 channel)
        mask.convertTo(mask_8UC1, CV_8UC1); // Converting for inpainting (8-bit 1 channel)
        cout << "Inpainting" << std::endl;
        inpaint(image_8UC1, mask_8UC1, output_image, 1, INPAINT_NS); // Could also use INPAINT_TELEA
        cout << "Reconverting" << std::endl;
        output_image.convertTo(image, CV_64FC1);
        cout << "Done Inpainting" << std::endl;

    }

    void image_to_point( DataPoint &point, int i, int j) {
        point.x = min_x + i*resolution;
        point.y = min_y + j*resolution;
        point.z = image.at<double>(i,j)*(max_z - min_z)/255.0;
    }
    void generate_points( void ) {
        for (int i = 0; i < image.rows; i++) {
            for (int j = 0; j < image.cols; j++) {
                DataPoint point;
                image_to_point(point, i, j);
                output_points.push_back(point);
                cout << "Point (" << point.x << ", " << point.y << ", " << point.z << ")\n";
            }
        }
    }

    void filter( void ) {
        Mat filtered_image;

        // Filter
	image.copyTo(filtered_image);
        

        // Copy the filtered image back
        filtered_image.copyTo(image);
    }
};


int
  main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr incloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::string file_path = "/home/ubuntu/filtering/files/";
  //std::string cloud_file = file_path + "table_scene_lms400.pcd";
  std::string cloud_file = file_path + "construction_site.ply";
  if (pcl::io::loadPLYFile<pcl::PointXYZ> (cloud_file, *incloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Initial Point Cloud" << std::endl;
  std::cout << *incloud << std::endl;

  // Remove NaNs
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*incloud, *cloud, indices);
  
  // Remove Outliers
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50); // Number of Neighbors
  sor.setStddevMulThresh (1.0); // Standard Deviation Multiplier
  sor.filter(*outcloud);
  //sor.filter (*cloud_inliers);

  std::cout << "After Removing Outliers: " << std::endl;
  std::cout << *outcloud << std::endl;
  //std::cout << *cloud_inliers << std::endl;

  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers (new pcl::PointCloud<pcl::PointXYZ>);
  
  /**
  PointCloud point_cloud;

  std::cout << "Adding Point CLoud Info" << std::endl;

  for (size_t i = 0; i < cloud_inliers->points.size(); i++) {
      point_cloud.add(cloud_inliers->points[i].x, cloud_inliers->points[i].y, cloud_inliers->points[i].z);
  }
  std::cout << "Minimum x: " << point_cloud.min_x << std::endl;
  std::cout << "Maximum x: " << point_cloud.max_x << std::endl;
  std::cout << "Minimum y: " << point_cloud.min_y << std::endl;
  std::cout << "Maximum y: " << point_cloud.max_y << std::endl;

  point_cloud.generate_image();
  
  point_cloud.fill_image();

  point_cloud.filter();

  point_cloud.generate_points();


  pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud (new pcl::PointCloud<pcl::PointXYZ>);
  outcloud->resize(point_cloud.output_points.size());
  for (size_t i = 0; i < point_cloud.output_points.size(); i++) {
      outcloud->points[i].x = point_cloud.output_points[i].x;
      outcloud->points[i].y = point_cloud.output_points[i].y;
      outcloud->points[i].z = point_cloud.output_points[i].z;
  }
 **/
  // Normal estimation*
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (outcloud);
  n.setInputCloud (outcloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*outcloud, *normals, *cloud_with_normals);
  //* cloud_with_normals = cloud + normals

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  //gp3.setSearchRadius (0.025);
  gp3.setSearchRadius (50);
  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (500);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // Additional vertex information
  std::vector<int> parts = gp3.getPartIDs();
  std::vector<int> states = gp3.getPointStates();

  // Save Mesh
  pcl::io::savePolygonFileSTL("/home/ubuntu/filtering/files/construction_site.stl", triangles, BINARY);

  // Apply Bilateral Filter
  /*
  pcl::PointCloud<PointT>::Ptr cloud_bilateral (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered_xyzi (new pcl::PointCloud<PointT>);

  cloud_bilateral->points.resize(cloud_inliers->size());
  for (size_t i = 0; i < cloud_inliers->points.size(); i++) {
      cloud_bilateral->points[i].x = cloud_inliers->points[i].x;
      cloud_bilateral->points[i].y = cloud_inliers->points[i].y;
      cloud_bilateral->points[i].z = cloud_inliers->points[i].z;
      cloud_bilateral->points[i].intensity = cloud_inliers->points[i].z;
  }
  //pcl::copyPointCloud(cloud_inliers, cloud_bilateral);
  pcl::KdTreeFLANN<PointT>::Ptr tree (new pcl::KdTreeFLANN<PointT>);

  float sigma_s = 1, sigma_r = 1;
  pcl::BilateralFilter<PointT> bf;
  bf.setInputCloud (cloud_bilateral);
  //bf.setSearchMethod (tree);
  bf.setHalfSize (sigma_s);
  bf.setStdDev (sigma_r);
  bf.applyFilter (*cloud_filtered_xyzi);
  
  // Transfer the data back
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  cloud_filtered->points.resize(cloud_filtered_xyzi->size());
  for (size_t i = 0; i < cloud_filtered_xyzi->points.size(); i++) {
      cloud_filtered->points[i].x = cloud_filtered_xyzi->points[i].x;
      cloud_filtered->points[i].y = cloud_filtered_xyzi->points[i].y;
      cloud_filtered->points[i].z = cloud_filtered_xyzi->points[i].intensity;
  }

  std::cout << "After Bilateral Filtering:" << std::endl;
  std::cout << *cloud_filtered << std::endl;

  // Write the output to a file
  //pcl::PCDWriter writer;
  //std::string filtered_file = file_path + "table_scene_lms400_filtered.pcd";
  //writer.write<pcl::PointXYZ> (filtered_file, *cloud_bilateral, false);
  */
  return (0);
}
