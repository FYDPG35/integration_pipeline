#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <opencv2/photo/photo.hpp>

using namespace cv;
using namespace std;

#define RESOLUTION 0.2f
// #define RESOLUTION 1.0f
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
        std::vector<DataPoint> points;
        std::vector<DataPoint> output_points;
        Mat image;
        Mat num_points;
        Mat mask;

    PointCloud(): min_x(99999.0), min_y(99999.0), max_x(0.0), max_y(0.0), min_z(99999.0), max_z(0.0), num_rows(0), num_cols(0) {}

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
        num_rows = (max_x - min_x)/RESOLUTION;
        num_cols = (max_y - min_y)/RESOLUTION;
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
        point.x = min_x + i*RESOLUTION;
        point.y = min_y + j*RESOLUTION;
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
};

int main( int argc, char** argv )
{
    // if( argc != 2)
    // {
    //  cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
    //  return -1;
    // }

    // Mat image;
    // image = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    // if(! image.data )                              // Check for invalid input
    // {
    //     cout <<  "Could not open or find the image" << std::endl ;
    //     return -1;
    // }

    // namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
    // imshow( "Display window", image );                   // Show our image inside it.

    // waitKey(0);
    // Mat mask(image.rows, image.cols, CV_64FC1, cvScalar(0.));
    // cout << "Rows: " << image.rows << " and Column: " << image.cols << endl;
    // for (int i = 0; i < image.cols; i++) {
    //     for (int j = 0; j < image.rows; j++) {
    //         if (i > 80 && i < 110 && j > 30 && j < 70) { //random masking window (would be done through -1)
    //             mask.at<double>(i,j) = 1;
    //         } else {
    //             mask.at<double>(i,j) = 0;
    //         }
    //     }
    // }

    // cout << "Inpainting" <<std::endl;

    // Mat output_image, mask_image;
    // mask.convertTo(mask_image, CV_8UC1); // Converting for inpainting (8-bit 1 channel)
    // inpaint(image, mask_image, output_image, 1, INPAINT_NS); // Could also use INPAINT_TELEA
    // namedWindow( "Display window", WINDOW_AUTOSIZE );  // Create a window for display.
    // imshow( "Display window", output_image );                   // Show our image inside it.
    // waitKey(0);

    PointCloud point_cloud;
    
    ifstream file("/Users/hugolouisseize/filtering/MATLAB_Testing/cameraman.csv");

    if(!file.is_open()) std::cout << "ERROR: File Open" << std::endl;

    while (file.good()) {
        cout << "Reading line" << std::endl;
        std::string x_string;
        getline(file, x_string, ',');
        std::string y_string;
        getline(file, y_string, ',');
        std::string z_string;
        getline(file, z_string);

        double x;
        double y;
        double z;
        std::stringstream x_ss(x_string);
        std::stringstream y_ss(y_string);
        std::stringstream z_ss(z_string);

        if (!(x_ss >> x)) {
            x = 0;
        }

        if (!(y_ss >> y)) {
            y = 0;
        }

        if (!(z_ss >> z)) {
            z = 0;
        }
        cout << "x: " << x << " ";
        cout << "y: " << y << " ";
        cout << "z: " << z << std::endl;

        point_cloud.add(x,y,z);

    }
    point_cloud.generate_image();
    point_cloud.fill_image();
    point_cloud.generate_points();

    ofstream outfile("/Users/hugolouisseize/filtering/MATLAB_Testing/cameraman_out.csv");

    for (auto point:point_cloud.output_points) {
        outfile << point.x << "," << point.y << "," << point.z << "\n";
    }
    return 0;
}
