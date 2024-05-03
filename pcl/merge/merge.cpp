#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/point_types.h>
#include <pcl/common/io.h> // for concatenateFields

// You might need to replace "pcl::PointXYZ" with your point cloud data type

int main() {
  // Load your point clouds

    pcl::PointCloud<pcl::PointXYZ>::Ptr big_cloud (new pcl::PointCloud<pcl::PointXYZ>), small_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader;
    reader.read ("../learn34.pcd", *big_cloud);
    std::cout << "PointCloud before filtering has: " << big_cloud->size () << " data points." << std::endl; 

    reader.read ("../cloud_cluster_0.pcd", *small_cloud);
    std::cout << "PointCloud before filtering has: " << small_cloud->size () << " data points." << std::endl; 
    // Load data into big_cloud and small_cloud

    // Create a KDTree for efficient neighbor search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(small_cloud);

    // Define a search radius (adjust based on your point density)
    double search_radius = 0.1;

    // Iterate through each point in the big cloud
    std::vector<int> point_indices;
    std::vector<float> point_distances;

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    for (size_t i = 0; i < big_cloud->points.size(); ++i) {
        const pcl::PointXYZ& point = big_cloud->points[i];

        // Search for neighboring points in the small cloud
        kdtree.nearestKSearch(point, 1, point_indices, point_distances);

        // Check if there are any neighbors within the search radius
        if (point_distances[0] <= search_radius) {
        // Point is between points in the small cloud, add it to the filtered cloud
        filtered_cloud->push_back(point);
        }

        // Clear search results for the next iteration
        point_indices.clear();
        point_distances.clear();
    }

  // filtered_cloud now contains all points from big_cloud that are between points in small_cloud
    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("clustered.pcd", *filtered_cloud, false);
  // Use filtered_cloud for further processing or visualization
  // ...

  return 0;
}