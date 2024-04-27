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

int 
main (int argc, char** argv)
{
  // Read in the cloud data
  pcl::PCDReader reader;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  reader.read ("../filter.pcd", *cloud);
  std::cout << "PointCloud before filtering has: " << cloud->size () << " data points." << std::endl; //*

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->size ()  << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.01);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::vector<pcl::PointXYZRGB> new_points;
  for(const auto& p : cloud->points) {
    pcl::PointXYZRGB p_rgb;
    p_rgb.x = p.x;
    p_rgb.y = p.y;
    p_rgb.z = p.z;
    // You may also want to set the RGB values
    p_rgb.r = 0; // Red
    p_rgb.g = 0; // Green
    p_rgb.b = 0; // Blue

    new_points.push_back(p_rgb);
  } 
  cloud_output->points.insert(cloud_output->points.end(), new_points.begin(), new_points.end());
 
  uint8_t r;
  uint8_t g;
  uint8_t b;
  int i=0, nr_points = (int) cloud_filtered->size ();

  while (cloud_filtered->size () > 0.4 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;
    // Add the points from the planar component to cloud_output with yellow color
    r = static_cast<uint8_t>(rand() % 256);
    g = static_cast<uint8_t>(rand() % 256);
    b = static_cast<uint8_t>(rand() % 256);
    
    std::cout << "RGB: " << (int)r << " " << (int)g << " " << (int)b << "\n";
    for(const auto& p : cloud_plane->points) {
      pcl::PointXYZRGB p_rgb;
      p_rgb.x = p.x;
      p_rgb.y = p.y;
      p_rgb.z = p.z;
      // Set the RGB values for yellow color
      p_rgb.r = r; // Red
      p_rgb.g = 0; // Green
      p_rgb.b = b; // Blue
      cloud_output->points.push_back(p_rgb);
    }
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud_filtered = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (0);
  ec.setMaxClusterSize (4000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  int j = 0;

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        r = static_cast<uint8_t>(rand() % 256);
        g = static_cast<uint8_t>(rand() % 256);
        b = static_cast<uint8_t>(rand() % 256);
        std::cout << "RGB: " << (int)r << " " << (int)g << " " << (int)b << "\n";
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {
          pcl::PointXYZRGB point;
          point.x = (*cloud_filtered)[*pit].x;
          point.y = (*cloud_filtered)[*pit].y;
          point.z = (*cloud_filtered)[*pit].z;
          point.r =  r;
          point.g =  g;
          point.b =  b;
          cloud_cluster->push_back(point);
        }
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
      if(cloud_cluster->size() >= 100) { 
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
        j++;
      }

        // Merge cloud_cluster into cloud_output
        cloud_output->points.insert(cloud_output->points.end(), cloud_cluster->points.begin(), cloud_cluster->points.end());
  }
  std::cout << "PointCloud representing the PCD: " << cloud_output->size () << " data points." << std::endl;
  cloud_output->width = cloud_output->points.size();
  cloud_output->height = 1;
  writer.write<pcl::PointXYZRGB> ("cloud_clusterxx.pcd", *cloud_output, false);

  return (0);
}