#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/segmentation/min_cut_segmentation.h>

int main ()
{
    pcl::PCDWriter writer;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
    if ( pcl::io::loadPCDFile <pcl::PointXYZRGB> ("../learn34.pcd", *cloud) == -1 )
    {
        std::cout << "Cloud reading failed." << std::endl;
        return (-1);
    }

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::removeNaNFromPointCloud(*cloud, *indices);

    pcl::MinCutSegmentation<pcl::PointXYZRGB> seg;
    seg.setInputCloud (cloud);
    seg.setIndices (indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointXYZRGB point;
    point.x = 68.97;
    point.y = -18.55;
    point.z = 0.57;

    point.r = 255;
    point.g = 255;
    point.b = 255;
    foreground_points->points.push_back(point);
    seg.setForegroundPoints (foreground_points);

    seg.setSigma (0.25);
    seg.setRadius (3.0433856);
    seg.setNumberOfNeighbours (14);
    seg.setSourceWeight (0.8);

    std::vector <pcl::PointIndices> clusters;
    seg.extract (clusters);

    std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
    writer.write<pcl::PointXYZRGB> ("../learn34_seg.pcd", *colored_cloud, false);

    return (0);
}