# import numpy as np
# import open3d as o3d

# if __name__ == "__main__":
#     print("Testing IO for point cloud ...")
#     pcd = o3d.io.read_point_cloud("office1.pcd", format='pts')
#     print(pcd)
#     o3d.io.write_point_cloud("copy_of_fragment.ply", pcd)
#     # o3d.io.write_point_cloud("pcd/segmentation/mOSD/learn/learn0.pcd", pcd)

#     print("Load a ply point cloud, print it, and render it")
#     pcd1 = o3d.io.read_point_cloud("copy_of_fragment.ply")
#     print(pcd1)
#     print(np.asarray(pcd1.points))
#     o3d.visualization.draw_geometries([pcd1])

    # print("Downsample the point cloud with a voxel of 0.05")
    # downpcd = pcd1.voxel_down_sample(voxel_size=0.05)
    # o3d.visualization.draw_geometries([downpcd])

    # print("Recompute the normal of the downsampled point cloud")
    # downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(
    #     radius=0.1, max_nn=30))
    # o3d.visualization.draw_geometries([downpcd])

    # print("Print a normal vector of the 0th point")
    # print(downpcd.normals[0])
    # print("Print the normal vectors of the first 10 points")
    # print(np.asarray(downpcd.normals)[:10, :])
    # print("")
# import open3d as o3d

# cloud = o3d.io.read_point_cloud('pcd/segmentation/mOSD/learn/learn1.pcd')
# print(np.asarray(cloud.points))
# print(np.asarray(cloud.colors))
# print(np.asarray(cloud.normals))
# o3d.visualization.draw_geometries([cloud])
import open3d as o3d
import numpy as np


if __name__ == "__main__":
    # Read the point cloud
    pcd = o3d.io.read_point_cloud('office1.pcd')

    # Convert points to numpy array
    points = np.asarray(pcd.points)

    # Create a mask of boolean values representing whether each point is a real number
    mask = ~np.isnan(points).any(axis=1)

    # Use the mask to filter the points
    real_points = points[mask]

    # Translate and scale the points
    real_points = real_points - np.mean(real_points, axis=0)
    real_points = real_points / np.std(real_points, axis=0)

    # Update the point cloud with the new points
    pcd.points = o3d.utility.Vector3dVector(real_points)

    # Visualize the point cloud
    # o3d.visualization.draw_geometries([pcd])

    ######### REMOVE POINTS BELOW Z=0 #########
    points = np.asarray(pcd.points)
    pcd_sel = pcd.select_by_index(np.where(points[:, 2] < 0)[0])

    # visualize different point clouds
    o3d.visualization.draw_geometries([pcd])
    o3d.visualization.draw_geometries([pcd_sel])