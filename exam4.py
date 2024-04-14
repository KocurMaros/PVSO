import open3d as o3d
from pathlib import Path

pcs_path = Path("point_clouds")
pcd = o3d.io.read_point_cloud(str(pcs_path.joinpath("01.pcd")))

vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(pcd)

for p in pcs_path.iterdir():
    pcd = o3d.io.read_point_cloud(str(p))
    
    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()

vis.destroy_window()
