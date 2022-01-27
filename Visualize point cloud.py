from turtle import down
import numpy as np
import open3d as o3d


def custom_draw_geometry_load_option(pcd):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.get_render_option().load_from_json(
        'renderoption.json'
    )
    vis.run()
    vis.destroy_window()


def generate_mesh_using_bpa(pcd):
    # Generate meshes using bpa method
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 2 * avg_dist
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd,
        o3d.utility.DoubleVector([radius, radius * 2])
    )
    dec_mesh = bpa_mesh.simplify_quadric_decimation(1000000)
    """
    dec_mesh.remove_degenerate_triangles()
    dec_mesh.remove_duplicated_triangles()
    dec_mesh.remove_duplicated_vertices()
    dec_mesh.remove_non_manifold_edges()
    """
    return dec_mesh


if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("./5088.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    # Voxel downsampling
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=1)
    print(downsampled_pcd)
    custom_draw_geometry_load_option(downsampled_pcd)
    # custom_draw_geometry_load_option(pcd)
    # Estimate pcd normals
    downsampled_pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=0.01, max_nn=30)
    )
    mesh = generate_mesh_using_bpa(downsampled_pcd)

    # Generate meshes using poisson method
    """
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=1, width=2, scale=1, linear_fit=False)[0]
    bbox = pcd.get_axis_aligned_bounding_box()
    p_mesh_crop = poisson_mesh.crop(bbox)
    """
    # Coloring meshes
    print("Painting the mesh")
    mesh.paint_uniform_color([1, 0.706, 0])
    """
    poisson_mesh.paint_uniform_color([0.5, 0.7, 0.2])
    p_mesh_crop.paint_uniform_color([0.5, 0.7, 0.2])
    """
    # custom_draw_geometry_load_option(pcd)

    custom_draw_geometry_load_option(mesh)
    # custom_draw_geometry_load_option(poisson_mesh)
