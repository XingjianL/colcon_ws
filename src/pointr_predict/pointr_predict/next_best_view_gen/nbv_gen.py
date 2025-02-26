# import open3d as o3d
# import numpy as np
# import time
# class NBV():
#     def __init__(self, radius = 0.5):
#         self.input_pcd = o3d.geometry.PointCloud()
#         self.pred_pcd = o3d.geometry.PointCloud()
#         self.filtered_pred_pcd = o3d.geometry.PointCloud()
#         self.total_pcd = o3d.geometry.PointCloud()
#         self.pred_pcd_pca = []
#         self.view_cand_radius = radius
#         pass

#     def pred_filter(self):
#         # various filters for the prediction point cloud
#         def dists():
#             # filter for prediction points where the closest point is also a prediction point
#             dist_pi = np.asarray(self.pred_pcd.compute_point_cloud_distance(self.input_pcd))
#             nn_p = np.asarray(self.pred_pcd.compute_nearest_neighbor_distance())+0.001
#             select_idx = np.where(dist_pi - nn_p > 0.001)[0] # meters
#             return dist_pi, nn_p, select_idx
#         def get_outer_points(pcd):
#             # optional: perform concave hull with an alpha parameter for point cloud filtering
#             try:
#                 import pyvista as pv
#             except ImportError:
#                 print("PyVista not found: skipping concave hull filtering")
#                 return pcd
#             # Create a PyVista point cloud
#             cloud = pv.PolyData(np.asarray(pcd.points))
#             # Compute alpha shape (tune alpha for the desired level of detail)
#             alpha = 0.05
#             hull = cloud.delaunay_3d(alpha=alpha)

#             # Extract outer surface
#             outer_pcd = o3d.geometry.PointCloud()
#             outer_pcd.points = o3d.utility.Vector3dVector(np.asarray(hull.extract_geometry().points))
#             return outer_pcd
#         dist_pi, nn_p, select_idx = dists()
#         filtered = self.pred_pcd.select_by_index(select_idx)
#         filtered, _ = filtered.remove_radius_outlier(2,0.015) # number of points in meter radius
#         return filtered.voxel_down_sample(0.001) # voxel with size in meters
#     def pca(self, points3d):
#         points_center = np.mean(points3d,axis=0)
#         shifted_points = points3d-points_center
#         print(shifted_points.shape, shifted_points.dtype)
#         cov_mat = np.cov(shifted_points, rowvar=False)
#         eig_val, eig_vec = np.linalg.eig(cov_mat)
#         add_val, add_vec = self.additional_vectors(eig_val, eig_vec)
#         eig_val = np.hstack([eig_val, add_val])
#         eig_vec = np.hstack([eig_vec, add_vec])
#         ind = np.argsort(eig_val)[::-1]
#         sort_eig_val = eig_val[ind]
#         print(sort_eig_val)
#         sort_eig_vec = eig_vec[:,ind]

#         return points_center, sort_eig_val, sort_eig_vec
#     def additional_vectors(self, eig_val, eig_vectors):
#         vectors = []
#         vectors.append(eig_vectors[:,0]+eig_vectors[:,1]+eig_vectors[:,2])
#         vectors.append(eig_vectors[:,0]-eig_vectors[:,1]+eig_vectors[:,2])
#         vectors.append(eig_vectors[:,0]+eig_vectors[:,1]-eig_vectors[:,2])
#         vectors.append(eig_vectors[:,0]-eig_vectors[:,1]-eig_vectors[:,2])
#         values = [np.mean(eig_val),np.mean(eig_val),np.mean(eig_val),np.mean(eig_val)]
#         return np.array(values), np.array(vectors).T
#     def set_pcds(self, input_pcd, pred_pcd):
#         self.input_pcd = input_pcd
#         self.pred_pcd = pred_pcd
#         self.filtered_pred_pcd = self.pred_filter()
#         self.total_pcd.points = o3d.utility.Vector3dVector(np.vstack((np.array(self.input_pcd.points), np.array(self.filtered_pred_pcd.points))))
#         self.filtered_pred_pcd_pca = self.pca(np.asarray(self.filtered_pred_pcd.points))



#     def hpr_pca_grad_des(self, axis = 0, const_r = False):
#         def hidden_point(points3d: o3d.geometry.PointCloud, viewpoint, radius_multi = 100, only_pred = True):
#             diameter = np.linalg.norm(
#                 np.asarray(points3d.get_max_bound()) - np.asarray(points3d.get_min_bound())
#             )
#             _, ind = points3d.hidden_point_removal(viewpoint, diameter * radius_multi)
#             if not only_pred:
#                 return points3d.select_by_index(np.array(ind))
#             ind = np.array(ind)
#             ind = np.where(ind > len(self.input_pcd.points), ind - len(self.input_pcd.points), -1)
#             hpr_filtered = self.filtered_pred_pcd.select_by_index(ind[ind!=-1])
#             return hpr_filtered
#         def get_false_points(pair):
#             num_union_points = len(pair[0].points) + len(pair[1].points)
#             union = o3d.geometry.PointCloud()
#             union.points = o3d.utility.Vector3dVector(np.vstack((np.asarray(pair[0].points), np.asarray(pair[1].points))))
#             xor_points = len(union.remove_duplicated_points().points)
#             intersection_points = num_union_points - xor_points        
#             fn = len(self.filtered_pred_pcd.points) - xor_points
#             fp = intersection_points
#             #print(num_union_points, xor_points, intersection_points, len(pcd.points))
#             return (fp + fn, fp, fn)
#         def find_gradient(pair_one, pair_two):
#             # pair_one: left view point cloud at steps t1 and t2
#             # pair_one: right view point cloud at steps t1 and t2
#             # return: (difference of all falses, (all falses, fp, fn) at t1)
#             t1_falses = get_false_points([pair_one[0], pair_two[0]])    # initial falses
#             delta_falses = get_false_points([pair_one[1], pair_two[1]]) # next step falses
#             gradient = t1_falses[0] - delta_falses[0]
#             return gradient, t1_falses
#         def optimal_of_axis(axis, const_r = const_r):
#             filter_mean, filter_val, filter_vec = self.filtered_pred_pcd_pca
#             vec_axis = filter_vec[:,axis] / np.linalg.norm(filter_vec[:,axis])
#             distance_multi = 0.36
#             view_point_one = [filter_mean[0]+vec_axis[0]*distance_multi,
#                               filter_mean[1]+vec_axis[1]*distance_multi,
#                               filter_mean[2]+vec_axis[2]*distance_multi]
#             view_point_two = [filter_mean[0]-vec_axis[0]*distance_multi,
#                               filter_mean[1]-vec_axis[1]*distance_multi,
#                               filter_mean[2]-vec_axis[2]*distance_multi]
#             r = [1000, 1000+100]
#             all_falses = []
#             optimal_pair = []
#             all_estimates_one = []
#             all_estimates_two = []
#             step_rate = 25
#             left_bound = 10
#             right_bound = 100000

#             if (const_r):
#                 hpr_filtered_one_pair = hidden_point(self.total_pcd, view_point_one, r[-2])
#                 hpr_filtered_two_pair = hidden_point(self.total_pcd, view_point_two, r[-2])
#                 all_estimates_one.append(len(hpr_filtered_one_pair.points))
#                 all_estimates_two.append(len(hpr_filtered_two_pair.points))
#                 falses = get_false_points([hpr_filtered_one_pair, hpr_filtered_two_pair])
#                 all_falses.append(falses)
#                 return all_estimates_one, all_estimates_two, np.array(all_falses), r, [view_point_one, view_point_two]
#             # gradient descent optimization on R
#             for iteration in range(11,1,-1):
#                 hpr_filtered_one_pair = [hidden_point(self.total_pcd, view_point_one, r[-2]),hidden_point(self.total_pcd, view_point_one, r[-1])]
#                 hpr_filtered_two_pair = [hidden_point(self.total_pcd, view_point_two, r[-2]),hidden_point(self.total_pcd, view_point_two, r[-1])]
#                 all_estimates_one.append(len(hpr_filtered_one_pair[0].points))
#                 all_estimates_two.append(len(hpr_filtered_two_pair[0].points))
#                 gradient, falses = find_gradient(hpr_filtered_one_pair, hpr_filtered_two_pair)
#                 if r[-1] < r[-2]:
#                     gradient *= -1
#                 if abs(gradient) < 1:
#                     gradient += 0.05*r[-1]
#                 all_falses.append(falses)
#                 clipped_gradient = np.clip(iteration * step_rate * np.sign(gradient)*np.log(abs(gradient)), 0.9*(left_bound-r[-1]), 0.9*(right_bound-r[-1]))

#                 r.append(r[-1] + clipped_gradient)
#             return all_estimates_one, all_estimates_two, np.array(all_falses), r, [view_point_one, view_point_two]
#         print("start")
#         start_t = time.time()
#         positive_esti, negative_esti, falses, radii, view_angle = optimal_of_axis(axis)
#         print(time.time()-start_t)
#         optimal_r = radii[np.argmin(falses[:,0])]
        
#         optimal_estimates = [positive_esti[np.argmin(falses[:,0])], negative_esti[np.argmin(falses[:,0])]]
#         print("finished")
#         return {"positive_esti" : positive_esti, 
#                 "negative_esti" : negative_esti, 
#                 "falses" : falses, 
#                 "radii" : radii, 
#                 "view_angles" : view_angle, 
#                 "optimal_estimates" : optimal_estimates,
#                 "visualize_one" : hidden_point(self.total_pcd, view_angle[0], optimal_r).translate(view_angle[0]),
#                 "visualize_two" : hidden_point(self.total_pcd, view_angle[1], optimal_r).translate(view_angle[1])}
#     def generate(self, threaded = False, visualize = False, const_r = False):
#         def visualization(vis_one, vis_two, views, orders):
#             vis = o3d.visualization.Visualizer()
#             vis.create_window()
#             vis.get_render_option().mesh_show_back_face = True
#             vis.get_render_option().point_size = 5.0
#             self.input_pcd.colors = o3d.utility.Vector3dVector(np.array([[1,1,0]]*len(self.input_pcd.points)))
#             vis.add_geometry(self.input_pcd)
#             self.filtered_pred_pcd.colors = o3d.utility.Vector3dVector(np.array([[1,0,1]]*len(self.filtered_pred_pcd.points)))
#             vis.add_geometry(self.filtered_pred_pcd)
#             for i,(v1,v2) in enumerate(zip(vis_one,vis_two)):
#                 v1.colors = o3d.utility.Vector3dVector(np.array([[0,0,0]]*len(v1.points)))
#                 vis.add_geometry(v1)
#                 v2.colors = o3d.utility.Vector3dVector(np.array([[0,0,0]]*len(v2.points)))
#                 vis.add_geometry(v2)
#             for i, o in enumerate(orders):
#                 label = o3d.t.geometry.TriangleMesh.create_text(f"C{i}", depth = 0.1).to_legacy()
#                 print(views.shape, views)
#                 label = label.scale(0.005, np.zeros((3,1)))
#                 label.paint_uniform_color((0,0,1))
#                 label = label.translate(views[o]-np.array([0,0,0.05]))
#                 label = label.rotate(
#                     np.array([[1,0,0],
#                               [0,np.cos(np.pi/3),-np.sin(np.pi/3)],
#                               [0,np.sin(np.pi/3),np.cos(np.pi/3)]]))
#                 vis.add_geometry(label)
#             vis.run()
#             vis.destroy_window()
#             return
#         if threaded:
#             from multiprocessing.pool import ThreadPool
#             pool = ThreadPool(processes=3)
#             tasks = [(0,const_r),(1,const_r),(2,const_r)]
#             results = pool.starmap(self.hpr_pca_grad_des,tasks)
#             all_angles = np.array(results[0]["view_angles"] + results[1]["view_angles"] + results[2]["view_angles"])
#             optimal_order = np.argsort(results[0]["optimal_estimates"]+results[1]["optimal_estimates"]+results[2]["optimal_estimates"])[::-1]

#             return all_angles, optimal_order
        
#         all_angles = []
#         orders = []
#         vis_one = []
#         vis_two = []
#         view_center = np.array([])
#         for i in range(len(self.filtered_pred_pcd_pca[1])):
#             print(f"optimizing ax{i}")
#             ax_results = self.hpr_pca_grad_des(i,const_r)
#             view_center = self.filtered_pred_pcd_pca[0]
#             all_angles.append(ax_results["view_angles"])
#             orders.append(ax_results["optimal_estimates"])
#             if visualize:
#                 vis_one.append(ax_results["visualize_one"])
#                 vis_two.append(ax_results["visualize_two"])
#                 visualization(vis_one, vis_two, np.array(all_angles).reshape(-1,3), np.argsort(np.array(orders).flatten())[::-1])
#         pred_new_points = len(self.filtered_pred_pcd.points)
#         return np.array(all_angles), np.argsort(np.array(orders).flatten())[::-1], view_center, pred_new_points
#         # print("optimizing ax2")
#         # ax2_results = self.hpr_pca_grad_des(1)
#         # print("optimizing ax3")
#         # ax3_results = self.hpr_pca_grad_des(2)

#         all_angles = np.array(ax1_results["view_angles"] + ax2_results["view_angles"] + ax3_results["view_angles"])
#         optimal_order = np.argsort(ax1_results["optimal_estimates"]+ax2_results["optimal_estimates"]+ax3_results["optimal_estimates"])[::-1]

#         return all_angles, optimal_order
    
# if __name__ == "__main__":
#     #select = 1
#     inputpcd = o3d.io.read_point_cloud(f"testpcd/681_Intermediate_0_26_0_12.pcd")
#     pred = o3d.io.read_point_cloud(f"testpcd/fine.pcd")
#     inputpcd.translate([-1.2,1.8,0])
#     nbv = NBV()
#     nbv.set_pcds(inputpcd, pred)
#     import time
#     start = time.time()
#     print(nbv.generate(threaded=False, visualize=True, const_r=False))
#     print(time.time() - start)