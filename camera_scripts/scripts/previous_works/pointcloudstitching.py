import numpy as np
import cv2
import open3d as o3d
import matplotlib.pyplot as plt
import copy

def find_melon_clusters(frame_buf, color_buf, pos_buf, camint):
	vis = o3d.visualization.Visualizer()
	vis.create_window(window_name='point cloud', width=480, height=848, left=50, top=50,visible=False)
	target_d = frame_buf[:,:,1]
	target_c = color_buf[:,:,30:33]
	target_pos = pos_buf[10][0][2]
	
	gray_img = cv2.cvtColor(target_c, cv2.COLOR_RGB2GRAY)
	ret, thresh = cv2.threshold(gray_img,130,255,cv2.THRESH_BINARY)
	overlay_td = cv2.bitwise_or(target_d, target_d, mask=thresh)
	
	target_depth_img = o3d.geometry.Image(np.array(overlay_td))
	target_color_img = o3d.geometry.Image(np.array(target_c))
	
	rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(target_color_img,target_depth_img, convert_rgb_to_intensity=False)
	target_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, camint)
	total_pcd = copy.deepcopy(target_pcd).translate((0,(target_pos/1000),0))
	
	for i in range(11, len(pos_buf)):
		j = i*3
		k = j+3
		
		source_d = frame_buf[:,:,i]
		source_c = color_buf[:,:,j:k]
		source_pos = pos_buf[i][0][2]
		
		gray_source = cv2.cvtColor(source_c, cv2.COLOR_RGB2GRAY)
		ret, thresh_c = cv2.threshold(gray_source,130,255,cv2.THRESH_BINARY)
		overlay_tds = cv2.bitwise_or(source_d, source_d, mask=thresh_c)
		
		source_depth_img = o3d.geometry.Image(np.array(overlay_tds))
		source_color_img = o3d.geometry.Image(np.array(source_c))
		
		rgbd_image_source =o3d.geometry.RGBDImage.create_from_color_and_depth(source_color_img,source_depth_img, convert_rgb_to_intensity=False)
		
		source_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image_source,camint)
		
		pos_diff = (source_pos-target_pos)/1000
		source_trans = copy.deepcopy(source_pcd).translate((0,pos_diff,0))
		total_pcd = total_pcd + source_trans
	
	
	downpcd = total_pcd.voxel_down_sample(voxel_size=0.01)
	view = copy.deepcopy(downpcd)
	view.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
	
	vis.add_geometry(view)
	vis.poll_events()
	vis.update_renderer()
	points = vis.capture_screen_float_buffer(do_render=False)
	plt.imshow(points)
	plt.axis('off')
	plt.show()
	
	cl, ind = downpcd.remove_statistical_outlier(nb_neighbors=50, std_ratio=0.0001)
	inliercloud = downpcd.select_by_index(ind)
	cluster = copy.deepcopy(inliercloud)
	
	labels = np.array(cluster.cluster_dbscan(eps=0.04, min_points=50,print_progress=True))
	print(labels)
	max_label = labels.max()
	print(f"point cloud has {max_label + 1} clusters")
	colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
	colors[labels < 0] = 0
	cluster.colors = o3d.utility.Vector3dVector(colors[:, :3])
	
	view_cluster = copy.deepcopy(cluster)
	view_cluster.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
	vis.clear_geometries()
	vis.add_geometry(view_cluster)
	vis.poll_events()
	vis.update_renderer()
	points_cluster = vis.capture_screen_float_buffer(do_render=False)
	plt.imshow(points_cluster)
	plt.axis('off')
	plt.show()
	
	segmentlist = []
	
	for num in range (0, max_label+1):
		clusterlist = []
		idx = 0
		for i in labels:
			if i == num:
				clusterlist.append(idx)
			idx = idx + 1
		clusterc = cluster.select_by_index(clusterlist)
		segmentlist.append(clusterc)
		
	centroids = []
	
	for obj in segmentlist:
		mean = obj.compute_mean_and_covariance()
		centroids.append(mean)
	centroids.reverse()
		
	return centroids
