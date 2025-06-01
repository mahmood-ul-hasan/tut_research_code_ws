#data path
target_point_cloud_path=/media/aisl2/aisl_data/code_ws/pairwise_incremental_registration_using_pcl/build_5_varying_boom_Angle_during_cycle/capture00001.pcd;
source_point_cloud_path=/media/aisl2/aisl_data/code_ws/pairwise_incremental_registration_using_pcl/build_5_varying_boom_Angle_during_cycle/capture00002.pcd;
output_point_cloud_path=/media/aisl2/aisl_data/code_ws/pairwise_incremental_registration_using_pcl/build_5_varying_boom_Angle_during_cycle/capture000012.pcd;

#parameters
using_feature=B;              # Feature selection [ B: BSC, F: FPFH, R: RoPS, N: register without feature ]
corres_estimation_method=N;   # Correspondence estimation by [ K: Bipartite graph min weight match using KM, N: Nearest Neighbor, R: Reciprocal NN ]

downsample_resolution=3;    # Raw data downsampling voxel size, just keep one point in the voxel  
neighborhood_radius=5;      # Curvature estimation / feature encoding radius
curvature_non_max_radius=1.0; # Keypoint extraction based on curvature: non max suppression radius 
weight_adjustment_ratio=1.1;  # Weight would be adjusted if the IoU between expected value and calculated value is beyond this value
weight_adjustment_step=0.1;   # Weight adjustment for one iteration
registration_dof=6;           # Degree of freedom of the transformation [ 4: TLS with leveling, 6: arbitary ]
appro_overlap_ratio=0.5;      # Estimated approximate overlapping ratio of two point cloud 

launch_realtime_viewer=1;     # Launch the realtime registration viewer during registration or not (1: Launch, 0: Not launch)

#run
./bin/ghicp ${target_point_cloud_path} ${source_point_cloud_path} ${output_point_cloud_path} \
${using_feature} ${corres_estimation_method} \
${downsample_resolution} ${neighborhood_radius} ${curvature_non_max_radius} \
${weight_adjustment_ratio} ${weight_adjustment_step} ${registration_dof} ${appro_overlap_ratio} ${launch_realtime_viewer}

