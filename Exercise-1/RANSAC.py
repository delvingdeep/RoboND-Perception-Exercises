# Import PCL module
import pcl

# Load Point Cloud file
cloud = pcl.load_XYZRGB('tabletop.pcd')


#### Voxel Grid filter ####

# create a VoxelGrid filter object
vox = cloud.make_voxel_grid_filter()

# choose voxel leaf size
LEAF_SIZE = 0.01

# set voxel size
vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)

# use filter function to downsample point cloud
cloud_filtered = vox.filter()
filename = 'voxel_downsampled.pcd'
pcl.save(cloud_filtered, filename)


#### PassThrough filter ####

# create PassThrough filter object
passthrough = cloud_filtered.make_passthrough_filter()

# assign axis and range to the passthrough filter object 
filter_axis = 'z'
passthrough.set_filter_field_name(filter_axis)
axis_min = 0.5
axis_max = 1.5
passthrough.set_filter_limits(axis_min, axis_max)

# use filter function to obtain the resultant poin cloud
cloud_filtered = passthrough.filter()
filename2 = 'pass_through_filtered.pcd'
pcl.save(cloud_filtered, filename2)

# RANSAC plane segmentation


# Extract inliers

# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers


# Save pcd for tabletop objects


