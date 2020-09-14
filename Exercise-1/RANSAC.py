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
pcl.save(cloud_filtered, 'voxel_downsampled.pcd')


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
pcl.save(cloud_filtered, 'pass_through_filtered.pcd')


#### RANSAC plane segmentation ####

# create segmentation object
seg = cloud_filtered.make_segmenter()

# set the model to fit
seg.set_model_type(pcl.SACMODEL_PLANE)
seg.set_method_type(pcl.SAC_RANSAC)

# max distance for segmenting the table
max_distance = 0.01
seg.set_distance_threshold(max_distance)

# call segment function to obtain set of inliner indices and model coefficients
inliners, coefficients = seg.segment()


#### Extract inliers ####
extracted_inliners = cloud_filtered.extract(inliners, negative=False)
pcl.save(extracted_inliners, 'extracted_inliners.pcd')

#### Extract outliers ####
extracted_inliners = cloud_filtered.extract(inliners, negative=True)
pcl.save(extracted_inliners, 'extracted_outliers.pcd')

# Save pcd for table
# pcl.save(cloud, filename)


# Extract outliers


# Save pcd for tabletop objects


