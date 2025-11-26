# Task-specific parameters

string task_type             # Type of task (e.g., "object_detection")
uint32 image_width
uint32 image_height
string image_encoding        # e.g., "rgb8", "bgr8", "jpeg"
float32 confidence_threshold
string[] target_classes
uint32 num_points           # For point cloud tasks
float32 voxel_size          # For point cloud filtering
float64 complexity_estimate # Estimated computational complexity (GFLOPS)



