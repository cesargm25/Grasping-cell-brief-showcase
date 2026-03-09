ros2 launch yolo_bringup yolo.launch.py \
  model:=yolov8m-seg.pt \
  input_image_topic:=/camera/camera/color/image_raw \
  input_depth_topic:=/camera/camera/depth/image_rect_raw \
  input_depth_info_topic:=/camera/camera/depth/camera_info \
  use_3d:=True \
  target_frame:=camera_depth_optical_frame \
  device:=cpu



