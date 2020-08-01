#
# http://mirai-tec.hatenablog.com/entry/2019/08/24/102129
#

#############################################
##      D415 Depth
#############################################
import pyrealsense2 as rs
import numpy as np
import cv2
# import open3d as o3d

TARGET_DISTANCE = 1.0 # meter
WIDTH = 640
HEIGHT = 480
FPS = 30

# Configure depth and color streams
config = rs.config()
config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

# start streaming
pipeline = rs.pipeline()
profile = pipeline.start(config)

# get Depth data
#   distance[m] = depth * depth_scale 
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
distance_min = 10.0 / depth_scale

distance_max = TARGET_DISTANCE / depth_scale
print("Depth Scale = {} -> {}".format(depth_scale, distance_max))

bg_image = np.full((HEIGHT, WIDTH), 0, dtype = np.uint8)
null_image = np.full((HEIGHT, WIDTH, 3), 100, dtype = np.uint8)
cv2.circle(bg_image, (WIDTH /2 , HEIGHT / 2), HEIGHT / 4, 1, thickness = -1, lineType = cv2.LINE_8)
cv2.circle(null_image, (WIDTH /2 , HEIGHT / 2), HEIGHT / 4, (127, 127, 127), thickness = -1, lineType = cv2.LINE_8)

try:
    while True:
        # wait for frame(Depth & Color)
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        # processing before Depth(within 2m)
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = depth_image * bg_image
        min_depth_image = (depth_image < 160.0) * distance_max + depth_image
        minDistance = np.amin(min_depth_image)
        findX = -1
        findY = -1
        if (minDistance < distance_max - 1):
            depth_image = (depth_image < distance_max) * depth_image
            depth_graymap = ((min_depth_image <= (minDistance + 0.05)) * 127. + depth_image * 127. / distance_max) 
            depth_graymap = depth_graymap.reshape((HEIGHT, WIDTH)).astype(np.uint8)
            depth_colormap = cv2.cvtColor(depth_graymap, cv2.COLOR_GRAY2BGR)

            for x in range(0, WIDTH):
                for y in range(0, HEIGHT):
                    if (min_depth_image[y, x] == minDistance):
                        findX = x
                        findY = y
                        depth_colormap[y ,x] = [127, 127, 255]
        else:
            depth_colormap = null_image
            minDistance = -1

        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow("RealSense", cv2.WINDOW_AUTOSIZE)
        cv2.imshow("RealSense", images)
        print("x = ", findX, ", y = ", findY, "dist = ", minDistance)

        if cv2.waitKey(1) & 0xff == 27:
            break
finally:
    # stop streaming
    pipeline.stop()
    cv2.destroyAllWindows()
