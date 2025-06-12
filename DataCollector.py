"""
Helpers to collect data while processing a slam frame sequence

"""

"""
Columns of data: frame_idx, kp_idx, kp_uuv, kp_response, kp_p_reproj_err

At /pyslam/main_slam.py line 318 in the 'compute metrics' section
slam.map.frames has all the frames

for frame in slam.map.frames:
    for kp in frame:

for point in map.points
    for view in point.views
        kp, err = point_reprojection_err()

Steps
    Create memmap array
    Process frame and collect data
    flush frame data to file


"""