"""
Helpers to collect data while processing a slam frame sequence

"""

"""

At /pyslam/main_slam.py line 318 in the 'compute metrics' section
slam.map.frames deque is limited to 20 frames

Steps
    Create memmap array (this is good for processing but nor for appending)
    Use a list instead
    Process frame and collect data
    flush frame data to file


"""
import numpy as np
if __name__ == "__main__":
    # just to setup the environment for testing
    import main_slam
from frame import Frame, FeatureTrackerShared, FrameBase
import cv2

import traceback

# Added data elements
# Frame.__init__:456: kps_data = np.array([ [x.pt[0], x.pt[1], x.octave, x.size, x.angle, x.response] for x in self.kps ], dtype=np.float32)                     
# Frame.__init__:461: self.response  = kps_data[:,5]  

# instantiate this class on the map, so it is accessible everywhere
# Map.__init__:92: self.data_collector = DataCollector()
# Tracking.track:1067: self.map.data_collector.collect_frame(self.f_cur)
# Main_slam:336: slam.map.data_collector.save()

class DataCollector():
    def __init__(self):
        
        # collect all in the end did not work, map.frames is a window (deque) with only 20 frames
        #self.map = map
        #self.data = np.array( [ [f.id, f.points[kpi].id, kpi, kpsu[0], kpsu[1], f.response[kpi], self.p_to_kp_reprojection_err(kpi, f)] \
        #                        for f in map.frames    if f.id > 2 \
        #                            for (kpi, kpsu) in enumerate(f.kpsu)   if f.points[kpi]], \
        #                    dtype=np.float32)
        
        self.cols_data = ["frame_id", "point_id", "kp_idx", "kpu_u", "kpu_v", "kp_response", "kp_p_reproj_err", "kp_des_angle", "kp_octave", "kp_des"] # keep kp_des at the end
        self.data = []

    
    # collect data on every tracked frame
    # first two frames are not valid, place collect_frame after initialization 
    def collect_frame(self, f:Frame, img):
        '''
        Collect data from frame
        '''
        # index can be a float because an int is represented exactly up to the mantissa 2^23 for float32
        # frame_id is unique and kp_idx is unique within a frame
        # take only kp that have a world point which not is_bad (has at least 2 observations)
        # the patch has to be centered on the kps (coordinates returned by cv2, see frame.py L456)
        self.data.extend([ [ f.id, f.points[kpi].id, kpi, kpsu[0], kpsu[1], f.response[kpi], \
                             self.p_to_kp_reprojection_err(kpi, f), f.angles[kpi], f.octaves[kpi], \
                             self.get_patch(f.kps[kpi], f.sizes[kpi], 0, img),  f.des[kpi] ] \
                                for (kpi, kpsu) in enumerate(f.kpsu)   if (f.points[kpi] and not f.points[kpi].is_bad) ]) 
                            
                            #above is an experiment with no rotation, below is the original line for reference
                            #self.get_patch(f.kps[kpi], f.sizes[kpi], f.angles[kpi], img),  f.des[kpi] ] \
                            
                             
    def get_patch(self, kpuv, size, angle, img):
        '''
        get 16x16 gray patch from image at keypoint location
        descale the size to 31 and rotate 
        kp:[u,v] center of patch , size:float , angle:float , img:array[]  
        recortar um pedaço maior, converter para cinza, rotacionar, reduzir à oitava, recortar 16x16

        '''
        # check if kpuv has margin to rotate and extract a clean patch, this excludes point too close to the border
        margin = np.ceil(size*np.sqrt(2))
        room_for_patch_rotation = (kpuv[0] > margin) and (kpuv[1] > margin) and (img.shape[0]-kpuv[0]) > margin and (img.shape[1]-kpuv[1]) > margin
        size_out = 16
        if room_for_patch_rotation:
            kp_base_size = 31
            patch_center = int(size)
            patch = img[int(kpuv[0]-patch_center):int(kpuv[0]+patch_center), int(kpuv[1]-patch_center):int(kpuv[1]+patch_center)]
            if len(patch.shape) > 2:
                if patch.shape[2] > 1:
                    # if patch has more than one color layer
                    patch = cv2.cvtColor(patch, cv2.COLOR_RGB2GRAY)
            # rotate around center of patch and scale to the kp_base_size
            rot_m = cv2.getRotationMatrix2D((patch_center,patch_center), angle, kp_base_size/size)
            patch = cv2.warpAffine(patch, rot_m, patch.shape, flags=cv2.INTER_NEAREST)
            min, max = int(patch_center-size_out/2), int(patch_center+size_out/2)
            patch = patch[min:max, min:max]
        else:
            patch = np.zeros((size_out, size_out))
        return patch

    # inspired on map.remove_points_with_big_reproj_err() 
    def p_to_kp_reprojection_err(self, kpidx:int, frame:Frame) -> float:
        '''
        Calculate the reprojection chi2 of the point kpidx in frame
        '''
        uv = frame.kpsu[kpidx]
        proj,z = frame.project_map_point(frame.points[kpidx])
        invSigma2 = FeatureTrackerShared.feature_manager.inv_level_sigmas2[frame.octaves[kpidx]]
        err = (proj-uv)
        chi2 = np.inner(err,err)*invSigma2
        return chi2

    
    # save the data collected in the end
    def save(self):
        '''
        Save collected data on local folder
        '''
        # save as np array not to import pandas
        # convert list to np.array just now because np is inneficient to append data
        try:
            des_col = -1  # to keep descriptors in the last column separate because of the shape
            img_col = -2
            data_without_descriptors = np.array([row[:img_col] for row in self.data], dtype=np.float32)  # all columns except descriptors
            descriptors = np.array([row[des_col] for row in self.data], dtype=np.uint8)  # just the descriptors
            patches = np.array([row[img_col] for row in self.data], dtype=np.uint8)  # just the patches
            cols_data_without_descriptors = np.array(self.cols_data[:img_col], dtype=str)
            filename = "data_collected.npz"
            np.savez(filename, cols_data_without_descriptors=cols_data_without_descriptors, data_without_descriptors=data_without_descriptors, 
                     descriptors=descriptors, patches=patches )
            print("DataCollector: Saved", filename, "with", len(self.data), "key points")
        except Exception as e:
            print('Exception while saving DataCollector: ', e)
            print(f'traceback: {traceback.format_exc()}')

if __name__  == "__main__":
    
    # test the implementation of get_patch()
    import matplotlib.pyplot as plt
    import matplotlib as mpl
    img_size = 200
    img_fill_start = int(img_size/4)
    img_fill_end = img_size-img_fill_start
    img = np.zeros((img_size, img_size), dtype=np.uint8)  # type needs to be uint8
    halftone = 127
    bright = 255
    img.fill(halftone)
    step_size = 6 # aplox diagonal 8*sqrt(2) 
    steps = 3
    tone_step_size = (bright - halftone) / steps
    for step in range(steps):
        img[img_fill_start+step*step_size:img_fill_end,img_fill_start+step*step_size:img_fill_end].fill(bright-step*tone_step_size)
    orb = cv2.ORB_create()
    kps, des = cv2.ORB.detectAndCompute(orb, img, None)
    green = (0, 255, 0)
    img_with_keypoints = cv2.drawKeypoints(img, kps, None, color=green, flags=cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    fig, ax = plt.subplots()
    ax.imshow(img_with_keypoints)
    cv2.imshow('img_with_keypoints', img_with_keypoints)
    #cv2.waitKey(0)
    
    dc = DataCollector()
    patches= np.empty((16, 0), dtype='uint8')
    for i, kp in enumerate(kps):
        patches = np.append(patches, dc.get_patch(kp.pt, kp.size, kp.angle, img), axis=1)
    fig, ax = plt.subplots()
    ax.imshow(patches)
    cv2.imshow('patches',patches)
    fig, ax = plt.subplots()
    mpl.widgets.TextBox(ax, "Info:\nHit a key to close the cv2 window and finish the program")
    cv2.waitKey(0)
    
    cv2.destroyAllWindows()

#    orb = cv2.ORB_create()
#    kps, des = cv2.ORB.detectAndCompute(orb, img, None)
#
#    plt.imshow(img)
#    plt.show()
#
#    cv2.imshow('img',img)
#    cv2.waitKey(0)
#    cv2.destroyAllWindows()
    