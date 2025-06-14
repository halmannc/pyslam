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
from frame import Frame, FeatureTrackerShared, FrameBase

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
        
        self.col_labels = ["frame_id", "point_id" "kp_idx", "kpu_u", "kpu_v", "kp_response", "kp_p_reproj_err"]
        self.data = []

    
    # collect data on every tracked frame
    # first two frames are not valid, place collect_frame after initialization 
    def collect_frame(self, f:Frame):
        '''
        Collect data from frame
        '''
        # index can be a float because an int is represented exactly up to the mantissa 2^23 for float32
        # frame_id is unique and kp_idx is unique within a frame
        # take only kp that have a world point
        self.data.extend([ [ f.id, f.points[kpi].id, kpi, kpsu[0], kpsu[1], f.response[kpi], self.p_to_kp_reprojection_err(kpi, f) ] \
                                for (kpi, kpsu) in enumerate(f.kpsu)   if f.points[kpi] ])
    
    
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
        data = np.array(self.data, dtype=np.float32)
        cols = np.array(self.col_labels, dtype=str)
        np.savez("data_collected.npz", cols=cols, data=data )
    