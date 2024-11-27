#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import Image,CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge,CvBridgeError
import cv2
import numpy as np
from simple_pid import PID
import time




class CameraNode(Node):
    def __init__(self):
        super().__init__("camera_node")
        
        self.qos_profile=QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.subscriber_ = self.create_subscription(CompressedImage, 
                                                    "/zed_cam/camera_sensor/right/image_raw/compressed",
                                                    self.callback_camera, 
                                                    qos_profile=self.qos_profile

                                                    )
        
        self.publisher_ = self.create_publisher(Twist, "/autocar/cmd_vel",10)
        self.msg_velocity=Twist()
        
        self.vectorize_int=np.vectorize(int)
        

        
        self.source_points= np.float32([(0, 337), (282, 209), (383, 209), (672, 337)])
        self.destination_points= np.float32([(250, 376), (250, 0), (422, 0), (422, 376)])
        
        self.thd_highlight_L=                  150
        self.thd_highlight_S=                  30
        self.thd_shadow_L=                     30
        self.thd_shadow_S=                     50
        self.thd_S_mag=                        25
        self.thd_S_arg=                        0
        self.thd_S_x=                          0
        self.thd_L_mag=                        20
        self.thd_L_arg=                        100
        self.thd_L_y=                          100
        
        self.thd_highlight_L=                  150
        self.thd_highlight_S=                  30
        self.thd_shadow_L=                     30
        self.thd_shadow_S=                     50
        self.thd_S_mag=                        10
        self.thd_S_arg=                        25
        self.thd_S_x=                          0
        self.thd_L_mag=                        20
        self.thd_L_arg=                        0
        self.thd_L_y=                          75
        
        self.num_of_windows=                    15
        self.histogram_width=                  75
        self.histogram_seed=                  64
        self.histogram_vertical_ratio_end=     1
        self.histogram_vertical_ratio_start=   0
        self.histogram_ratio_localmax=         1
        self.offset_cam=                      0
        self.m_vehicle_width=                  1.8
        self.m_look_ahead=                     10
        self.margin_x=                         50
        self.min_pixel_inside=                 50
        self.max_pixel_inside=              4500
        self.max_width_not_a_line=             115
        self.min_pixel_confindex=              50
        self.xm_by_pixel=                      3.7
        self.ym_by_pixel=                     30/720
        self.thd_confindex=                    33
        self.min_pixel_bold=                   150
        self.min_pixel_doubleline=             150
        self.doubleline_width_px=              50
        self.bold_width_px=                    75
        
        self.lanechange                             = None
        
        self.coeff_L                                = [0, 0, 0, 0]
        self.coeff_R                                = [0, 0, 0, 0]     
        self.confindex_L                            = None
        self.confindex_R                            = None
        self.linetype_L                             = None
        self.linetype_R                             = None                
        self.coeff_next_L                           = [0, 0, 0, 0]
        self.coeff_next_R                           = [0, 0, 0, 0]           
        self.confindex_next_L                       = None
        self.confindex_next_R                       = None  
        self.linetype_next_L                        = None
        self.linetype_next_R                        = None

        self.previous_coeff_L                       = [0, 0, 0, 0]
        self.previous_coeff_R                       = [0, 0, 0, 0]
        self.previous_confindex_L                   = None
        self.previous_confindex_R                   = None
        self.previous_linetype_L                    = None
        self.previous_linetype_R                    = None
        self.previous_coeff_next_L                  = [0, 0, 0, 0]
        self.previous_coeff_next_R                  = [0, 0, 0, 0]
        self.previous_confindex_next_L              = None
        self.previous_confindex_next_R              = None
        self.previous_linetype_next_L               = None
        self.previous_linetype_next_R               = None    

        self.birdeye_matrix        = cv2.getPerspectiveTransform(self.source_points, self.destination_points)
        self.inv_birdeye_matrix    = cv2.getPerspectiveTransform(self.destination_points, self.source_points)
        
        self.points_ZoI = np.float32([(252, 204), (0, 209), (0, 376), (672, 376), (672, 209), (420, 204)])
    
        self.PID=PID(Kp=1.5,
                     Ki=0,
                     Kd=-0.09)
        
        self.bridge = CvBridge()
        
        self.pTime=0
                
        self.get_logger().info("Camera has started.")

    def callback_camera(self, msg):
        
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        except CvBridgeError as e:
            print(e)
            
        
            
        frame_HLS=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HLS)
        frame_gray=cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)
            
        ### FRAME HLS BALANCE EXPOSURE
        
        exposure_correction_ratio = 1
        frame_L                 = frame_HLS[:,:,1]
        mean_L                  = np.mean(frame_L)
        max_L                   = np.max(frame_L)
        min_L                   = np.min(frame_L)
        if ~(max_L == min_L):
            frame_L             = frame_L - np.abs((frame_L - mean_L)/(max_L - min_L))*(frame_L - mean_L)*exposure_correction_ratio
        frame_HLS[:,:,1]        = np.uint8(frame_L)
        
        ### FRAME HLS BALANCE WHITE
        ratio_S = 1
        ratio_L = 1
        """apply white balance correction based on a reference pixel""" 
        frame_S                 = frame_HLS[:,:,2]
        frame_L_negative        = 255 - frame_HLS[:,:,1]    
        frame_S_Ln              = ratio_S*frame_S + ratio_L*frame_L_negative

        mask_min_S_Ln           = np.zeros_like(frame_HLS[:,:,2], dtype=bool)
        mask_max_Ln             = np.zeros_like(frame_HLS[:,:,1], dtype=bool)

        mask_min_S_Ln[(frame_S_Ln == np.min(frame_S_Ln))]                                = True
        mask_max_Ln[frame_L_negative == np.min(frame_L_negative[mask_min_S_Ln])]         = True

        min_S                   = frame_S[mask_min_S_Ln & mask_max_Ln][0]

        frame_S                 = frame_S - min_S
        frame_S[frame_S < 0]    = 0
        
        frame_HLS[:,:,2]        = frame_S

        ### APPLY SOBEL
        
        sobel_x     = cv2.Sobel(frame_HLS[:,:,1], cv2.CV_64F, 1, 0, 9) #frame sobelx
        sobel_y     = cv2.Sobel(frame_HLS[:,:,1], cv2.CV_64F, 0, 1, 9) #frame sobely
        sobel_mag   = np.sqrt(sobel_x**2 + sobel_y**2)
        sobel_arg   = (np.abs(np.arctan2(sobel_y, sobel_x)) - np.pi/2)*180/np.pi
        
        length      = np.max(np.abs(sobel_x))    
        if length > 0:
            sobel_L_x  = np.uint8(127*(1 + sobel_x/length))
        else:
            sobel_L_x  = np.zeros_like(sobel_x, dtype = np.uint8)
            
        length      = np.max(np.abs(sobel_y))    
        if length > 0:
            sobel_L_y  = np.uint8(127*(1 + sobel_y/length))
        else:
            sobel_L_y  = np.zeros_like(sobel_y, dtype = np.uint8)
            
        length      = np.max(np.abs(sobel_mag))    
        if length > 0:
            sobel_L_mag  = np.uint8(127*(1 + sobel_mag/length))
        else:
            sobel_L_mag  = np.zeros_like(sobel_mag, dtype = np.uint8)
            
        length      = np.max(np.abs(sobel_arg))    
        if length > 0:
            sobel_L_arg  = np.uint8(127*(1 + sobel_arg/length))
        else:
            sobel_L_arg  = np.zeros_like(sobel_arg, dtype = np.uint8)
            
        sobel_x     = cv2.Sobel(frame_HLS[:,:,2], cv2.CV_64F, 1, 0, 9) #frame sobelx
        sobel_y     = cv2.Sobel(frame_HLS[:,:,2], cv2.CV_64F, 0, 1, 9) #frame sobely
        sobel_mag   = np.sqrt(sobel_x**2 + sobel_y**2)
        sobel_arg   = (np.abs(np.arctan2(sobel_y, sobel_x)) - np.pi/2)*180/np.pi
        
        length      = np.max(np.abs(sobel_x))    
        if length > 0:
            sobel_S_x  = np.uint8(127*(1 + sobel_x/length))
        else:
            sobel_S_x  = np.zeros_like(sobel_x, dtype = np.uint8)
            
        length      = np.max(np.abs(sobel_y))    
        if length > 0:
            sobel_S_y  = np.uint8(127*(1 + sobel_y/length))
        else:
            sobel_S_y  = np.zeros_like(sobel_y, dtype = np.uint8)
            
        length      = np.max(np.abs(sobel_mag))    
        if length > 0:
            sobel_S_mag  = np.uint8(127*(1 + sobel_mag/length))
        else:
            sobel_S_mag  = np.zeros_like(sobel_mag, dtype = np.uint8)
            
        length      = np.max(np.abs(sobel_arg))    
        if length > 0:
            sobel_S_arg  = np.uint8(127*(1 + sobel_arg/length))
        else:
            sobel_S_arg  = np.zeros_like(sobel_arg, dtype = np.uint8)
            
        ### APPLY SOBEL MASK
        
        L_mag_cond                              = np.abs(127 - sobel_L_mag.astype(int)) > self.thd_L_mag
        S_mag_cond                              = np.abs(127 - sobel_S_mag.astype(int)) > self.thd_S_mag        
        LS_mag_cond                             = L_mag_cond | S_mag_cond
        self.mask_LS_mag                        = np.zeros_like(LS_mag_cond, dtype=np.uint8)
        self.mask_LS_mag[LS_mag_cond]           = 1
        
        """we keep pixel with sobel_S_arg far from scaled-127 = unscaled-90deg"""       
        Larg_cond                               = np.abs(127 - sobel_L_arg.astype(int)) > self.thd_L_arg
        Sarg_cond                               = np.abs(127 - sobel_L_arg.astype(int)) > self.thd_S_arg
        LS_arg_cond                             = Larg_cond & Sarg_cond
        self.mask_LS_Larg                       = np.zeros_like(LS_arg_cond, dtype=np.uint8)
        self.mask_LS_Larg[LS_arg_cond]          = 1
        
        """we remove pixels with high Ly """       
        Ly_cond                                 = np.abs(127 - sobel_L_y.astype(int)) < self.thd_L_y
        self.mask_Ly                            = np.zeros_like(LS_arg_cond, dtype=np.uint8)
        self.mask_Ly[Ly_cond]                   = 1          
        
        
        ### CREATE FILTER MASK
        
        mask_note = self.mask_LS_mag + self.mask_LS_Larg + self.mask_Ly
        frame_HLS[~(mask_note >= 3)] = [0, 0, 0]
        
        ### APPLY HIGHLIGHT REMOVE

        L_cond = frame_HLS[:,:,1] > self.thd_highlight_L
        S_cond = frame_HLS[:,:,2] < self.thd_shadow_S        
        """we wont keep pixel with low saturation and high lightness"""
        LS_cond                        = (L_cond & S_cond)       
        frame_HLS[LS_cond,:]      = [0, 0, 0]
        
        ### APPLY SHADOW REMOVE
        
        L_cond = frame_HLS[:,:,1]  < self.thd_shadow_L
        S_cond = frame_HLS[:,:,2]  > self.thd_shadow_S        
        """we wont keep pixel with high saturation and low lightness"""
        LS_cond                        = (L_cond & S_cond)       
        frame_HLS[LS_cond,:]      = [0, 0, 0]
        
        shape           = (cv_image.shape[1], cv_image.shape[0])
        frame_skyview   = cv2.warpPerspective(cv_image, self.birdeye_matrix, (shape)) #, flags = cv2.INTER_LINEAR
        frame_RGB_skyview = frame_skyview
        
        
        if len(frame_HLS.shape)    == 2:
            mask_ = np.zeros_like(frame_HLS[:,:], dtype=np.uint8)
        elif len(frame_HLS.shape)  == 3:
            mask_ = np.zeros_like(frame_HLS[:,:,0], dtype=np.uint8)
            
        poly_mask=cv2.fillPoly(mask_, [self.vectorize_int(self.points_ZoI)], color=(255))
        
        frame_HLS[~(poly_mask > 0)] = [0, 0, 0]  
        
        frame_RGB_skyview_preprocessed=cv2.warpPerspective(cv2.cvtColor(frame_HLS,cv2.COLOR_HLS2BGR), self.birdeye_matrix, (shape))
        
        frame_gray=cv2.cvtColor(frame_RGB_skyview_preprocessed,cv2.COLOR_BGR2GRAY)
        
        frame_binary_2nd                = np.zeros_like(frame_gray, dtype = np.uint8)
        frame_binary_2nd[frame_gray > 0]= 255
        frame_binary_skyview            = frame_binary_2nd
        
        self.frame_width                = frame_binary_skyview.shape[1]
        self.frame_height               = frame_binary_skyview.shape[0]
        self.window_height              = self.vectorize_int(self.frame_height/self.num_of_windows)
        self.width_center               = self.vectorize_int(self.frame_width/2 - self.offset_cam)

        # find non zero pixel from grayscale frame
        self.nonzeropixel_x             = np.array(np.nonzero(frame_binary_skyview)[1])
        self.nonzeropixel_y             = np.array(np.nonzero(frame_binary_skyview)[0])
        self.frame_binary_skyview               = frame_binary_skyview
        self.thd_local_maximum          = (self.thd_confindex/100*self.num_of_windows)*self.min_pixel_inside*self.histogram_ratio_localmax

        histogram_slice_num     = self.vectorize_int(self.frame_width/self.histogram_seed)
        
        vertical_start          = self.vectorize_int(self.frame_height*self.histogram_vertical_ratio_start)
        vertical_end            = self.vectorize_int(self.frame_height*self.histogram_vertical_ratio_end)
        
        histogram               = np.sum(frame_binary_skyview[vertical_start : vertical_end, : ], axis = 0)
        histogram_seeded        = np.zeros_like(histogram)
        
        last_seed               = self.frame_width%(self.histogram_seed*histogram_slice_num)
        for ij in range(histogram_slice_num + 1):
            left                = self.histogram_seed*ij
            right               = self.histogram_seed*(ij + 1) - 1        
            histogram_seeded[self.vectorize_int(left + self.histogram_seed/2)-1]\
                                =  np.sum(histogram[left : right])        
        
        if last_seed > 0:
            histogram_seeded[self.histogram_seed*histogram_slice_num + self.vectorize_int(last_seed/2)]\
                                =  np.sum(histogram[self.histogram_seed*histogram_slice_num : self.frame_width - 1]) 
        
        histogram=histogram_seeded
        
        width_center                    = self.width_center
        px_ind_on_any_line              = np.array([], dtype = np.int64)
        # frame_RGB_draw_windows          = np.dstack((frame_binary_skyview*150, frame_binary_skyview*150, frame_binary_skyview*150))
        frame_RGB_draw_windows          = np.dstack((frame_binary_skyview, frame_binary_skyview, frame_binary_skyview))
    
        """find curve on the left side"""
        curve                           = []
        for ij in range(1, 15):
            # if histogram[np.argmax(histogram)] < self.thd_local_maximum :
            #     break
            
            # Sliding            
            max_x                           = self.corrector_windows_start(np.argmax(histogram))
            px_ind_on_line, px_ind_on_windows, px_ind_on_any_line, drawn_windows \
                                            = self.window_slide_a_curve(max_x, px_ind_on_any_line)
            x_found                         = self.nonzeropixel_x[px_ind_on_line]
            # Evaluate detected pixels
            confindex, linetype             = self.curve_type(px_ind_on_windows)
            
            # plot.figure()
            # plot.plot(histogram)
            # print(max_x)
            
            # Erase a bandwidth to do confuse not other sliding windows
            if len(x_found) > 0:
                left_boundary                               = self.vectorize_int(max([min([min(x_found), max_x - self.margin_x]), 0]))
                right_boundary                              = self.vectorize_int(min([max([max(x_found), max_x + self.margin_x]), self.frame_width - 1]))
                histogram[left_boundary : right_boundary]   = 0
            else:
                left_boundary                               = self.vectorize_int(max([max_x - self.histogram_width, 0]))
                right_boundary                              = self.vectorize_int(min([max_x + self.histogram_width, self.frame_width - 1]))
                histogram[left_boundary : right_boundary]   = 0
            
            # Gather information for a curve
            if confindex >= self.thd_confindex:
                """extrapolation of detected pixels"""
                x_found_ext, y_found_ext, xy_start, xy_end \
                                            = self.curve_extrapolation(px_ind_on_line, px_ind_on_windows)
                                                
                """ fit detected pixels with polynomial """
                coeff, frame_binary_line    = self.polyfit_3rd(x_found_ext, y_found_ext, frame_binary_skyview)
                
                """estimate space-left-in-lane at y = 0 from [y] = coeff*[x]"""
                x_window_center_from_vehicle_birdview   = [self.width_center -  (topleft_rightbottom[0][0] + topleft_rightbottom[1][0])/2 for topleft_rightbottom in drawn_windows]
                dist_at_0                               = np.mean(x_window_center_from_vehicle_birdview)
                
                """put all information into an instant a_curve"""
                coeff_from_vehicle_birdview = self.coeff_from_vehicle_birdview(coeff)
                a_curve = {'dist_at_0'                      : dist_at_0,
                           'abs_dist_at_0'                  : np.abs(dist_at_0),
                           'coeff'                          : coeff,
                           'coeff_from_vehicle_birdview'    : coeff_from_vehicle_birdview,                           
                           'xy_start'                       : xy_start,
                           'xy_end'                         : xy_end,
                           'confindex'                      : confindex,
                           'linetype'                       : linetype,
                           'drawn_windows'                  : drawn_windows}
                            #'frame_binary_line'              : frame_binary_line,
                curve.append(a_curve)

        """choose best two candidates for each side"""
        # a good candidate stays near to the frame center
        curve_L = []
        curve_R = []
        
        if len(curve) > 0:
            curve_L = [d for d in curve if d['dist_at_0'] >= 0] 
            curve_R = [d for d in curve if d['dist_at_0'] < 0] 
        
        # a good candidate stays near to the frame center
        if len(curve_L) > 0:
            curve_L = sorted(curve_L, key = lambda i: i['abs_dist_at_0'], reverse = False)
            del curve_L[2:]
        if len(curve_R) > 0:
            curve_R = sorted(curve_R, key = lambda i: i['abs_dist_at_0'], reverse = False)
            del curve_R[2:]            
            
        # left line
        """fit curve with 3rd polynomial"""
        if len(curve_L) > 0:
            confindex_L             = curve_L[0]['confindex']
            linetype_L              = curve_L[0]['linetype']
            coeff_L                 = curve_L[0]['coeff']
            xy_L_start              = curve_L[0]['xy_start']
            xy_L_end                = curve_L[0]['xy_end']
                        
            """draw windows"""
            frame_RGB_draw_windows          = self.draw_windows(curve_L[0]['drawn_windows'], frame_RGB_draw_windows, [255, 0, 0])
        else:
            confindex_L = 0
            linetype_L  = 0
            xy_L_start  = 0
            xy_L_end    = 0
            coeff_L, __    = self.polyfit_3rd([], [], frame_binary_skyview)

        # next-left line
        """fit curve with 3rd polynomial"""
        if len(curve_L) > 1:
            confindex_next_L             = curve_L[1]['confindex']
            linetype_next_L              = curve_L[1]['linetype']
            coeff_next_L                 = curve_L[1]['coeff']
            xy_next_L_start              = curve_L[1]['xy_start']
            xy_next_L_end                = curve_L[1]['xy_end']
            
            """draw windows"""
            frame_RGB_draw_windows                      = self.draw_windows(curve_L[1]['drawn_windows'], frame_RGB_draw_windows, [255, 255, 0])            
        else:
            confindex_next_L = 0
            linetype_next_L  = 0
            xy_next_L_start  = 0
            xy_next_L_end    = 0            
            coeff_next_L, __      = self.polyfit_3rd([], [], frame_binary_skyview)
            
        # right line
        """fit curve with 3rd polynomial"""
        if len(curve_R) > 0:
            confindex_R             = curve_R[0]['confindex']
            linetype_R              = curve_R[0]['linetype']
            coeff_R                 = curve_R[0]['coeff']
            xy_R_start              = curve_R[0]['xy_start']
            xy_R_end                = curve_R[0]['xy_end']
            
            """draw windows"""
            frame_RGB_draw_windows          = self.draw_windows(curve_R[0]['drawn_windows'], frame_RGB_draw_windows, [0, 0, 255])
        else:
            confindex_R = 0
            linetype_R  = 0
            xy_R_start  = 0
            xy_R_end    = 0            
            coeff_R, __    = self.polyfit_3rd([], [], frame_binary_skyview)

        # next-right line
        """fit curve with 3rd polynomial"""
        if len(curve_R) > 1:
            confindex_next_R             = curve_R[1]['confindex']
            linetype_next_R              = curve_R[1]['linetype']
            coeff_next_R                 = curve_R[1]['coeff']
            xy_next_R_start              = curve_R[1]['xy_start']
            xy_next_R_end                = curve_R[1]['xy_end']
            
            """draw windows"""
            frame_RGB_draw_windows                      = self.draw_windows(curve_R[1]['drawn_windows'], frame_RGB_draw_windows, [0, 255, 255])
        else:
            confindex_next_R = 0
            linetype_next_R  = 0
            xy_next_R_start  = 0
            xy_next_R_end    = 0                
            coeff_next_R, __      = self.polyfit_3rd([], [], frame_binary_skyview)
        
        ## Update current coeff_            
        self.coeff_L                                = coeff_L
        self.coeff_R                                = coeff_R
        self.confindex_L                            = confindex_L
        self.confindex_R                            = confindex_R
        self.linetype_L                             = linetype_L
        self.linetype_R                             = linetype_R
        self.coeff_next_L                           = coeff_next_L
        self.coeff_next_R                           = coeff_next_R
        self.confindex_next_L                       = confindex_next_L
        self.confindex_next_R                       = confindex_next_R
        self.linetype_next_L                        = linetype_next_L
        self.linetype_next_R                        = linetype_next_R
        
        self.xy_L_start                             = xy_L_start
        self.xy_L_end                               = xy_L_end    
        self.xy_next_L_start                        = xy_next_L_start
        self.xy_next_L_end                          = xy_next_L_end
        self.xy_R_start                             = xy_R_start
        self.xy_R_end                               = xy_R_end    
        self.xy_next_R_start                        = xy_next_R_start
        self.xy_next_R_end                          = xy_next_R_end   
        
        self.frame_RGB_draw_windows                 = frame_RGB_draw_windows
               
        """check lane change"""
        lane_change                                 = self.lanechange_check()
        
        """# Update previous data"""
        
        self.previous_data()
        
        frame_ZoI_source_points     = np.copy(cv_image)
        cv2.drawContours(frame_ZoI_source_points, [self.vectorize_int(self.points_ZoI)], contourIdx=-1, color=(255,0,0), thickness=5)
        frame_mix_ZoI_points        = cv2.addWeighted(frame_ZoI_source_points, 0.5, cv_image, 0.5, 0)    
       
        frame_source_points         = np.copy(cv2.cvtColor(frame_HLS,cv2.COLOR_HLS2BGR))
        frame_dest_points           = np.copy(cv2.cvtColor(frame_HLS, cv2.COLOR_HLS2BGR))
        cv2.drawContours(frame_source_points, [self.vectorize_int(self.source_points)], contourIdx=-1, color=(255,0,0), thickness=5)
        cv2.drawContours(frame_dest_points, [self.vectorize_int(self.destination_points)], contourIdx=-1, color=(0,255,0), thickness=5)
        frame_birdeye_points        = cv2.addWeighted(frame_dest_points, 0.5, frame_source_points, 0.5, 0)
        
        
        frame_RGB_skyview_CurrentLane       = np.zeros_like(cv_image)
        frame_RGB_skyview_CurrentLane       = self.frame_RGB_draw_curve(frame_RGB_skyview_CurrentLane, self.coeff_L, self.xy_L_start, self.xy_L_end, [255, 0, 50])
        frame_RGB_skyview_CurrentLane       = self.frame_RGB_draw_curve(frame_RGB_skyview_CurrentLane, self.coeff_R, self.xy_R_start, self.xy_R_end, [0, 0, 255])
        
        frame_RGB_skyview_NextLane          = np.zeros_like(cv_image)
        frame_RGB_skyview_NextLane          = self.frame_RGB_draw_curve(frame_RGB_skyview_NextLane, self.coeff_next_L, self.xy_next_L_start, self.xy_next_L_end, [255, 255, 0])
        frame_RGB_skyview_NextLane          = self.frame_RGB_draw_curve(frame_RGB_skyview_NextLane, self.coeff_next_R, self.xy_next_R_start, self.xy_next_R_end, [0, 255, 255])
        
        # Reversed birdseye view transform
        frame_RGB_camview_CurrentLane       = self.apply_vehicleview(frame_RGB_skyview_CurrentLane)
        frame_RGB_camview_NextLane          = self.apply_vehicleview(frame_RGB_skyview_NextLane)
        
        frame_mix_RGB_origin_Lane           = cv2.addWeighted(cv_image, 0.9, frame_RGB_camview_NextLane, 1, 0)
        frame_mix_RGB_origin_Lane           = cv2.addWeighted(frame_mix_RGB_origin_Lane, 0.9, frame_RGB_camview_CurrentLane, 1, 0)
        
        """make verbose frame"""
        # frame_mix_RGB_origin_Lane_verbose   = self.make_frame_verbose(preprocessing, self.curve_class, frame_mix_RGB_origin_Lane, frame_RGB_skyview, frame_binary_skyview)
        
        if self.xy_L_start!=0 and self.xy_R_start!=0:
            lane_middle = (self.xy_L_start[0]+self.xy_R_start[0])//2
            error = self.width_center-lane_middle
            print("error:",error)
            cv2.circle(frame_mix_RGB_origin_Lane,(lane_middle,300),5,(255,0,255),-1)
            
            self.cTime=time.time()
            self.PID.sample_time=(self.cTime-self.pTime)
            self.pTime=self.cTime
            
            control_output=self.PID(error)
            
            self.msg_velocity.angular.z=float(-control_output)
        
        else:
            self.msg_velocity.angular.z=0.0
            
        self.msg_velocity.linear.x=0.5
        
        self.publisher_.publish(self.msg_velocity)
            
        
        cv2.imshow("frame_mix",frame_mix_RGB_origin_Lane)
        
        key=cv2.waitKey(1)
        
    def corrector_windows_start(self, max_x_):
         """improving of start position for better accuracy, we can disable this section"""
         max_x_at_start  = max_x_
         histogram_width = self.histogram_width

         # Find a better starting position than max_x_, for start a new window in window_slider and for histogram bandwidth cleaning
         for ij in range(self.num_of_windows):
             mask_empty                              = np.array([], dtype = np.int64)
             x_left_                                 = max(0,max_x_ - histogram_width)
             x_right_                                = min(max_x_ + histogram_width, self.frame_width)
             y_top, y_bottom                         = self.next_y(ij)
             pixel_indice_for_histogram_windows      = self.find_pixel_indice_inside_rectangle(y_top, y_bottom, x_left_, x_right_, mask_empty)
             if len(pixel_indice_for_histogram_windows) >= self.min_pixel_inside:
                 max_x_at_start = self.vectorize_int(np.mean(self.nonzeropixel_x[pixel_indice_for_histogram_windows]))
                 break
             
         # # Find a better ending position than max_x_, for histogram bandwidth cleaning
         # for ij in range(self.num_of_windows - 1, -1, -1):
         #     mask_empty                              = np.array([], dtype = np.int64)
         #     x_left_                                 = max(0,max_x_ - histogram_width)
         #     x_right_                                = min(max_x_ + histogram_width, self.frame_width)
         #     y_top, y_bottom                         = self.next_y(ij)
         #     pixel_indice_for_histogram_windows      = self.find_pixel_indice_inside_rectangle(y_top, y_bottom, x_left_, x_right_, mask_empty)
         #     if len(pixel_indice_for_histogram_windows) >= self.min_pixel_inside:
         #         max_x_at_end = int(np.mean(self.nonzeropixel_x[pixel_indice_for_histogram_windows]))
         #         break
         # return max_x_at_start, max_x_at_end    
         return max_x_at_start
     
    def next_y(self, i_window):
        """go from bottom to top""" 
        y_top                   = self.frame_height - (i_window + 1)*self.window_height
        y_bottom                = self.frame_height - (i_window + 0)*self.window_height
        
        """go from top down to bottom""" 
        # y_top                   = (i_window + 0)*self.window_height
        # y_bottom                = (i_window + 1)*self.window_height
        
        return y_top, y_bottom
    
    def find_pixel_indice_inside_rectangle(self, top, bottom, left, right, mask_found_indice_on_any_line):
        mask_vertical           = (self.nonzeropixel_y <= bottom) & (self.nonzeropixel_y > top) # attention, we climb from the bottom of the frame to the top
        mask_horizontal         = (self.nonzeropixel_x >= left) & (self.nonzeropixel_x < right)
        
        mask_inside             = np.zeros_like(mask_vertical, dtype=np.uint8)     
        mask_inside[(mask_vertical & mask_horizontal)] = 1
        
        " exclude found indice on other lines"
        mask_inside[mask_found_indice_on_any_line]     = 0
        
        " exclude pixels from a window if they are so populated or so dispersed"
        x_inside                = self.nonzeropixel_x[np.nonzero(mask_inside)[0][:]]
        if len(x_inside) > 0:
            if (len(np.nonzero(mask_inside)[0][:]) > self.max_pixel_inside) \
                | (abs(max(x_inside) - min(x_inside)) > self.max_width_not_a_line) :
                    
                mask_inside[:]                         = 0
        
        """ return np.nonzero(mask_inside)[0][:] as indice of pixels found inside"""
        return np.nonzero(mask_inside)[0][:]
    
    def lanechange_check(self): #, coeff_L, coeff_R):
        lane_change                                 = False
        self.lanechange                             = 'None'
        
        # Change to vehicle origin
        coeff_L     = self.coeff_from_vehicle_birdview(self.coeff_L)
        coeff_R     = self.coeff_from_vehicle_birdview(self.coeff_R)        
        pre_coeff_L = self.coeff_from_vehicle_birdview(self.previous_coeff_L)
        pre_coeff_R = self.coeff_from_vehicle_birdview(self.previous_coeff_R)        
        
        is_all_zero_L                               = np.all((np.array(coeff_L) == 0))
        is_all_zero_R                               = np.all((np.array(coeff_R) == 0))
        is_all_zero_pre_L                           = np.all((np.array(pre_coeff_L) == 0))
        is_all_zero_pre_R                           = np.all((np.array(pre_coeff_R) == 0))
                
        if ~is_all_zero_pre_L & ~is_all_zero_R:
            coeff_grad = np.abs(pre_coeff_L - coeff_R)
            d_cond = coeff_grad[0] < 5E-6
            c_cond = coeff_grad[1] < 5E-3
            b_cond = coeff_grad[2] < 5E-1
            a_cond = coeff_grad[3] < 1E1
                            
            if a_cond & b_cond & c_cond & d_cond:
                lane_change                                 = True
                self.lanechange                             = 'Right to Left'
                
        if ~is_all_zero_pre_R & ~is_all_zero_L:
            coeff_grad = np.abs(pre_coeff_R - coeff_L)
            d_cond = coeff_grad[0] < 5E-6
            c_cond = coeff_grad[1] < 5E-3
            b_cond = coeff_grad[2] < 5E-1
            a_cond = coeff_grad[3] < 1E1
            
            if a_cond & b_cond & c_cond & d_cond:
                lane_change                                 = True
                self.lanechange                             = 'Left to Right'                      
        
        return lane_change
    
    def coeff_from_vehicle_birdview(self, coeff_from_topright_birdview):
        # + a translation of frame_width/2 in lateral direction
        # + a translation of frame_height in logitudinal
        # + a rotation of 180° around origin point
        # New Origin : Camera's center
        # New view : birdseye view
        # New x direction : from new Origin to the left of ego vehicle
        # New y direction : from new Origin toward ahead
        
        # coeff from the view at bottom-center of frame (cam origin)
        coeff_from_vehicle_birdview = [0, 0, 0, 0]
        
        is_all_zero                 = np.all((np.array(coeff_from_topright_birdview) == 0))
        if ~is_all_zero:        
            H                               = self.frame_height            
            d                               = coeff_from_topright_birdview[0]
            c                               = coeff_from_topright_birdview[1]
            b                               = coeff_from_topright_birdview[2]
            a                               = coeff_from_topright_birdview[3]           
                        
            coeff_from_vehicle_birdview       = np.zeros_like(coeff_from_topright_birdview)
            coeff_from_vehicle_birdview[0]    = d
            coeff_from_vehicle_birdview[1]    = -(3*d*H + c)
            coeff_from_vehicle_birdview[2]    = 3*d*H**2 + 2*c*H + b
            coeff_from_vehicle_birdview[3]    = -(d*H**3 + c*H**2 + b*H + a) + self.width_center
            
        return coeff_from_vehicle_birdview
    
    def previous_data(self):        
        self.previous_coeff_L                                = self.coeff_L        
        self.previous_coeff_R                                = self.coeff_R
        self.previous_confindex_L                            = self.confindex_L
        self.previous_confindex_R                            = self.confindex_R
        self.previous_linetype_L                             = self.linetype_L
        self.previous_linetype_R                             = self.linetype_R
        self.previous_coeff_next_L                           = self.coeff_next_L
        self.previous_coeff_next_R                           = self.coeff_next_R
        self.previous_confindex_next_L                       = self.confindex_next_L
        self.previous_confindex_next_R                       = self.confindex_next_R
        self.previous_linetype_next_L                        = self.linetype_next_L
        self.previous_linetype_next_R                        = self.linetype_next_R
        
    def window_slide_a_curve(self, window_x_mid, mask_found_indice_on_any_line):       
        pixel_indice_on_line                = np.array([], dtype = np.int64)   
        pixel_indice_on_windows             = []
        drawn_windows                       = []
        
        for window_i in range(self.num_of_windows):
            y_top, y_bottom                 = self.next_y(window_i)
            x_left, x_right                 = self.next_x(window_x_mid)
            
            # find pixel on current lane's curves            
            pixel_indice_on_window_i        = self.find_pixel_indice_inside_rectangle(y_top, y_bottom, x_left, x_right, mask_found_indice_on_any_line)        
            mask_found_indice_on_any_line   = np.append(mask_found_indice_on_any_line, pixel_indice_on_window_i)         
            # append found indice for each windows i inside a serie
            pixel_indice_on_line            = np.append(pixel_indice_on_line, pixel_indice_on_window_i)
            pixel_indice_on_windows.append(pixel_indice_on_window_i)
         
            # # draw windows
            if len(self.nonzeropixel_x[pixel_indice_on_window_i]) > 0:
                drawn_windows.append(((x_left,y_top), (x_right,y_bottom)))
            # found next-window's bottom center
            window_x_mid = self.next_window_x_mid_(window_x_mid, pixel_indice_on_window_i)
        
        return pixel_indice_on_line, pixel_indice_on_windows, mask_found_indice_on_any_line, drawn_windows

    def next_window_x_mid_(self, current_window_x_mid_, pixel_indice_inside):
        next_window_x_mid_      = current_window_x_mid_
        if len(pixel_indice_inside) >= self.min_pixel_inside:
             next_window_x_mid_ = self.vectorize_int(np.mean(self.nonzeropixel_x[pixel_indice_inside]))
        return next_window_x_mid_
    
    def next_x(self, mid_x_):
        x_left_                 = max(0, mid_x_ - self.margin_x)
        x_right_                = min(self.frame_width - 1, mid_x_ + self.margin_x)
        return x_left_, x_right_ 
    
    def curve_type(self, pixel_indice_on_windows_):                
        windows_size                = np.zeros(len(pixel_indice_on_windows_))
        mask_bold                   = np.zeros(len(pixel_indice_on_windows_))
        mask_doubleline             = np.zeros(len(pixel_indice_on_windows_))
        
        for ij in range(len(pixel_indice_on_windows_)):
            windows_size[ij]        = len(pixel_indice_on_windows_[ij])            
            x_in_window             = self.nonzeropixel_x[pixel_indice_on_windows_[ij]]
            
            if len(x_in_window) > 0:
                is_bold                 = (abs(max(x_in_window) - min(x_in_window)) > self.bold_width_px) \
                                        & (windows_size[ij] > self.min_pixel_bold)
                                        
                is_doubleline           = (abs(max(x_in_window) - min(x_in_window)) > self.doubleline_width_px) \
                                        & (windows_size[ij] > self.min_pixel_doubleline) \
                                        & ~is_bold
                if is_bold:
                    mask_bold[ij]       = 1
                if is_doubleline:
                    mask_doubleline[ij] = 1
            
        # Detect confiance level
        confindex           = self.vectorize_int(len(windows_size[windows_size > self.min_pixel_confindex])/len(windows_size)*100)        
        bold_index          = self.vectorize_int(len(mask_bold[mask_bold > 0])/len(mask_bold)*100)
        doubleline_index    = self.vectorize_int(len(mask_doubleline[mask_doubleline > 0])/len(mask_doubleline)*100)
                
        linetype    = 'No Line'
        # 30
        if self.thd_confindex < confindex < 85:
            linetype = 'Dashed'
        elif 85 <= confindex:
            linetype = 'Solid'
            
        if doubleline_index > 50:
            linetype = 'Double-line'
            
        if bold_index > 50:
            linetype = 'Bold'
       
        return confindex, linetype  
    
    def curve_extrapolation(self, pixel_indice_on_line_, pixel_indice_on_windows_):
        # detected pixels of a cruve
        x_found_                    = self.nonzeropixel_x[pixel_indice_on_line_]
        y_found_                    = self.nonzeropixel_y[pixel_indice_on_line_]        
                        
        """start and end point of detected curve, before extrapolation"""
        xy_start                            = (0,0)
        xy_end                              = (0,0)
        if len(y_found_) > 0:
            y_max_ind                       = np.argmax(y_found_)
            y_min_ind                       = np.argmin(y_found_)
            xy_start                        = (x_found_[y_max_ind], y_found_[y_max_ind]) # start is near to the frame's bottom
            xy_end                          = (x_found_[y_min_ind], y_found_[y_min_ind]) # end is near to the frame's top     
                
        """linear extrapolation based on 3 next (previous) windows that are not empty"""
        windows_size                = np.zeros(len(pixel_indice_on_windows_))
        for ij in range(len(pixel_indice_on_windows_)):
            windows_size[ij]        = len(pixel_indice_on_windows_[ij])       
        
        num_window_notempty         = len(windows_size[windows_size > self.min_pixel_confindex])
        if num_window_notempty >= self.vectorize_int(self.thd_confindex/100*len(windows_size)):
                        
            # Fill extrapolated pixels for first window
            x_base = np.array([], dtype = np.int64)
            y_base = np.array([], dtype = np.int64)
            count      = 0            
            if len(pixel_indice_on_windows_[0]) == 0:
                # pick 3 next windows
                for ki in range(1, len(pixel_indice_on_windows_)):
                    if len(pixel_indice_on_windows_[ki]) > 0:
                        count += 1
                        x_base     = np.append(x_base, np.mean(self.nonzeropixel_x[pixel_indice_on_windows_[ki]]))
                        y_base     = np.append(y_base, np.mean(self.nonzeropixel_y[pixel_indice_on_windows_[ki]]))
                        
                    if count == 3:                      
                        # fit curve with a line
                        coeff_extrapolated                = np.polyfit(y_base, x_base, 1)
                        
                        y_extrapolated                   = np.int64(np.linspace(self.frame_height - 1, self.frame_height - self.min_pixel_inside, self.min_pixel_inside))
                        x_extrapolated                   = np.int64(np.polyval(coeff_extrapolated, y_extrapolated))
                        
                        # eliminate out-of-frame pixels >> to be done in polyfit_3rd
                        """extrapolated pixel could be out of frame"""                        
                        x_found_                          = np.append(x_found_, x_extrapolated)
                        y_found_                          = np.append(y_found_, y_extrapolated)                        
                        break
                
            # Fill extrapolated pixels for final window
            x_base = np.array([], dtype = np.int64)
            y_base = np.array([], dtype = np.int64)
            count      = 0
            final_ind  = len(pixel_indice_on_windows_) - 1
            
            if len(pixel_indice_on_windows_[final_ind]) == 0:
                # pick 3 previous windows
                for ki in range(final_ind - 1, 0, -1):
                    if len(pixel_indice_on_windows_[ki]) > 0:
                        count += 1
                        x_base     = np.append(x_base, np.mean(self.nonzeropixel_x[pixel_indice_on_windows_[ki]]))
                        y_base     = np.append(y_base, np.mean(self.nonzeropixel_y[pixel_indice_on_windows_[ki]]))
                                            
                    if count == 3:                     
                        # fit curve with a line
                        coeff_extrapolated                = np.polyfit(y_base, x_base, 1)
                        
                        y_extrapolated                   = np.int64(np.linspace(0, self.min_pixel_inside - 1, self.min_pixel_inside))
                        x_extrapolated                   = np.int64(np.polyval(coeff_extrapolated, y_extrapolated))
                        
                        # eliminate out-of-frame pixels >> to be done in polyfit_3rd
                        """extrapolated pixel could be out of frame"""                        
                        x_found_                          = np.append(x_found_, x_extrapolated)
                        y_found_                          = np.append(y_found_, y_extrapolated)                        
                        break                    
            
        return x_found_, y_found_, xy_start, xy_end 
    
    def polyfit_3rd(self, x_found_, y_found_, frame_binary):
        """ fit detected pixels with polynomial """
        coeff_                              = [0, 0, 0, 0]
        frame_binary_line_                  = np.zeros_like(frame_binary, dtype = np.uint8)    
       
        # fit curve with 3rd degree polynomial        
        if len(y_found_) > 0:
            """ fit detected pixels with 3rd degree polynomial """
            coeff_                          = np.polyfit(y_found_, x_found_, 3)
            
            """"we removed out-of-frame pixels"""
            mask_outofframe                 =   (x_found_ > self.frame_width - 1) | (x_found_ < 0) \
                                              | (y_found_ > self.frame_height - 1) | (y_found_ < 0)
            x_found_                        = x_found_[~mask_outofframe]
            y_found_                        = y_found_[~mask_outofframe]
            
        if len(y_found_) > 0:
            # draw binary frame of detected pixels of a curve            
            frame_binary_line_[y_found_, x_found_] = 1        
        
        return coeff_, frame_binary_line_
    
    def draw_windows(self, drawn_windows, frame_RGB_draw_windows, window_color):       
        for lefttop_rightbottom in drawn_windows:    
            # draw windows
            cv2.rectangle(frame_RGB_draw_windows, lefttop_rightbottom[0], lefttop_rightbottom[1], window_color, 2)
        return frame_RGB_draw_windows    
        
    def histogram_seeded(self, frame_binary):
        histogram_slice_num     = int(self.frame_width/self.histogram_seed)
        
        vertical_start          = int(self.frame_height*self.histogram_vertical_ratio_start)
        vertical_end            = int(self.frame_height*self.histogram_vertical_ratio_end)
        
        histogram               = np.sum(frame_binary[vertical_start : vertical_end, : ], axis = 0)
        histogram_seeded        = np.zeros_like(histogram)
        
        last_seed               = self.frame_width%(self.histogram_seed*histogram_slice_num)
        for ij in range(histogram_slice_num + 1):
            left                = self.histogram_seed*ij
            right               = self.histogram_seed*(ij + 1) - 1        
            histogram_seeded[int(left + self.histogram_seed/2)]\
                                =  np.sum(histogram[left : right])        
        
        if last_seed > 0:
            histogram_seeded[self.histogram_seed*histogram_slice_num + int(last_seed/2)]\
                                =  np.sum(histogram[self.histogram_seed*histogram_slice_num : self.frame_width - 1])
                                
        return histogram_seeded
    
    def frame_RGB_draw_curve(self, frame_RGB, coeff_, xy_start, xy_end, line_color_ = [255, 0, 0]):
        is_all_zero_        = np.all((np.array(coeff_) == 0))
        if ~is_all_zero_:
            """draw A 3rd degree polynomial curves on a RGB-frame"""
            frame_width     = frame_RGB[:,:,0].shape[1]
            frame_height    = frame_RGB[:,:,0].shape[0]

            y_linespace     = np.uint64(np.linspace(0, frame_height - 1, self.vectorize_int(frame_height/3))) 
            x_linespace     = np.uint64(coeff_[0]*y_linespace**3 + coeff_[1]*y_linespace**2 + coeff_[2]*y_linespace + coeff_[3]*y_linespace**0)

            # remove pixel outside of frame
            # also remove extrapolated pixels
            mask_out_of_frame_  = (x_linespace < 0) | (x_linespace > frame_width - 1) \
                                    | (y_linespace < xy_end[1]) | (y_linespace > xy_start[1])
                                    # start near the frame's bottom
                                    # end near the frame's top
            y_linespace         = y_linespace[~mask_out_of_frame_]
            x_linespace         = x_linespace[~mask_out_of_frame_]

            """draw curve"""
            t = 8
            for (x, y) in zip(x_linespace, y_linespace):
                cv2.line(frame_RGB, (int(x - t), y), (int(x + t), y), line_color_, int(t / 2))
        return frame_RGB
    
    def apply_vehicleview(self, frame_skyview):
        """apply reversed birdseye view transform to get back to camera view"""
        shape           = (frame_skyview.shape[1], frame_skyview.shape[0])
        frame_camview   = cv2.warpPerspective(frame_skyview, self.inv_birdeye_matrix, shape)
        self.frame_camview = frame_camview
        return frame_camview
        
def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    rclpy.shutdown

if __name__=="__main__":
    main()