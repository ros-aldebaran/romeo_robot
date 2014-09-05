import math
import romeo_description.urdf as ur
pi_2 = math.pi/2.0
Nao_offsets = {
    #### mechanical offsets #####

    'FootHeight_V50':4.519e-2,
    'HandOffsetX_V50':5.775e-2,
    'HandOffsetZ_V50':1.231e-2,
    'FootHeight_V40':4.519e-2,
    'HandOffsetX_V40':5.775e-2,
    'HandOffsetZ_V40':1.231e-2,
    'FootHeight_V33':4.519e-2,
    'HandOffsetX_V33':5.775e-2,
    'HandOffsetZ_V33':1.231e-2,
    'FootHeight_V32':4.511e-2,
    'HandOffsetX_V32':5.8e-2,
    'HandOffsetZ_V32':1.59e-2,

    #### sensors offsets  ####

    # cameras
    'CameraTopV4OffsetX':5.871e-2,
    'CameraTopV4OffsetY':0.0,
    'CameraTopV4OffsetZ':6.364e-2,
    'CameraTopV4RotX':0.0,
    'CameraTopV4RotY':2.09e-2,
    'CameraTopV4RotZ':0.0,
    'CameraBottomV4OffsetX':5.071e-2,
    'CameraBottomV4OffsetY':0.0,
    'CameraBottomV4OffsetZ':1.774e-2,
    'CameraBottomV4RotX':0.0,
    'CameraBottomV4RotY':0.6929,
    'CameraBottomV4RotZ':0.0,
    'CameraTopV3OffsetX':5.39e-2,
    'CameraTopV3OffsetY':0.0,
    'CameraTopV3OffsetZ':6.79e-2,
    'CameraTopV3RotX':0.0,
    'CameraTopV3RotY':0.0,
    'CameraTopV3RotZ':0.0,   
    'CameraBottomV3OffsetX':4.88e-2,
    'CameraBottomV3OffsetY':0.0,
    'CameraBottomV3OffsetZ':2.381e-2,
    'CameraBottomV3RotX':0.0,
    'CameraBottomV3RotY':0.6981,
    'CameraBottomV3RotZ':0.0,

    # sonars
    'SonarOffsetX':5.07e-2,
    'SonarOffsetY':3.785e-2,
    'SonarOffsetZ':6.035e-2,
    'SonarRotX':0.0,
    'SonarRotY':4.365e-2,
    'SonarRotZ':-0.39265,

    # InfraRed
    'IROffsetX':4.41e-2,
    'IROffsetY':2.57e-2,
    'IROffsetZ':4.66e-2,
    
    # FSRs
    'FSROffsetXFL':0.07025,
    'FSROffsetYFL':0.0299,
    'FSROffsetYFR':0.0231,
    'FSROffsetXRL':0.03025,
    'FSROffsetXRR':0.02965,
    'FSROffsetYRR':0.0191,
    
    # IMU
    'IMUGyroOffsetX':8e-3,
    'IMUGyroOffsetY':6e-3,
    'IMUGyroOffsetZ':2.9e-2,
    'IMUAcceleroOffsetX':8e-3,
    'IMUAcceleroOffsetY':6.06e-3,
    'IMUAcceleroOffsetZ':2.7e-2,

    # Bumpers
    'FootBumperOffsetX':8.76e-2,
    'FootBumperOffsetYLeft':1.9e-2,
    'FootBumperOffsetYRight':1.4e-2,
    'FootBumperOffsetZ':3.5e-2,

    'ChestButtonOffsetX':4.9871e-2,
    'ChestButtonOffsetY':1.588e-3,
    'ChestButtonOffsetZ':5.5163e-2,
 
    # Touch Sensors
        # Head Touch sensors
    'HeadTouchFrontOffsetX':3.12e-2,
    'HeadTouchFrontOffsetY':0.0,
    'HeadTouchFrontOffsetZ':0.1014,
    'HeadTouchFrontRotX':0.0,
    'HeadTouchFrontRotY':-1.186099535,
    'HeadTouchFrontRotZ':0.0,

    'HeadTouchMiddleOffsetX':1.0e-3,
    'HeadTouchMiddleOffsetY':0.0,
    'HeadTouchMiddleOffsetZ':0.1099,
    'HeadTouchMiddleRotX':0.0,
    'HeadTouchMiddleRotY':-pi_2,
    'HeadTouchMiddleRotZ':0.0,

    'HeadTouchRearOffsetX':-2.57e-2,
    'HeadTouchRearOffsetY':0.0,
    'HeadTouchRearOffsetZ':0.1045,
    'HeadTouchRearRotX':math.pi,
    'HeadTouchRearRotY':-1.333192995,
    'HeadTouchRearRotZ':math.pi,

    # Hand Touch sensor
    'HandTouchLeftOffsetX':3.2e-2,
    'HandTouchLeftOffsetY':2.5e-2,
    'HandTouchLeftOffsetZ':3e-3,
    'HandTouchLeftRotX':0.0,
    'HandTouchLeftRotY':0.0,
    'HandTouchLeftRotZ':pi_2,
    
    'HandTouchBackOffsetX':3.8e-2,
    'HandTouchBackOffsetY':0.0,
    'HandTouchBackOffsetZ':2.5e-2,
    'HandTouchBackRotX':0.0,
    'HandTouchBackRotY':-pi_2,
    'HandTouchBackRotZ':0.0,

    'HandTouchRightOffsetX':3.5e-2,
    'HandTouchRightOffsetY':-2.5e-2,
    'HandTouchRightOffsetZ':3.5e-3,
    }

Nao_links = {
    u'TorsoLink':'torso',        	        # collada
    u'HeadYawLink':'Neck',
    u'HeadPitchLink':'Head',
    u'LShoulderPitchLink':'LShoulder', 
    u'RShoulderPitchLink':'RShoulder', 
    u'LShoulderRollLink':'LBicep',       	        # collada
    u'RShoulderRollLink':'RBicep', #u'HeadPitchLink':'gaze',
    u'LElbowYawLink':'LElbow',
    u'RElbowYawLink':'RElbow',
    u'LElbowRollLink':'LForeArm',
    u'RElbowRollLink':'RForeArm',
    u'LHipYawPitchLink':'LPelvis',
    u'RHipYawPitchLink':'RPelvis',
    u'LHipRollLink':'LHip',
    u'RHipRollLink':'RHip',
    u'LHipPitchLink':'LThigh',
    u'RHipPitchLink':'RThigh',
    u'LKneePitchLink':'LTibia',
    
    u'RKneePitchLink':'RTibia',
    u'LAnklePitchLink':'l_ankle',
    u'RAnklePitchLink':'r_ankle',
    u'LAnkleRollLink':'l_sole',            # collada
    u'RAnkleRollLink':'r_sole',  	        # collada
    u'LWristYawLink':'l_wrist',             # collada
    u'RWristYawLink':'r_wrist',             # collada
    }

Nao_visu = {
    u'TorsoLink': ur.Cylinder(0.015,0.2115),       	        # collada
    u'HeadPitchLink':ur.Cylinder(0.04,0.14),                 # collada
    u'LShoulderRollLink':ur.Cylinder(0.015,0.09),            # collada
    u'LElbowRollLink': ur.Cylinder(0.015,0.05065),           # collada
    u'LHipPitchLink': ur.Cylinder(0.015,0.1),                # collada
    u'LKneePitchLink':   ur.Cylinder(0.015,0.1),             # collada
    #u'LanklePitchLink':     ur.Cylinder(0.015,0.046),            # collada
    u'RHipPitchLink': ur.Cylinder(0.015,0.1),                # collada  
    
    u'RKneePitchLink': ur.Cylinder(0.015,0.1),  	            # collada
    #u'RAnklePitchLink':   ur.Cylinder(0.015,0.046),   	        # collada
    u'RShoulderRollLink': ur.Cylinder(0.015,0.09),            # collada
    u'RElbowRollLink':  ur.Cylinder(0.015,0.05065),          # collada
    u'LAnkleRollLink': ur.Box((0.16,0.06,0.015)),
    u'RAnkleRollLink': ur.Box((0.16,0.06,0.015)),
    u'LWristYawLink':  ur.Cylinder(0.015,0.058), 
    u'RWristYawLink':  ur.Cylinder(0.015,0.058), 
    }
Nao_orig=  {
    u'TorsoLink': ur.Pose((0,0,0.02075),(0,0,0)),       	        # collada
    u'HeadPitchLink':  ur.Pose((0,0,0.058),(pi_2,0,0)),          # collada
    u'LShoulderRollLink':  ur.Pose((0.045,0,0),(pi_2,0,pi_2)),   # collada
    u'LElbowRollLink': ur.Pose((0.025325,0,0),(pi_2,0,pi_2)),    # collada
    u'LHipPitchLink':   ur.Pose((0,0,-0.05),(0,0,0)),            # collada
    u'LKneePitchLink':  ur.Pose((0,0,-0.05),(0,0,0)),            # collada
    #u'LanklePitchLink':      ur.Pose((0,0,-0.023),(0,0,0)),          # collada
    u'RHipPitchLink':   ur.Pose((0,0,-0.05),(0,0,0)),            # collada  
    u'RKneePitchLink': ur.Pose((0,0,-0.05),(0,0,0)),    	        # collada
    #u'RAnklePitchLink':       ur.Pose((0,0,-0.023),(0,0,0)),  	    # collada
    u'RShoulderRollLink': ur.Pose((0.045,0,0),(pi_2,0,pi_2)),    # collada
    u'RElbowRollLink': ur.Pose((0.025325,0,0),(pi_2,0,pi_2)),    # collada
    u'LAnkleRollLink': ur.Pose((0.02,0,0.0075),(0,0,0)),
    u'RAnkleRollLink': ur.Pose((0.02,0,0.0075),(0,0,0)),
    u'LWristYawLink': ur.Pose((0.029,0,0),(pi_2,0,pi_2)),
    u'RWristYawLink': ur.Pose((0.029,0,0),(pi_2,0,pi_2)),
    }

