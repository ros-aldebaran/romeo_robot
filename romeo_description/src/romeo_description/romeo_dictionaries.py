import math
pi_2 = math.pi/2.0

############################
########## ROMEO ###########
############################

Romeo_offsets = {
    #### mechanical offsets #####

    'FootHeight':6.84e-2, # 41.1 + 23.7
    'HandOffsetX':5.775e-2,
    'HandOffsetZ':1.231e-2,

    #### sensors offsets  ####

    # cameras
    'CameraLeftEyeOffsetX':0.11017,
    'CameraLeftEyeOffsetY':3.192e-2,
    'CameraLeftEyeOffsetZ':5.426e-2,
    'CameraLeftEyeRotX':0.0,
    'CameraLeftEyeRotY':0.0,
    'CameraLeftEyeRotZ':0.0,
    'CameraRightEyeOffsetX':0.11017,
    'CameraRightEyeOffsetY':-3.192e-2,
    'CameraRightEyeOffsetZ':5.426e-2,
    'CameraRightEyeRotX':0.0,
    'CameraRightEyeRotY':0.0,
    'CameraRightEyeRotZ':0.0,

    'CameraLeftOffsetX':0.11999,
    'CameraLeftOffsetY':4.0e-2,
    'CameraLeftOffsetZ':9.938e-2,
    'CameraLeftRotX':0.0,
    'CameraLeftRotY':0.0,
    'CameraLeftRotZ':0.0,
    'CameraRightOffsetX':0.11999,
    'CameraRightOffsetY':-4.0e-2,
    'CameraRightOffsetZ':9.938e-2,
    'CameraRightRotX':0.0,
    'CameraRightRotY':0.0,
    'CameraRightRotZ':0.0,

    'CameraDepthOffsetX':0.1403,
    'CameraDepthOffsetY':-4.708e-2,
    'CameraDepthOffsetZ':0.14655,
    'CameraDepthRotX':2.0043951,
    'CameraDepthRotY':0.0,
    'CameraDepthRotZ':pi_2,

    # FSRs
    'FsrFLOffsetX':0.13,
    'FsrFLOffsetY':0.0337,
    'FsrFLOffsetZ':-0.0646,
    'FsrFLRotX':0.0,
    'FsrFLRotY':0.0,
    'FsrFLRotZ':0.0,

    'FsrFROffsetX':0.13,
    'FsrFROffsetY':-0.0337,
    'FsrFROffsetZ':-0.0646,
    'FsrFRRotX':0.0,
    'FsrFRRotY':0.0,
    'FsrFRRotZ':0.0,

    'FsrCenterOffsetX':0.073,
    'FsrCenterOffsetY':0.02,
    'FsrCenterOffsetZ':-0.0646,
    'FsrCenterRotX':0.0,
    'FsrCenterRotY':0.0,
    'FsrCenterRotZ':0.0,

    'FsrRCenterOffsetX':-0.04,
    'FsrRCenterOffsetY':0.0,
    'FsrRCenterOffsetZ':-0.0646,
    'FsrRCenterRotX':0.0,
    'FsrRCenterRotY':0.0,
    'FsrRCenterRotZ':0.0,
    
    # IMU
    'IMUTorsoAcceleroOffsetX':6.185e-2,
    'IMUTorsoAcceleroOffsetY':8.7e-3,
    'IMUTorsoAcceleroOffsetZ':-0.1582,
    'IMUTorsoAcceleroRotX':0.0,
    'IMUTorsoAcceleroRotY':0.0,
    'IMUTorsoAcceleroRotZ':0.0,
    'IMUTorsoGyroOffsetX':6.185e-2,
    'IMUTorsoGyroOffsetY':8.7e-3,
    'IMUTorsoGyroOffsetZ':-0.1582,
    'IMUTorsoGyroRotX':0.0,
    'IMUTorsoGyroRotY':0.0,
    'IMUTorsoGyroRotZ':0.0,


    'IMUHeadAcceleroOffsetX':-1.135e-2,
    'IMUHeadAcceleroOffsetY':-4.225e-2,
    'IMUHeadAcceleroOffsetZ':0.16011,
    'IMUHeadAcceleroRotX':0.0,
    'IMUHeadAcceleroRotY':0.0,
    'IMUHeadAcceleroRotZ':pi_2,
    'IMUHeadGyroOffsetX':-1.135e-2,
    'IMUHeadGyroOffsetY':-4.225e-2,
    'IMUHeadGyroOffsetZ':0.16011,
    'IMUHeadGyroRotX':0.0,
    'IMUHeadGyroRotY':0.0,
    'IMUHeadGyroRotZ':pi_2,
 
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
    }



Romeo_links = {
    u'TorsoLink':'torso',        	        # collada
    u'TrunkYawLink':'body',
    u'LAnkleRollLink':'l_ankle',            # collada
    u'RAnkleRollLink':'r_ankle',  	        # collada
    u'LWristPitchLink':'l_wrist',             # collada
    u'RWristPitchLink':'r_wrist',             # collada
    }

