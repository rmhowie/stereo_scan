# caseDict
#
# This file contains the dictionary for the parameters that are needed to process the case
#
#  ##########################################################################
#  #                                                                        #
#  #  NOTE: This file is where the parameters are entered before run time   #
#  #                                                                        #
#  ##########################################################################
#
# Copyright (C) Robert Howie 2011.
#
################################GPL Version 3###############################
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
#############################################################################
#
# You should have recived a copy of the GNU GPL with this program in the file
# "gpl.txt" located in the same directory as this file.

dict(


#### Simgle Camera Calibration ####


    camVerticalFormat = False,
    #whether the images are taken in a horizontal or vertical orientation
    #default: True
    #True: vertical format (grip up on a Canon 60D), False: horizontal format (camera upright)

    camCalibrationImagesPath = "images/calibration/",
    #path to the calibration images for single camera calibration
    #default: "images/calibration/"
    #trailing slash is important becuase the path is just concatenated with the filename

    camCalibrationImages = "*.JPG",
    #filename matching patern for the calibration images
    #default: "*.JPG"

    camUsePreviousCalibrationData = True,
    #use previous single camera calibration data if available
    #default: True
    #will to calibration if data is not found

    camNumberOfPointsPerRow = 7,
    #the number of circles per row in the asymmetric circles grid calibration target
    #default: 6

    camNumberOfPointsPerColumn = 21,
    #the number of circles per column in the asymmetric circles grid calibration target
    #default: 8

    camCircleSpacingSize = 50.0,
    #the size of each small gap on the asymmetric circles grid calibration target
    #default: 25.0
    #a chessboardSquareSize of 1 means the base unit is 1 small gap

    camDrawCentres = True,
    #whether corners should be drawn on the single camera calibration images
    #default = True

    camCalibrationImagesWithCentresPath = "images/calibration/centresDrawn/",
    #the directory to save calibration images with centres added"
    #default: "images/calibration/cornersDrawn/"
    #trailing slash is important becuase the path is just concatanated with the filename


#### Stereo Calibration and Rectification ####


    stereoVerticalFormat = False,
    #whether the images are taken in a horizontal or vertical orientation
    #default: True
    #True: vertical format (grip up on a Canon 60D), False: horizontal format (camera upright)

    stereoPreviousIntrinsicsMatrix = "images/calibration/intrinsicMatrix.txt",
    #name of camera intrinsic matrix from previous calibration
    #default: "intrinsicMatrix.txt"
    #the data is saved under the default name,
    #so this only needs to be changed if the file has been renamed.

    stereoPreviousDistortionMatrix = "images/calibration/distortionMatrix.txt",
    #name of camera distortion matrix from previous calibration
    #default: "distortionMatrix.txt"
    #the data is saved under the default name,
    #so this only needs to be changed if the file has been renamed.

    stereoPathToStereoImages = "images/pairs/",
    #path to the stereo image pair
    #default: "images/pair/"
    #trailing slash is important becuase the path is just concatenated with the filename

    stereoStereoImages = "*.JPG",

    stereoNumberOfPointsPerRow = 7,
    #the number of circles per row in the asymmetric circles grid calibration target
    #default: 6

    stereoNumberOfPointsPerColumn = 21,
    #the number of circles per column in the asymmetric circles grid calibration target
    #default: 8

    stereoCircleSpacingSize = 50.0,
    #the size of each small gap on the asymmetric circles grid calibration target
    #default: 25.0
    #a chessboardSquareSize of 1 means the base unit is 1 small gap

    stereoDrawCentres = True,
    #whether corners should be drawn on the single camera calibration images
    #default = True

    stereoStereoImagesWithCentresPath = "images/pairs/centresDrawn/",
    #the directory to save stereo pair images with centres added"
    #default: "images/pairs/centresDrawn/"
    #trailing slash is important becuase the path is just concatanated with the filename

    stereoSaveRectifiedImages = True,
    #whether the rectified image pair should be saved to disk

    stereoRectifiedStereoImagesPath = "images/pairs/rectified/",
    #the directory to save rectified stereo pairs"
    #default: "images/pairs/rectified/"
    #trailing slash is important becuase the path is just concatanated with the filename

    stereoRectificationAlpha = -1,
    #the scaling factor for rectification
    #default: -1 (lets OpenCV use it's sensible default)
    #1: keep everything, 0: throw out everything that may not be valid

    stereoCalibrateForZeroDisparity = True,
    #calibrate for zero disparity = zero depth
    #default: False

    stereoRectificationImageScaleFactor = 0.8,
    #scale factor for the rectified images
    #default: 0.8


#### Semi Global Block Matching ####

    SGBMSaveDisparityMaps = False,
    #whether the disparity maps should be saved to disk
    #default: True

    SGBMDisparityMapsPath = "images/pairs/disparity/",
    #the directory to save disparity images"
    #default: "images/pairs/disparity"
    #trailing slash is important becuase the path is just concatanated with the filename

    SGBMUseSobelPreFilter = True,
    #use the sobel prefilter, if false the normalized response filter is used instead
    #default: True

    SGBMSADWindowSize = 11,
    #size of the window to match, must be odd and greater than 5
    #default: 15

    SGBMMinDisparity = 1*16,
    #minimum disparity
    #default: 0*16

    SGBMNumberOfDisparities = 3*16,
    #lenght of the search window, must be divisible by 16
    #default: 20*16

    SGBMPreFilterCap = 63,
    #maximum value allowed after the prefilter is applied (top limit)
    #default: 63

    SGBMUniquenessRatio = 5,
    #the uniqueness ration, 5-15 is usually good according to the API documentation
    #default: 15

    SGBMSpeckleWindowSize = 300,
    #size of the speckle/snow filtering window
    #default: 300

    SGBMSpeckleRange = 16,
    #maximum disparity variation in connected components
    #must be a multiple of 16, API documentation reccomends 16 or 32
    #default: 16

    SGBMP1 = 0*3*11*11,
    #SGBM cost parameter for neighboring pixels
    #API Documentation reccomends 8*numberOfChannels*SADWindowSize*SADWindowSize
    #default: 8*3*9*9

    SGBMP2 = 0*3*11*11,
    #SGBM cost parameter for non-neighboring pixels
    #API Documentation reccomends 32*numberOfChannels*SArighDWindowSize*SADWindowSize
    #default: 16*3*9*9


#### Point Cloud Generation ####

    PCPointCloudFilename = "pointCloud.pcd")
    #name to save the output point cloud as
    #default "pointCloud.pcd"