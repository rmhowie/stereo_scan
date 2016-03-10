#!/usr/bin/env python2.6


# stereoScan
#
# Finds the stereo correspondence using single camera calibration and an image pair with visible chessboards to find the extrinsic properties of the image pair.
# calibration images should be locate within images/calibration and the stereo pair with visible chessboards should be located within images/pair the left image should be taken first
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



#import standard modules
from __future__ import print_function #import new print function to replace the print statement
import os
import cv #OpenCV library
import numpy
import argparse
import fnmatch
#import pdb #Python debugger

#import non standard modules
import circlesGridImage
import camera
import stereo
import pointCloud

def main():

    parser = argparse.ArgumentParser(description="A program to do stereo correspondence matching on two image with calibration target in frame. Outputs disparity maps and a point cloud.")
    parser.add_argument("caseDirectory", type=str)
    caseDirectory = parser.parse_args().caseDirectory + "/"

    if(os.path.exists(caseDirectory + "caseDict.py")):

        caseDict = eval("{0}".format(open(caseDirectory + "caseDict.py").read()))

        if caseDict["camUsePreviousCalibrationData"]:
            if (os.path.exists(caseDirectory + caseDict["stereoPreviousIntrinsicsMatrix"]) and os.path.exists(caseDirectory + caseDict["stereoPreviousDistortionMatrix"])):
                doSingleCamcalibration = False
                print("\nUsing previously calculated single camera calibration data\n")
            #end if
            else:
                print("\nPrevious single camera calibration data not found, attempting to perform single camera calibration\n")
                doSingleCamcalibration = True
            #end else

        if doSingleCamcalibration:
            #do single camera calibration if required
            cam = camera.camera(
                verticalFormat = caseDict["camVerticalFormat"],
                pathToCalibrationImages = caseDirectory + caseDict["camCalibrationImagesPath"],
                calibrationImages = caseDict["camCalibrationImages"],
                numberOfPointsPerRow= caseDict["camNumberOfPointsPerRow"],
                numberOfPointsPerColumn = caseDict["camNumberOfPointsPerColumn"],
                circleSpacingSize = caseDict["camCircleSpacingSize"],
                drawCentres = caseDict["camDrawCentres"],
                pathToCalibrationImagesWithCentresDrawn = caseDirectory + caseDict["camCalibrationImagesWithCentresPath"])

            #store calibration data
            cameraIntrinsicsMatrix = cam.intrinsicsMatrix
            cameraDistortionMatrix = cam.distortionMatrix
            cameraIntrinsicsMatrix = cam.intrinsicsMatrix
            cameraDistortionMatrix = cam.distortionMatrix
        #end if
        else:
            #use single camera calibration data loaded from file
            cameraIntrinsicsMatrix = numpy.genfromtxt(caseDirectory + caseDict["stereoPreviousIntrinsicsMatrix"])
            cameraDistortionMatrix = numpy.genfromtxt(caseDirectory + caseDict["stereoPreviousDistortionMatrix"])
            cameraIntrinsicsMatrix = numpy.genfromtxt(caseDirectory + caseDict["stereoPreviousIntrinsicsMatrix"])
            cameraDistortionMatrix = numpy.genfromtxt(caseDirectory + caseDict["stereoPreviousDistortionMatrix"])
        #end else

        #check for stereo images
        stereoImageFileNamesList = []
        for file in os.listdir(caseDirectory + caseDict["stereoPathToStereoImages"]):
            if fnmatch.fnmatch(file, caseDict["stereoStereoImages"]):
                stereoImageFileNamesList.append(file)
            #end if
        #end for
        #sort the list into filename order
        stereoImageFileNamesList.sort()
        print(stereoImageFileNamesList)

        rgbImagesList = []
        xyzImagesList = []
        validityMasksList = []
        stList = []

        if(len(stereoImageFileNamesList) >= 2):

            for i in range(len(stereoImageFileNamesList) - 1):
                #initialise
                st = stereo.stereo(
                    verticalFormat = caseDict["stereoVerticalFormat"],
                    cameraOneIntrinsicsMatrix = cameraIntrinsicsMatrix,
                    cameraOneDistortionMatrix = cameraDistortionMatrix,
                    cameraTwoIntrinsicsMatrix = cameraIntrinsicsMatrix,
                    cameraTwoDistortionMatrix = cameraDistortionMatrix,
                    pathToImagePair = caseDirectory + caseDict["stereoPathToStereoImages"],
                    leftImageFilename = stereoImageFileNamesList[i],
                    rightImageFilename = stereoImageFileNamesList[i+1])

                #do stereo calibration
                if(i == 0):
                    (cameraRotationMatrix, cameraTranslationVectorMatrix, leftCameraRotationMatrix, leftCameraTranslationVectorMatrix, imageSize, errorFlag) = st.calibrate(
                    drawCentres = caseDict["stereoDrawCentres"],
                    pathToImagePairWithCentresDrawn = caseDirectory + caseDict["stereoStereoImagesWithCentresPath"],
                    numberOfPointsPerRow = caseDict["stereoNumberOfPointsPerRow"],
                    numberOfPointsPerColumn = caseDict["stereoNumberOfPointsPerColumn"],
                    circleSpacingSize = caseDict["stereoCircleSpacingSize"])
                #end if

                #rectify images
                (leftRectificationMatrix, _) = st.rectify(
                    imageSize = imageSize,
                    cameraRotationMatrix = cameraRotationMatrix,
                    cameraTranslationVectorMatrix = cameraTranslationVectorMatrix,
                    alpha = caseDict["stereoRectificationAlpha"],
                    calibrateForZeroDisparity = caseDict["stereoCalibrateForZeroDisparity"],
                    saveRectifiedImages = caseDict["stereoSaveRectifiedImages"],
                    pathToRectifiedImages = caseDirectory + caseDict["stereoRectifiedStereoImagesPath"],
                    imageScaleFactor = caseDict["stereoRectificationImageScaleFactor"])

                #calculate camera matricies relative to the first camera position
                if (i == 0):
                    relativeLeftCameraRotationMatrix = leftRectificationMatrix.transpose()#numpy.identity(3, dtype = numpy.float64)
                    relativeLeftCameraTranslationVectorMatrix = numpy.zeros((3,1), dtype = numpy.float64)
                else:
                    relativeLeftCameraRotationMatrix = numpy.dot(cameraRotationMatrix.transpose(), previousRelativeLeftCameraRotationMatrix)
                    relativeLeftCameraTranslationVectorMatrix = previousRelativeLeftCameraTranslationVectorMatrix - numpy.dot(cameraRotationMatrix.transpose(), cameraTranslationVectorMatrix)
                #end else
                previousRelativeLeftCameraRotationMatrix = numpy.copy(relativeLeftCameraRotationMatrix)
                previousRelativeLeftCameraTranslationVectorMatrix = numpy.copy(relativeLeftCameraTranslationVectorMatrix)

                #do semi global block matching
                st.calculateDisparitySGBM(
                    saveDisparityMaps = caseDict["SGBMSaveDisparityMaps"],
                    pathToDisparityMaps = caseDirectory + caseDict["SGBMDisparityMapsPath"],
                    useSobelPreFilter = caseDict["SGBMUseSobelPreFilter"],
                    preFilterCap = caseDict["SGBMPreFilterCap"],
                    SADWindowSize = caseDict["SGBMSADWindowSize"],
                    minDisparity = caseDict["SGBMMinDisparity"],
                    numberOfDisparities = caseDict["SGBMNumberOfDisparities"],
                    uniquenessRatio = caseDict["SGBMUniquenessRatio"],
                    speckleWindowSize = caseDict["SGBMSpeckleWindowSize"],
                    speckleRange = caseDict["SGBMSpeckleRange"],
                    P1 = caseDict["SGBMP1"],
                    P2 = caseDict["SGBMP2"])

                #create the data to generate point clouds
                st.createPointClouds(
                    cameraRotationMatrix = relativeLeftCameraRotationMatrix,
                    cameraTranslationVectorMatrix = relativeLeftCameraTranslationVectorMatrix)

                #append point cloud data to the relevant lists
                rgbImagesList.append(st.leftRgbImage2DArray)
                xyzImagesList.append(st.leftTransformedXyzImage2DArray)
                validityMasksList.append(st.leftSuccessfulMatchMask)
            #end for

            #concatenate point cloud data
            concatenatedRgbArray = rgbImagesList[0]
            concatenatedXyxArray = xyzImagesList[0]
            concatenatedValidityMask = validityMasksList[0]

            for i in range(len(stereoImageFileNamesList) - 2):
                concatenatedRgbArray = numpy.concatenate((concatenatedRgbArray, rgbImagesList[i+1]), axis = 1)
                concatenatedXyxArray = numpy.concatenate((concatenatedXyxArray, xyzImagesList[i+1]), axis = 1)
                concatenatedValidityMask = numpy.concatenate((concatenatedValidityMask, validityMasksList[i+1]), axis = 1)

            #create point cloud from left image and disparity map
            pc = pointCloud.xyzrgbPointCloudOriented(
                rgbImage = concatenatedRgbArray,
                xyzImage = concatenatedXyxArray,
                validityMask = concatenatedValidityMask)

            print("Writing point cloud of {0} points ...\n".format(concatenatedValidityMask.shape[1] - numpy.sum(concatenatedValidityMask)))

            #write to disk
            pc.writeUnstructured(
                pointCloudPath = caseDirectory,
                pointCloudFilename = caseDict["PCPointCloudFilename"])

        #end if
        else:
            print("Error: two or more images not found for stereo matching")
        #end else

    #end if
    else:
        print("Error: Case directory not found")
    #end else
# end main


if __name__ == "__main__":

    main()

# end if