#!/usr/bin/env python2.6


# stereo
#
# does the stereo correspondence matching
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
import cv #OpenCV library
import cv2
import pdb #Python debugger
import os
import errno
import numpy
import numpy.ma #numpy masked arrays
import time

#import non standard modules
import circlesGridImage
import caseDict
import pointCloud

class stereo:
    "A class to do the stereo image calibration and processing"

    def __init__(
        self,
        verticalFormat,
        cameraOneIntrinsicsMatrix,
        cameraOneDistortionMatrix,
        cameraTwoIntrinsicsMatrix,
        cameraTwoDistortionMatrix,
        pathToImagePair,
        leftImageFilename,
        rightImageFilename):

        self.verticalFormat = verticalFormat
        self.cameraOneIntrinsicsMatrix = cameraOneIntrinsicsMatrix
        self.cameraOneDistortionMatrix = cameraOneDistortionMatrix
        self.cameraTwoIntrinsicsMatrix = cameraTwoIntrinsicsMatrix
        self.cameraTwoDistortionMatrix = cameraTwoDistortionMatrix
        self.pathToImagePair = pathToImagePair
        self.leftImageFilename = leftImageFilename
        self.rightImageFilename = rightImageFilename

        #load the images
        self.unorientedLeftImage = cv2.imread(pathToImagePair + leftImageFilename, 1)
        self.unorientedRightImage = cv2.imread(pathToImagePair + rightImageFilename, 1)
        #orient the images
        if verticalFormat:
            self.orientedLeftImage = cv2.transpose(self.unorientedLeftImage)
            self.orientedLeftImage = cv2.flip(self.orientedLeftImage, 0)
        #end if
        else:
            self.orientedLeftImage = numpy.copy(self.unorientedLeftImage)
        #right
        self.unorientedRightImage = cv2.imread(pathToImagePair + rightImageFilename, 1)
        #orient the images
        if verticalFormat:
            self.orientedRightImage = cv2.transpose(self.unorientedRightImage)
            self.orientedRightImage = cv2.flip(self.orientedRightImage, 0)
        #end if
        else:
            self.orientedRightImage = numpy.copy(self.unorientedRightImage)

    #end __init__

    def calibrate(
        self,
        drawCentres,
        pathToImagePairWithCentresDrawn,
        numberOfPointsPerRow,
        numberOfPointsPerColumn,
        circleSpacingSize):

        #initialise error flag as False
        errorFlag = False

        #check for images with corners drawn directory if required
        if drawCentres:
            try:
                os.makedirs(pathToImagePairWithCentresDrawn)
            except OSError as error:
                if error.errno == errno.EEXIST:
                    pass
                #end if
                else:
                    drawCentres = False
                    raise
                #end else
            #end except
       #end if

        #find left image points
        self.leftImage = circlesGridImage.circlesGridImage(
            self.verticalFormat,
            self.pathToImagePair,
            self.leftImageFilename,
            numberOfPointsPerRow,
            numberOfPointsPerColumn,
            True,
            pathToImagePairWithCentresDrawn,
            "Centres_" + self.leftImageFilename)

        #find right image points
        self.rightImage = circlesGridImage.circlesGridImage(
            self.verticalFormat,
            self.pathToImagePair,
            self.rightImageFilename,
            numberOfPointsPerRow,
            numberOfPointsPerColumn,
            True,
            pathToImagePairWithCentresDrawn,
            "Centres_" + self.rightImageFilename)

        #check that the chessboards were successfully found
        if(self.leftImage.success and self.rightImage.success):

            #check that the image sizes are the same
            if(self.leftImage.colourCalibrationImage.shape[:2] == self.rightImage.colourCalibrationImage.shape[:2]):

                #calculate number of circles in circles grid
                numberOfCirclesGridPoints = numberOfPointsPerRow*numberOfPointsPerColumn

                #create matricies for calibration parameters
                #input matricies
                objectPoints = numpy.zeros((1, numberOfCirclesGridPoints, 3), dtype=numpy.float32)

                #image points
                #leftImagePoints = numpy.zeros((1, numberOfCirclesGridPoints, 2), dtype=numpy.float32)
                #rightImagePoints = numpy.zeros((1, numberOfCirclesGridPoints, 2), dtype=numpy.float32)

                #set parameter matricies
                #object points
                for i in range(numberOfPointsPerColumn):
                    for j in range(numberOfPointsPerRow):
                        objectPoints[0][i*numberOfPointsPerRow + j][0] = float(i)*float(circleSpacingSize)
                        objectPoints[0][i*numberOfPointsPerRow + j][1] = float(i%2 + 2*j)*float(circleSpacingSize)
                        objectPoints[0][i*numberOfPointsPerRow + j][2] = 0.0
                    #end for
                #end for

                #pdb.set_trace() #start debugging
                #print([self.leftImage.centreLocations])
                #print(objectPoints)

                #retval, self.cameraOneIntrinsicsMatrix, self.cameraOneDistortionMatrix, self.cameraTwoIntrinsicsMatrix, self.cameraTwoDistortionMatrix, self.cameraRotationMatrix,  self.cameraTranslationVectorMatrix, self.essentialMatrix, self.fundamentalMatrix = cv2.stereoCalibrate(
                    #objectPoints,
                    #[self.leftImage.centreLocations],
                    #[self.rightImage.centreLocations],
                    #self.cameraOneIntrinsicsMatrix,
                    #self.cameraOneDistortionMatrix,
                    #self.cameraTwoIntrinsicsMatrix,
                    #self.cameraTwoDistortionMatrix,
                    #self.imageSize,
                    #criteria = (cv.CV_TERMCRIT_ITER, 120, 0),
                    #flags = cv.CV_CALIB_FIX_INTRINSIC)

                leftCameraRotationVector, leftCameraTranslationVectorMatrix = cv2.solvePnP(
                    objectPoints,
                    self.leftImage.centreLocations,
                    self.cameraOneIntrinsicsMatrix,
                    self.cameraOneDistortionMatrix)

                rightCameraRotationVector, rightCameraTranslationVectorMatrix = cv2.solvePnP(
                    objectPoints,
                    self.rightImage.centreLocations,
                    self.cameraTwoIntrinsicsMatrix,
                    self.cameraTwoDistortionMatrix)

                #convert from rotation vectors (axis-angle representation) rotation matricies
                leftCameraRotationMatrix, _ = cv2.Rodrigues(leftCameraRotationVector)
                rightCameraRotationMatrix, _ = cv2.Rodrigues(rightCameraRotationVector)

                #calculate rotation from the right camera coordinate system to the left camera coordinate system
                cameraRotationMatrix = numpy.dot(rightCameraRotationMatrix, leftCameraRotationMatrix.transpose())

                #calculate the translation from the right camera coordinate system to the left camera coordinate system
                cameraTranslationVectorMatrix = numpy.subtract(rightCameraTranslationVectorMatrix, numpy.dot(cameraRotationMatrix, leftCameraTranslationVectorMatrix))

                #save the parameters to file
                #numpy.savetxt(self.pathToImagePair + "leftCameraRotationMatrix.txt", leftCameraRotationMatrix)
                #numpy.savetxt(self.pathToImagePair + "rightCameraRotationMatrix.txt", rightCameraRotationMatrix)
                #numpy.savetxt(self.pathToImagePair + "leftCameraTranslationVectorMatrix.txt", leftCameraTranslationVectorMatrix)
                #numpy.savetxt(self.pathToImagePair + "rightCameraTranslationVectorMatrix.txt", rightCameraTranslationVectorMatrix)
                #numpy.savetxt(self.pathToImagePair + "rotationMatrix.txt", cameraRotationMatrix)
                #numpy.savetxt(self.pathToImagePair + "translationVectorMatrix.txt", cameraTranslationVectorMatrix)

            #end if

            else:
                errorFlag = True
                print("Error: Left and Right images must be of the same size.")

            #end else

        #end if

        else:
            errorFlag = True
            print("Error: Chessboards must be clearly visible and not too small or subpix window size may need to be reduced.")

        #end else

        #return values for rectification
        return (cameraRotationMatrix, cameraTranslationVectorMatrix, leftCameraRotationMatrix, leftCameraTranslationVectorMatrix, self.leftImage.colourCalibrationImage.shape[:2], errorFlag)

    #end calibrationWithChessboards

    def rectify(
        self,
        imageSize,
        cameraRotationMatrix,
        cameraTranslationVectorMatrix,
        alpha,
        calibrateForZeroDisparity,
        saveRectifiedImages,
        pathToRectifiedImages,
        imageScaleFactor):

        self.imageSize = imageSize

        #check for rectified images directory if required
        if saveRectifiedImages:
            try:
                os.makedirs(pathToRectifiedImages)
            except OSError as error:

                if error.errno == errno.EEXIST:
                    pass
                #end if
                else:
                    saveRectifiedImages = False
                    raise
                #end else
            #end except
        #end if

        #convert numpy arrays to old style CvMats becuase there is no new python wrapper for stereorectify :(
        cameraOneIntrinsicsMatrixMat = cv.fromarray(self.cameraOneIntrinsicsMatrix)
        cameraTwoIntrinsicsMatrixMat = cv.fromarray(self.cameraTwoIntrinsicsMatrix)
        cameraOneDistortionMatrixMat = cv.fromarray(self.cameraOneDistortionMatrix.reshape(1,-1))
        cameraTwoDistortionMatrixMat = cv.fromarray(self.cameraTwoDistortionMatrix.reshape(1,-1))
        cameraRotationMatrixMat = cv.fromarray(cameraRotationMatrix)
        cameraTranslationVectorMatrixMat = cv.fromarray(cameraTranslationVectorMatrix)

        #create output matricies
        leftRectificationMatrixMat = cv.CreateMat(3, 3, cv.CV_64FC1)
        rightRectificationMatrixMat = cv.CreateMat(3, 3, cv.CV_64FC1)
        leftProjectionMatrixMat = cv.CreateMat(3, 4, cv.CV_64FC1)
        rightProjectionMatrixMat = cv.CreateMat(3, 4, cv.CV_64FC1)
        disparityToDepthMappingMatrixMat = cv.CreateMat(4, 4, cv.CV_64FC1)

        #caclulate flags
        if calibrateForZeroDisparity:
            rectificationFlags = cv.CV_CALIB_ZERO_DISPARITY
        #end if
        else:
            rectificationFlags = 0
        #end else

        #calculate the rectification transforms, the function returns the left and right regions of interest
        self.leftImageRoi, self.rightImageRoi = cv.StereoRectify(
            cameraOneIntrinsicsMatrixMat,
            cameraTwoIntrinsicsMatrixMat,
            cameraOneDistortionMatrixMat,
            cameraTwoDistortionMatrixMat,
            self.imageSize,
            cameraRotationMatrixMat,
            cameraTranslationVectorMatrixMat,
            leftRectificationMatrixMat,
            rightRectificationMatrixMat,
            leftProjectionMatrixMat,
            rightProjectionMatrixMat,
            disparityToDepthMappingMatrixMat,
            rectificationFlags,
            alpha, #alpha = 1 ; all pixels are retained
            self.imageSize)

        self.leftRectificationMatrix = numpy.asarray(leftRectificationMatrixMat, dtype = numpy.float32)
        self.rightRectificationMatrix = numpy.asarray(rightRectificationMatrixMat, dtype = numpy.float32)
        self.leftProjectionMatrix = numpy.asarray(leftProjectionMatrixMat, dtype = numpy.float32)
        self.rightProjectionMatrix = numpy.asarray(rightProjectionMatrixMat, dtype = numpy.float32)
        self.disparityToDepthMappingMatrix = numpy.asarray(disparityToDepthMappingMatrixMat, dtype = numpy.float32)

        #calculate the left rectification maps
        self.leftMapOne, self.leftMapTwo = cv2.initUndistortRectifyMap(
            self.cameraOneIntrinsicsMatrix,
            self.cameraOneDistortionMatrix,
            self.leftRectificationMatrix,
            self.leftProjectionMatrix,
            (int(imageScaleFactor*float(self.imageSize[1])),int(imageScaleFactor*float(self.imageSize[0]))),
            cv.CV_32FC1)

        #calculate the right rectification maps
        self.rightMapOne, self.rightMapTwo = cv2.initUndistortRectifyMap(
            self.cameraOneIntrinsicsMatrix,
            self.cameraOneDistortionMatrix,
            self.rightRectificationMatrix,
            self.rightProjectionMatrix,
            (int(imageScaleFactor*float(self.imageSize[1])),int(imageScaleFactor*float(self.imageSize[0]))),
            cv.CV_32FC1)

        print("Rectifying Images ...\n")
        #rectify the images
        self.leftRectifiedImage = cv2.remap(self.orientedLeftImage, self.leftMapOne, self.leftMapTwo, cv2.INTER_LANCZOS4)
        self.rightRectifiedImage = cv2.remap(self.orientedRightImage, self.rightMapOne, self.rightMapTwo, cv2.INTER_LANCZOS4)

        #save the images if required
        if saveRectifiedImages:
            cv2.imwrite(pathToRectifiedImages + "Rectified_Left" + self.leftImageFilename, self.leftRectifiedImage)
            cv2.imwrite(pathToRectifiedImages + "Rectified_Right" + self.rightImageFilename, self.rightRectifiedImage)
        #end if

        return (self.leftRectificationMatrix, self.rightRectificationMatrix)
    #end rectify

    #function to calculate the disparity using a block matching algorithm
    def calculateDisparitySGBM(
        self,
        saveDisparityMaps,
        pathToDisparityMaps,
        useSobelPreFilter,
        preFilterCap,
        SADWindowSize,
        minDisparity,
        numberOfDisparities,
        uniquenessRatio,
        speckleWindowSize,
        speckleRange,
        P1,
        P2):

        #check for disparity maps directory if necessary
        if saveDisparityMaps:
            try:
                os.makedirs(pathToDisparityMaps)
            except OSError as error:
               if error.errno == errno.EEXIST:
                   pass
               #end if
               else:
                   saveDisparityMaps = False
                   raise
               #end else
           #end except
        #end if

        #used for filtering valid matches when creating point clouds
        self.minDisparity = minDisparity
        self.numberOfDisparities = numberOfDisparities

        #initialise the bmstate with basic parameters
        SGBM = cv2.StereoSGBM(
            minDisparity,
            numberOfDisparities,
            SADWindowSize,
            P1,
            P2,
            0,
            preFilterCap,
            uniquenessRatio,
            speckleWindowSize,
            speckleRange,
            False)

        print("Finding stereo correspondence (Semi Global Block Matching Method) ...\n")

        #left map
        #create left disparity map
        self.leftDisparityMapMat = SGBM.compute(self.leftRectifiedImage, self.rightRectifiedImage)
        #cv2.imwrite("lri.jpg", self.leftRectifiedImage)
        #cv2.imwrite("rri.jpg", self.rightRectifiedImage)
        #autoscale image range
        self.leftDisparityMap = numpy.float32(numpy.asarray(self.leftDisparityMapMat[:,:]))/-16.0
        lMinDisp, lMaxDisp, _, _ =  cv2.minMaxLoc(self.leftDisparityMap)
        self.leftDisparityMapDisplay = cv2.add(-self.leftDisparityMap, numpy.asarray([-minDisparity]))
        #avoid division by zero error
        if not (lMaxDisp - lMinDisp == 0):
            self.leftDisparityMapDisplay = self.leftDisparityMapDisplay*(256.0/(numberOfDisparities))
        #end if
        if(saveDisparityMaps):
            cv2.imwrite(pathToDisparityMaps + "Disparity_" + self.leftImageFilename, self.leftDisparityMapDisplay)
        #end if

        ##right map
        ##move disparity range
        #SGBM.minDisparity = -(minDisparity + numberOfDisparities)
        ##create left disparity map
        #self.rightDisparityMapMat = SGBM.compute(self.rightRectifiedImage, self.leftRectifiedImage)
        ##autoscale image range
        #self.rightDisparityMap = numpy.float32(numpy.asarray(self.rightDisparityMapMat[:,:]))/16.0
        #lMinDisp, lMaxDisp, lMinDispLoc, lMaxDispLoc =  cv2.minMaxLoc(self.rightDisparityMap)
        #self.rightDisparityMapDisplay = cv2.add(self.rightDisparityMap, numpy.asarray([-lMinDisp]))
        ##avoid division by zero error
        #if not (lMaxDisp - lMinDisp == 0):
            #self.rightDisparityMapDisplay = self.rightDisparityMapDisplay*(256.0/(lMaxDisp - lMinDisp))
        ##end if
        #if(saveDisparityMaps):
            #cv.SaveImage(pathToDisparityMaps + "Disparity_" + self.rightImageFilename, self.rightDisparityMapDisplay)
        ##end if

    #end calculateDisparityBM

    def createPointClouds(
    self,
    cameraRotationMatrix,
    cameraTranslationVectorMatrix):

        #notify user of progress
        print("Reprojecting to 3D ...\n")

        #left image
        #convert data to numpy arrays
        self.leftImageForProjection = numpy.asarray(self.leftRectifiedImage)
        self.leftImageForProjectionBGR = cv2.cvtColor(self.leftImageForProjection, cv.CV_RGB2BGR)

        #create homogeneous transformation matrix to move the points from camera frame of reference to object frame of reference
        #create homogeneous rotation matrix from the rectification
        #leftHomogeneousRectificationMatrix = numpy.identity(4)
        #leftHomogeneousRectificationMatrix[0:3,0:3] = self.leftRectificationMatrix.transpose()
        #print("Left camera rectification matrix:\n{0}".format(self.leftRectificationMatrix))
        #print("left homogeneous rectification matrix:\n{0}".format(leftHomogeneousRectificationMatrix))
        #create homogeneous rotation matrix
        leftHomogeneousRotationMatrix = numpy.identity(4)
        leftHomogeneousRotationMatrix[0:3,0:3] = cameraRotationMatrix
        #print("Left camera rotation matrix:\n{0}".format(self.leftCameraRotationMatrix))
        #print("Left homogeneous rotation matrix:\n{0}".format(leftHomogeneousRotationMatrix))
        #create homogeneous translation matrix
        leftHomogeneousTranslationMatrix = numpy.identity(4)
        leftHomogeneousTranslationMatrix[0:3,3] = cameraTranslationVectorMatrix[:,0]
        #print("Left camera translation matrix:\n{0}".format(self.leftCameraTranslationVectorMatrix))
        #print("Left homogeneous translation matrix:\n{0}".format(leftHomogeneousTranslationMatrix))
        leftHomogeneousCameraTransformationMatrix = numpy.dot(leftHomogeneousTranslationMatrix, leftHomogeneousRotationMatrix)
        #print("Homogeneous transformation matrix:\n{0}".format(homogeneousTransformationMatrixFromCameraFrameToObjectFrame))

        #calculate depth from disparity
        self.left3DImageCameraReference = cv2.reprojectImageTo3D(self.leftDisparityMap, self.disparityToDepthMappingMatrix, handleMissingValues = False, ddepth = cv.CV_32FC1)

        #reshape images into 2D arrays
        self.leftDisparity2DArray = self.leftDisparityMap.reshape((1, -1))
        self.leftRgbImage2DArray = self.leftImageForProjectionBGR.reshape((-1, 3)).transpose()
        self.leftXyzImage2DArray = self.left3DImageCameraReference.reshape((-1, 3)).transpose()

        #create a mask for the matched points by checking against the minimum disparity value
        self.leftSuccessfulMatchMask = numpy.ma.getmask(numpy.ma.masked_outside(self.leftDisparity2DArray, float(-self.minDisparity), float(-self.minDisparity - self.numberOfDisparities)))

        #transform to object reference frame
        #create homogeneous points
        self.leftHomogeneousXyzPoints = numpy.concatenate((self.leftXyzImage2DArray, numpy.ones((1, self.leftXyzImage2DArray.shape[1]), dtype = numpy.float32)), axis = 0)
        #multiply by transformation matrix
        self.leftTransformedXyzImage2DArray = numpy.dot(leftHomogeneousCameraTransformationMatrix, self.leftHomogeneousXyzPoints)

    #end createPointClouds

#end stereo