#!/usr/bin/env python2.6


# camera
#
# Finds the intrinsic and distortion parameters of a lens and camera combination
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
import os #used for directory listing
import fnmatch # used for filename matching
import cv #OpenCV library
import cv2
import errno #Error numbers module
import numpy
#import pdb #Python debugger

#import non standard modules
import circlesGridImage


class camera:
    "A class to hold the information relevant to a single camera and lens combination"
    def __init__(
        self,
        verticalFormat,
        pathToCalibrationImages,
        calibrationImages,
        numberOfPointsPerRow,
        numberOfPointsPerColumn,
        circleSpacingSize,
        drawCentres,
        pathToCalibrationImagesWithCentresDrawn):

        #check for images with centres drawn directory if required
        if drawCentres:
            try:
                os.makedirs(pathToCalibrationImagesWithCentresDrawn)
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

        imageFileNamesList = []
        for file in os.listdir(pathToCalibrationImages):
            if fnmatch.fnmatch(file, calibrationImages):
                imageFileNamesList.append(file)
            #end if
        #end for

        #calculate the number of chessboard points
        numberOfCirclesGridPoints = numberOfPointsPerRow * numberOfPointsPerColumn

        #initialise the number of succeful images to zero
        self.numberOfSuccessfulViews = 0

        #create a list for the centre locations
        centreLocationsList = []

        #initialise these varibales to prevent a referenced before assignment error
        previousImageSize = 0
        imageSize = 0

        #sort the image file names list into ascending order
        imageFileNamesList.sort()

        #for each filename
        for i in range(len(imageFileNamesList)):
            print("Processing ", imageFileNamesList[i], " (image ", i+1, " of ", len(imageFileNamesList), ") ...", sep="", end=" ")
            calibrationImage = circlesGridImage.circlesGridImage(
                verticalFormat,
                pathToCalibrationImages,
                imageFileNamesList[i],
                numberOfPointsPerRow,
                numberOfPointsPerColumn,
                drawCentres,
                pathToCalibrationImagesWithCentresDrawn,
                "Centres_" + imageFileNamesList[i])
            #find the image size
            imageSize = calibrationImage.colourCalibrationImage.shape[:2]
            #add to successful list if centre finding worked
            if calibrationImage.success:
                #notify the user of succeful centre search
                print("Centres found")
                #check that image sizes are the same
                if ((i == 0) or (previousImageSize == imageSize)):
                    #increment the number of successful views by one
                    self.numberOfSuccessfulViews += 1
                    centreLocationsList.append(calibrationImage.centreLocations.reshape(-1,2))
                #end if
            #end if
            else:
                print("Error: images must be the same size. Check the size of image ", imageFileNamesList[i], ".", sep="")
            #end else
            #store previous image size
            previousImageSize = imageSize
        #end for

        #continue only if we had more than 10 good images
        if (self.numberOfSuccessfulViews >= 10):
            #create matricies for calibration parameters
            #input matricies
            objectPoints = numpy.zeros((self.numberOfSuccessfulViews, numberOfCirclesGridPoints, 3), dtype=numpy.float32)
            imagePoints = numpy.zeros((self.numberOfSuccessfulViews, numberOfCirclesGridPoints, 2), dtype=numpy.float32)
            imageSize = calibrationImage.colourCalibrationImage.shape[:2] #NOTE: images must all be the same size
            #output matricies
            self.intrinsicMatrix = numpy.zeros((3, 3), dtype = numpy.float64)
            self.distortionMatrix = numpy.zeros((1,5), dtype = numpy.float64)

            #image points
            for i in range(self.numberOfSuccessfulViews):
                for j in range(numberOfCirclesGridPoints):
                    imagePoints[i][j][0] = centreLocationsList[i][j][0]
                    imagePoints[i][j][1] = centreLocationsList[i][j][1]
                #end for
            #end for

            #set parameter matricies
            #object points
            for i in range(self.numberOfSuccessfulViews):
                for j in range(numberOfPointsPerColumn):
                    for k in range(numberOfPointsPerRow):
                        objectPoints[i][j*numberOfPointsPerRow + k][0] = float(j)*float(circleSpacingSize)
                        objectPoints[i][j*numberOfPointsPerRow + k][1] = float(j%2 + 2*k)*float(circleSpacingSize)
                        objectPoints[i][j*numberOfPointsPerRow + k][2] = 0.0
                    #end for
                #end for
            #end for

            #pdb.set_trace()

            #calibrate the camera
            retval, self.intrinsicsMatrix, self.distortionMatrix, rotationVectorMatrix, translationVectorMatrix = cv2.calibrateCamera(
                objectPoints,
                imagePoints,
                imageSize,
                self.intrinsicMatrix,
                self.distortionMatrix,
                flags = cv.CV_CALIB_FIX_K3)

            #undistort an image to test
            #undistIm = cv2.undistort(calibrationImage.colourCalibrationImage, self.intrinsicMatrix, self.distortionMatrix)

            #cv2.imwrite("undistIm.JPG", undistIm)

            #save the parameters to disk
            numpy.savetxt(pathToCalibrationImages + "intrinsicMatrix.txt", self.intrinsicMatrix)
            numpy.savetxt(pathToCalibrationImages + "distortionMatrix.txt", self.distortionMatrix)

            #notify user
            print("\nIntrinsic and distortion parameters calculated from", self.numberOfSuccessfulViews, "images.\n")
        #end if

        else:
            print("\nFailed to find centres in 10 or more images. Take more images with the chessboard clearly visible. Calibration images should be in images/calibration\"\n")
        #end else

    # end __init__

#end camera