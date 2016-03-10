#!/usr/bin/env python2.6

# circlesGrid class
#
# Class for each calibration image with all the information for calibration
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

import cv2
import numpy

class circlesGridImage:
    """A class for the calbration images"""
    def __init__(
        self,
        verticalFormat,
        imagePath,
        imageFilename,
        numberOfPointsPerRow,
        numberOfPointsPerColumn,
        drawCentres,
        pathToImageswithCentres,
        imageWithCentresFilename):

        #so the filename and path can be used with other functions
        self.imagePath = imagePath
        self.imageFilename = imageFilename

        #load the image
        #colour
        self.unorientedColourCalibrationImage = cv2.imread(imagePath + imageFilename, 1)
        #greyscale
        self.unorientedGreyscaleCalibrationImage = cv2.imread(imagePath + imageFilename, 0)

        #orient the images
        if verticalFormat:
            self.colourCalibrationImage = cv2.transpose(self.unorientedColourCalibrationImage)
            self.greyscaleCalibrationImage = cv2.transpose(self.unorientedGreyscaleCalibrationImage)
            self.colourCalibrationImage = cv2.flip(self.colourCalibrationImage, 0)
            self.greyscaleCalibrationImage = cv2.flip(self.greyscaleCalibrationImage, 0)
        #end if
        else:
            self.colourCalibrationImage = numpy.copy(self.unorientedColourCalibrationImage)
            self.greyscaleCalibrationImage = numpy.copy(self.unorientedGreyscaleCalibrationImage)

        #find the centres of the circles in the circles grid
        self.centreLocations = cv2.findCirclesGridDefault(self.colourCalibrationImage, (numberOfPointsPerRow, numberOfPointsPerColumn), flags = cv2.CALIB_CB_ASYMMETRIC_GRID)

        #set success flag
        if (type(self.centreLocations) != None):
            self.success = True
        #end if
        else:
            self.success = False
        #end else

        #if centres are found, draw corners if required
        if self.success:
            if drawCentres:
                colourCalibrationImageCentres = numpy.copy(self.colourCalibrationImage)
                cv2.drawChessboardCorners(
                    colourCalibrationImageCentres,
                    (numberOfPointsPerRow, numberOfPointsPerColumn),
                    self.centreLocations,
                    self.success)
                #save image
                cv2.imwrite(pathToImageswithCentres + imageWithCentresFilename, colourCalibrationImageCentres)

            #end if
        #end if

    #end __init__

#end chessboardImage