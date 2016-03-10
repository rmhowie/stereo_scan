#!/usr/bin/env python2.6


# pointCloud class to deal with point clouds
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
import numpy

class xyzrgbPointCloudOriented:

    def __init__(
        self,
        rgbImage,
        xyzImage,
        validityMask):

            self.rgbImage = rgbImage
            self.xyzImage = xyzImage
            self.validityMask = validityMask

    #end __init__

    def writeUnstructured(
        self,
        pointCloudPath,
        pointCloudFilename):

            #duplicate data into local variables becuase it is faster
            validRgbImage2DArray = self.rgbImage
            validXyzImage2DArray = self.xyzImage
            validityMask = self.validityMask

            #write point cloud to disc
            with open(pointCloudPath + pointCloudFilename, "w") as pointCloudFile:
                pointCloudFile.write("# .PCD v.7 - Point Cloud Library Data File Format - Written By simpleStereo.py by Robert Howie\n")
                pointCloudFile.write("VERSION .7\n")
                pointCloudFile.write("FIELDS x y z rgb\n")
                pointCloudFile.write("SIZE 4 4 4 4\n")
                pointCloudFile.write("TYPE F F F U\n")
                pointCloudFile.write("COUNT 1 1 1 1\n")
                pointCloudFile.write("WIDTH {0}\n".format(validityMask.shape[1] - numpy.sum(validityMask)))
                pointCloudFile.write("HEIGHT 1\n")
                pointCloudFile.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                pointCloudFile.write("POINTS {0}\n".format(validityMask.shape[1] - numpy.sum(validityMask)))
                pointCloudFile.write("DATA ascii\n")
                #create data structure
                for i in range(validXyzImage2DArray.shape[1]):
                    if(validityMask[0,i] == False):
                        pointCloudFile.write("{0} {1} {2} {3}\n".format(validXyzImage2DArray[0,i], validXyzImage2DArray[1,i], validXyzImage2DArray[2,i], ((validRgbImage2DArray[0,i]*pow(2,16)) + (validRgbImage2DArray[1,i]*pow(2,8)) + (validRgbImage2DArray[2,i]*pow(2,0)))))
                #end for
            #end with

    #end writeUnstructured

#end class
