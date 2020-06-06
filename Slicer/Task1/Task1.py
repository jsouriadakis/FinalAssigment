import os
import unittest
import vtk, qt, ctk, slicer
from slicer.ScriptedLoadableModule import *
import logging
import numpy as np
import time
import math
import Algorithms
import MathTools


# import sympy
# from sympy import Line3D
#
# Task1
#

class Task1(ScriptedLoadableModule):
    """Uses ScriptedLoadableModule base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def __init__(self, parent):
        ScriptedLoadableModule.__init__(self, parent)
        self.parent.title = "Task1"  # TODO make this more human readable by adding spaces
        self.parent.categories = ["Examples"]
        self.parent.dependencies = []
        self.parent.contributors = ["Ioannis Souriadakis"]  # replace with "Firstname Lastname (Organization)"
        self.parent.helpText = """
This is an example of scripted loadable module bundled in an extension.
It performs a simple thresholding on the input volume and optionally captures a screenshot.
"""
        self.parent.helpText += self.getDefaultModuleDocumentationLink()
        self.parent.acknowledgementText = """
This file was originally developed by Jean-Christophe Fillion-Robin, Kitware Inc.
and Steve Pieper, Isomics, Inc. and was partially funded by NIH grant 3P41RR013218-12S1.
"""  # replace with organization, grant and thanks.


#
# Task1Widget
#

class Task1Widget(ScriptedLoadableModuleWidget):
    """Uses ScriptedLoadableModuleWidget base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def setup(self):
        ScriptedLoadableModuleWidget.setup(self)

        # Instantiate and connect widgets ...

        #
        # Parameters Area
        #
        parametersCollapsibleButton = ctk.ctkCollapsibleButton()
        parametersCollapsibleButton.text = "Parameters"
        self.layout.addWidget(parametersCollapsibleButton)

        # Layout within the dummy collapsible button
        parametersFormLayout = qt.QFormLayout(parametersCollapsibleButton)

        #
        # input volume selector
        #
        self.hippoSelector = slicer.qMRMLNodeComboBox()
        self.hippoSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.hippoSelector.selectNodeUponCreation = True
        self.hippoSelector.addEnabled = False
        self.hippoSelector.removeEnabled = False
        self.hippoSelector.noneEnabled = False
        self.hippoSelector.showHidden = False
        self.hippoSelector.showChildNodeTypes = False
        self.hippoSelector.setMRMLScene(slicer.mrmlScene)
        self.hippoSelector.setToolTip("Pick the target object.")
        parametersFormLayout.addRow("Input Hippocampus Label Volume: ", self.hippoSelector)

        self.ventSelector = slicer.qMRMLNodeComboBox()
        self.ventSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.ventSelector.selectNodeUponCreation = True
        self.ventSelector.addEnabled = False
        self.ventSelector.removeEnabled = False
        self.ventSelector.noneEnabled = False
        self.ventSelector.showHidden = False
        self.ventSelector.showChildNodeTypes = False
        self.ventSelector.setMRMLScene(slicer.mrmlScene)
        self.ventSelector.setToolTip("Pick the blood vessels dilated object.")
        parametersFormLayout.addRow("Input Blood Vessels Dilated Label Volume: ", self.ventSelector)

        self.vesselSelector = slicer.qMRMLNodeComboBox()
        self.vesselSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.vesselSelector.selectNodeUponCreation = True
        self.vesselSelector.addEnabled = False
        self.vesselSelector.removeEnabled = False
        self.vesselSelector.noneEnabled = False
        self.vesselSelector.showHidden = False
        self.vesselSelector.showChildNodeTypes = False
        self.vesselSelector.setMRMLScene(slicer.mrmlScene)
        self.vesselSelector.setToolTip("Pick the blood vessel object.")
        parametersFormLayout.addRow("Input Blood Vessels Label Volume: ", self.vesselSelector)

        self.cortexSelector = slicer.qMRMLNodeComboBox()
        self.cortexSelector.nodeTypes = ["vtkMRMLLabelMapVolumeNode"]
        self.cortexSelector.selectNodeUponCreation = True
        self.cortexSelector.addEnabled = False
        self.cortexSelector.removeEnabled = False
        self.cortexSelector.noneEnabled = False
        self.cortexSelector.showHidden = False
        self.cortexSelector.showChildNodeTypes = False
        self.cortexSelector.setMRMLScene(slicer.mrmlScene)
        self.cortexSelector.setToolTip("Pick the cortex object.")
        parametersFormLayout.addRow("Input cortex Label Volume: ", self.cortexSelector)

        # creating the ui dropdowns will let you generalize to other nodes
        self.inputTargetFiducialSelector = slicer.qMRMLNodeComboBox()
        self.inputTargetFiducialSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.inputTargetFiducialSelector.selectNodeUponCreation = True
        self.inputTargetFiducialSelector.addEnabled = False
        self.inputTargetFiducialSelector.removeEnabled = False
        self.inputTargetFiducialSelector.noneEnabled = False
        self.inputTargetFiducialSelector.showHidden = False
        self.inputTargetFiducialSelector.showChildNodeTypes = False
        self.inputTargetFiducialSelector.setMRMLScene(slicer.mrmlScene)
        self.inputTargetFiducialSelector.setToolTip("Pick the input fiducials to the algorithm.")
        parametersFormLayout.addRow("Input Target Fiducials: ", self.inputTargetFiducialSelector)

        self.inputEntryFiducialSelector = slicer.qMRMLNodeComboBox()
        self.inputEntryFiducialSelector.nodeTypes = ["vtkMRMLMarkupsFiducialNode"]
        self.inputEntryFiducialSelector.selectNodeUponCreation = True
        self.inputEntryFiducialSelector.addEnabled = False
        self.inputEntryFiducialSelector.removeEnabled = False
        self.inputEntryFiducialSelector.noneEnabled = False
        self.inputEntryFiducialSelector.showHidden = False
        self.inputEntryFiducialSelector.showChildNodeTypes = False
        self.inputEntryFiducialSelector.setMRMLScene(slicer.mrmlScene)
        self.inputEntryFiducialSelector.setToolTip("Pick the input fiducials to the algorithm.")
        parametersFormLayout.addRow("Input Entry Fiducials: ", self.inputEntryFiducialSelector)

        self.validAngleSlider = ctk.ctkSliderWidget()
        self.validAngleSlider.singleStep = 1
        self.validAngleSlider.minimum = 0
        self.validAngleSlider.maximum = 90
        self.validAngleSlider.value = 55
        self.validAngleSlider.setToolTip("Set angle value for valid incision")
        parametersFormLayout.addRow("Valid Incision Angle", self.validAngleSlider)

        self.precisionSlider = ctk.ctkSliderWidget()
        self.precisionSlider.singleStep = 0.001
        self.precisionSlider.minimum = 0.001
        self.precisionSlider.maximum = 0.1
        self.precisionSlider.value = 0.01
        self.precisionSlider.setToolTip("Set precision value for maximising distance to the critical structure")
        parametersFormLayout.addRow("Precision", self.precisionSlider)

        self.maximumIncisionLengthSlider = ctk.ctkSliderWidget()
        self.maximumIncisionLengthSlider.singleStep = 1
        self.maximumIncisionLengthSlider.minimum = 0.00
        self.maximumIncisionLengthSlider.maximum = 9999999999999
        self.maximumIncisionLengthSlider.value = 9999999999999
        self.maximumIncisionLengthSlider.setToolTip("Set maximum incision value")
        parametersFormLayout.addRow("Maximum Incision Length", self.maximumIncisionLengthSlider)
        #

        #
        # Apply Button
        #
        self.applyButton = qt.QPushButton("Apply")
        self.applyButton.toolTip = "Run the algorithm."
        self.applyButton.enabled = False
        parametersFormLayout.addRow(self.applyButton)

        # connections
        self.applyButton.connect('clicked(bool)', self.onApplyButton)
        self.inputEntryFiducialSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)
        self.inputTargetFiducialSelector.connect("currentNodeChanged(vtkMRMLNode*)", self.onSelect)

        # Add vertical spacer
        self.layout.addStretch(1)

        # Refresh Apply button state
        self.onSelect()

    def cleanup(self):
        pass

    def onSelect(self):
        self.applyButton.enabled = self.hippoSelector.currentNode() and self.ventSelector.currentNode() and self.vesselSelector.currentNode() and self.cortexSelector.currentNode() and self.inputTargetFiducialSelector.currentNode() and self.inputEntryFiducialSelector.currentNode()

    def onApplyButton(self):
        logic = Task1Logic()
        validAngleOfIntersection = self.validAngleSlider.value
        precisionValue = self.precisionSlider.value
        maximumIncisionValue = self.maximumIncisionLengthSlider.value
        logic.run(self.hippoSelector.currentNode(), self.ventSelector.currentNode(), self.vesselSelector.currentNode(),
                  self.cortexSelector.currentNode(),
                  self.inputTargetFiducialSelector.currentNode(),
                  self.inputEntryFiducialSelector.currentNode(),
                  validAngleOfIntersection, precisionValue, maximumIncisionValue)


#
# Task1Logic
#

class Task1Logic(ScriptedLoadableModuleLogic):
    """This class should implement all the actual
    computation done by your module.  The interface
    should be such that other python code can import
    this class and make use of the functionality without
    requiring an instance of the Widget.
    Uses ScriptedLoadableModuleLogic base class, available at:
    https://github.com/Slicer/Slicer/blob/master/Base/Python/slicer/ScriptedLoadableModule.py
    """

    def isValidInputOutputData(self, hippo, ventricles, vessels, cortex, targetsNode, entriesNode, validAngleOfIntersection, precisionValue, maximumIncisionValue):
        """Validates if the output is not the same as input
        """
        if not hippo:
            logging.debug('No hippo data selected')
            return False
        if not ventricles:
            logging.debug('No blood vessels dilated data selected')
            return False
        if not vessels:
            logging.debug('No blood vessel data selected')
            return False
        if not cortex:
            logging.debug('No cortex data selected')
            return False
        if not targetsNode:
            logging.debug('No etarget point data selected')
            return False
        if not entriesNode:
            logging.debug('No entry point data selected')
            return False
        if not validAngleOfIntersection:
            logging.debug('No valid angle selected')
            return False
        if not precisionValue:
            logging.debug('No precision value selected')
            return False
        if not maximumIncisionValue:
            logging.debug('No incision length value selected')
            return False
        return True

    def run(self, hippo, ventricles, vessels, cortex, targetsNode, entriesNode, validAngleOfIntersection, precisionValue, maximumIncisionValue):
        """
        Run the actual algorithm
        """
        if not self.isValidInputOutputData(hippo, ventricles, vessels, cortex, targetsNode, entriesNode, validAngleOfIntersection, precisionValue, maximumIncisionValue):
            slicer.util.errorDisplay('Invalid input.')
            return False

        logging.info('Processing started')

        entriesAndTargets = {}
        combinedEntriesAndTargets = {}
        newTargets = []
        bestTrajectories = {}

        startTime = time.time()
        newTargets = Algorithms.getValidTargets(targetsNode, hippo)
        endTime = time.time()
        print('Hippo targets: ', endTime - startTime, 'seconds')

        entriesAndTargets = Algorithms.entriesAndTargetsInDict(entriesNode, Algorithms.convertMarkupNodeToPoints(targetsNode))

        startTime = time.time()
        Algorithms.getIncisionsWithValidLength(entriesAndTargets, maximumIncisionValue)
        endTime = time.time()
        print('Incisions - Valid Length: ', endTime - startTime, 'seconds')

        startTime = time.time()
        Algorithms.getIncisionsWithValidArea(entriesAndTargets, ventricles)
        endTime = time.time()
        print('Incisions - Dilated Blood Vessels: ', endTime - startTime, 'seconds')

        startTime = time.time()
        Algorithms.getIncisionsWithValidArea(entriesAndTargets, vessels)
        endTime = time.time()
        print('Incisions - Blood Vessels: ', endTime - startTime, 'seconds')

        startTime = time.time()
        Algorithms.getIncisionsWithValidAngle(entriesAndTargets, cortex, validAngleOfIntersection)
        endTime = time.time()
        print('Incisions - Valid Angle: ', endTime - startTime, 'seconds')

        startTime = time.time()
        newTargets = Algorithms.getValidTargets(targetsNode, hippo)
        combinedEntriesAndTargets = Algorithms.entriesAndTargetsInDict(entriesNode, newTargets)
        combinedEntriesAndTargets = Algorithms.combineConstraints(combinedEntriesAndTargets, ventricles, vessels, cortex, validAngleOfIntersection, maximumIncisionValue)
        endTime = time.time()
        print('Incisions - Combined: ', endTime - startTime, 'seconds')

        # good to have some way of seeing our results
        # allPaths = self.printEntryAndTargetsInDict(combinedEntriesAndTargets)
        # pathNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'GoodPaths')
        # pathNode.SetAndObserveMesh(allPaths)

        bestTrajectiries = Algorithms.printBestTrajectoryForEachEntry(precisionValue, maximumIncisionValue, combinedEntriesAndTargets, vessels, ventricles)
        print(bestTrajectiries)
        # you can add something here to output the good entry target pairs
        logging.info('Processing completed')
        return True

    def registerToSlicer(self, bestAndWorstPair):
        maxTrajectory = bestAndWorstPair[0][1]
        self.registerEntryTargetPair(maxTrajectory)

        # minTrajectory = PointUtils.convertEntryTargetPairToVtkObject(finalTrajectories[1][1])

    def registerEntryTargetPair(self, entryTargetPair):
        self.createAndRegisterFiducial(entryTargetPair[0], 'Entry')
        self.createAndRegisterFiducial(entryTargetPair[1], 'Target')

    def createAndRegisterFiducial(self, coordinates, label):
        fudicualNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLMarkupsFiducialNode', label)
        fudicualNode.AddFiducialFromArray(coordinates)




    # Intersection Line first Try
    # def pointClashVentricles(self, entry, target, ventricles):
    #   xEntry, yEntry, zEntry = entry[0], entry[1], entry[2]
    #   xTarget, yTarget, zTarget = target[0], target[1], target[2]
    #   subSampling = 4
    #   step = 0.01
    #   for t in np.arange(0,1,step):
    #     x = xEntry + t*(xTarget - xEntry)
    #     y = yEntry + t*(yTarget - yEntry)
    #     z = zEntry + t*(zTarget - zEntry)
    #     #imageVoxelID = x*y*z
    #     imageVoxelID = ventricles.GetImageData().FindPoint(x/subSampling,y/subSampling,z/subSampling)
    #     xIndex = int(imageVoxelID%x)
    #     yIndex = int(imageVoxelID%(x*y)/x)
    #     zIndex = int((imageVoxelID/(x*y)))

    #     if ventricles.GetImageData().GetScalarComponentAsDouble(xIndex,yIndex,zIndex,0) == 1.0:
    #       return True
    #   return False

    # Intersection Line first Try
    # def pointsClassVessels(self,entry,target,vessels):
    #   xEntry, yEntry, zEntry = entry[0], entry[1], entry[2]
    #   xTarget, yTarget, zTarget = target[0], target[1], target[2]
    #   subSampling = 4
    #   step = 0.0001
    #   for t in np.arange(0,1,step):
    #     x = xEntry + t*(xTarget - xEntry)
    #     y = yEntry + t*(yTarget - yEntry)
    #     z = zEntry + t*(zTarget - zEntry)
    #     #imageVoxelID = x*y*z
    #     imageVoxelID = vessels.GetImageData().FindPoint(x/subSampling,y/subSampling,z/subSampling)
    #     xIndex = int(imageVoxelID%x)
    #     yIndex = int(imageVoxelID%(x*y)/x)
    #     zIndex = int((imageVoxelID/(x*y)))

    #     if vessels.GetImageData().GetScalarComponentAsDouble(xIndex,yIndex,zIndex,0) == 1.0:
    #       return True
    #   return False


class Task1Test(ScriptedLoadableModuleTest):


    def __init__(self):
        self.DIRECTORY = "C:\Users\PWW\OneDrive - King's College London\Medical Robotics Software Integration\Week2\Tutorial\Assigment1\Assigment1"
        self.TARGETS_NODE_LABEL = "targets"
        self.ENTRIES_NODE_LABEL = "entries"
        self.CORTEX_NODE_LABEL = "CortexLabelMap"
        self.VESSEL_NODE_LABEL = "VesselsLabelMap"
        self.HIPPOCAMPUS_NODE_LABEL = "HippoLabelMap"
        self.VESSEL_DILATE_NODE_LABEL = "VesselsDilatedLabelMap"

    def setUp(self):
        slicer.mrmlScene.Clear(0)

    def runTest(self):
        self.setUp()
        self.testLoadAllData(self.DIRECTORY)
        self.testGetFilteredHippocampusValidTargets()
        self.testGetFilteredHippocampusInvalidTargets()
        self.testFilterTrajectoriesBelowDistanceThreshold()
        self.testFilterTrajectoriesAboveDistanceThreshold()
        self.testAvoidBloodVesselsDilateValidPath()
        self.testAvoidBloodVesselsDilateInvalidPath()
        self.testAvoidBloodVesselsValidPath()
        self.testAvoidBloodVesselsInvalidPath()
        self.testAngleValidPath()
        self.testAngleInvalidPath()
        # self.testUnnecessaryTests()
        self.delayDisplay('Finished testing')
        self.setUp()  # to reclear data

    def testLoadAllData(self, directory):
        self.delayDisplay("Starting load data test")
        self.testLoadLabel(directory + '/HippoLabelMap.nrrd')
        self.testLoadLabel(directory + '/VesselsLabelMap.nrrd')
        self.testLoadLabel(directory + '/VesselsDilatedLabelMap.nrrd')
        self.testLoadLabel(directory + '/CortexLabelMap.nrrd')
        self.testLoadFiducial(directory + '/targets.fcsv')
        self.testLoadFiducial(directory + '/entries.fcsv')
        self.delayDisplay('testLoadAllData passed!')

    def testLoadFiducial(self, path):
        self.assertTrue(slicer.util.loadMarkupsFiducialList(path))

    def testLoadLabel(self, path):
        self.assertTrue(slicer.util.loadLabelVolume(path))

    # They don't really test anything. Mostly used for sanity checking
    # Values rejected for count rejected tests might need some adjustments
    def testUnnecessaryTests(self):
        self.testCountRejectedTrajectories(True)  # slow test
        self.testAllTogether()  # slow test

    # Slow test
    # This is just to make sure all constraints run together. It doesn't actually validate the results
    def testAllTogether(self):
        hippocampus = self.getNode(self.HIPPOCAMPUS_NODE_LABEL)
        bloodVesselsDilate = self.getNode(self.VESSEL_DILATE_NODE_LABEL)
        bloodVessels = self.getNode(self.VESSEL_NODE_LABEL)
        cortex = self.getNode(self.CORTEX_NODE_LABEL)
        entries = self.getNode(self.ENTRIES_NODE_LABEL)
        targets = self.getNode(self.TARGETS_NODE_LABEL)
        angle = 55
        distance = 9999
        newTargets = Algorithms.getValidTargets(targets, hippocampus)
        entriesAndTargets = Algorithms.entriesAndTargetsInDict(entries, newTargets)
        trajectoriesForAllHardConstraints = Algorithms.applyAllHardConstraints(entriesAndTargets,
                                                                                     bloodVesselsDilate,
                                                                                     bloodVessels, cortex,
                                                                                     angle,distance)
        finalTrajectories = Algorithms.getBestAndWorstTrajectory(trajectoriesForAllHardConstraints,  0.01, bloodVessels,
                                                                       bloodVesselsDilate)
        logic = Task1Logic()
        logic.registerToSlicer(finalTrajectories)
        self.delayDisplay('testAllTogether passed!')

    def testGetFilteredHippocampusValidTargets(self):
        hippocampus = self.getNode(self.HIPPOCAMPUS_NODE_LABEL)
        targets = self.getNode(self.TARGETS_NODE_LABEL)
        filteredTargets = Algorithms.getValidTargets(targets, hippocampus)
        self.assertTrue(targets.GetNumberOfMarkups() > len(filteredTargets))
        self.delayDisplay('testGetFilteredHippocampusValidTargets passed!')

    def testGetFilteredHippocampusInvalidTargets(self):
        hippocampus = self.getNode(self.HIPPOCAMPUS_NODE_LABEL)
        coordinates = [0, 1, 2]
        targets = slicer.vtkMRMLMarkupsFiducialNode()
        targets.AddFiducialFromArray(coordinates)
        filteredTargets = Algorithms.getValidTargets(targets, hippocampus)
        self.assertTrue(len(filteredTargets) == 0)
        self.delayDisplay('testGetFilteredHippocampusInvalidTargets passed!')

    def testFilterTrajectoriesBelowDistanceThreshold(self):
        entriesAndTargets = {(196.989, 131.913, 32.491): [[150.0, 128.0, 114.0]]}
        threshold = 9999
        result = Algorithms.getIncisionsWithValidLength(entriesAndTargets, threshold)
        self.assertTrue(len(result) > 0)
        self.delayDisplay('testFilterTrajectoriesBelowDistanceThreshold passed!')

    def testFilterTrajectoriesAboveDistanceThreshold(self):
        entriesAndTargets = {(196.989, 131.913, 32.491): [[150.0, 128.0, 114.0]]}
        threshold = 0
        result = Algorithms.getIncisionsWithValidLength(entriesAndTargets, threshold)
        self.assertTrue(len(result) == 0)
        self.delayDisplay('testFilterTrajectoriesAboveDistanceThreshold passed!')

    def testAvoidBloodVesselsDilateValidPath(self):
        bloodVesselsDilate = self.getNode(self.VESSEL_DILATE_NODE_LABEL)
        entriesAndTargets = {(196.989, 131.913, 32.491): [[150.0, 128.0, 114.0]]}
        result = Algorithms.getIncisionsWithValidArea(entriesAndTargets, bloodVesselsDilate)
        self.assertTrue(len(result) > 0)
        self.delayDisplay('testAvoidBloodVesselsDilateValidPath passed!')

    def testAvoidBloodVesselsDilateInvalidPath(self):
        bloodVesselsDilate = self.getNode(self.VESSEL_DILATE_NODE_LABEL)
        entriesAndTargets = {(205.327, 107.823, 58.771): [[158.0, 133.0, 82.0]]}
        result = Algorithms.getIncisionsWithValidArea(entriesAndTargets, bloodVesselsDilate)
        self.assertTrue(len(result) == 0)
        self.delayDisplay('testAvoidBloodVesselsDilateInvalidPath passed!')

    def testAvoidBloodVesselsValidPath(self):
        bloodVessels = self.getNode(self.VESSEL_NODE_LABEL)
        entriesAndTargets = {(196.989, 131.913, 32.491): [[150.0, 128.0, 114.0]]}
        result = Algorithms.getIncisionsWithValidArea(entriesAndTargets, bloodVessels)
        self.assertTrue(len(result) > 0)
        self.delayDisplay('testAvoidBloodVesselsValidPath passed!')

    def testAvoidBloodVesselsInvalidPath(self):
        bloodVessels = self.getNode(self.VESSEL_NODE_LABEL)
        entriesAndTargets = {(212.09, 147.385, 76.878): [[158.0, 133.0, 82.0]]}
        result = Algorithms.getIncisionsWithValidArea(entriesAndTargets, bloodVessels)
        self.assertTrue(len(result) == 0)
        self.delayDisplay('testAvoidBloodVesselsInvalidPath passed!')

    def testAngleValidPath(self):
        cortex = self.getNode(self.CORTEX_NODE_LABEL)
        entriesAndTargets = {(196.989, 131.913, 32.491): [[150.0, 128.0, 114.0]]}
        result = Algorithms.getIncisionsWithValidAngle(entriesAndTargets, cortex, 55)
        self.assertTrue(len(result) > 0)
        self.delayDisplay('testAngleValidPath passed!')

    def testAngleInvalidPath(self):
        cortex = self.getNode(self.CORTEX_NODE_LABEL)
        entriesAndTargets = {(208.654, 134.777, 61.762): [[162.0, 128.0, 106.0]]}
        result = Algorithms.getIncisionsWithValidAngle(entriesAndTargets, cortex, 55)
        self.assertTrue(len(result) == 0)
        self.delayDisplay('testAngleInvalidPath passed!')

    # Slow test
    def testCountRejectedTrajectories(self, printOutput):
        hippocampus = self.getNode(self.HIPPOCAMPUS_NODE_LABEL)
        bloodVesselsDilate = self.getNode(self.VESSEL_DILATE_NODE_LABEL)
        bloodVessels = self.getNode(self.VESSEL_NODE_LABEL)
        cortex = self.getNode(self.CORTEX_NODE_LABEL)
        entries = self.getNode(self.ENTRIES_NODE_LABEL)
        targets = self.getNode(self.TARGETS_NODE_LABEL)
        angle = 55
        distanceThreshold = 60
        total = entries.GetNumberOfMarkups() * targets.GetNumberOfMarkups()
        if printOutput:
            print("Total: ", total)
        self.testCountRejectedTrajectoriesForHippocampus(entries, targets, hippocampus, total, printOutput)
        self.testCountRejectedByHardConstraints(entries, targets, hippocampus, bloodVesselsDilate, bloodVessels, cortex,
                                                angle, distanceThreshold, total, printOutput)
        self.delayDisplay('testCountRejectedTrajectories passed!')

    def testCountRejectedTrajectoriesForHippocampus(self, entries, targets, hippocampus, total, printOutput):
        startTime = time.time()
        filteredTargets = Algorithms.getValidTargets(targets, hippocampus)
        endTime = time.time()
        totalTrajectoriesOnHippocampus = entries.GetNumberOfMarkups() * len(filteredTargets)
        if printOutput:
            print('Filtering hippocampus total time: ', endTime - startTime, 'seconds')
            print("Total accepted filtering hippocampus: ", totalTrajectoriesOnHippocampus)
            print("Total rejected filtering hippocampus: ", total - totalTrajectoriesOnHippocampus)
        self.assertTrue(total - totalTrajectoriesOnHippocampus == 83520)
        self.delayDisplay('testCountRejectedTrajectoriesForHippocampus passed!')

    def testCountRejectedByHardConstraints(self, entries, targets, hippocampus, bloodVesselsDilate, bloodVessels,
                                           cortex,
                                           angle, distanceThreshold, total, printOutput):
        entriesAndTargets = Algorithms.entriesAndTargetsInDict(entries, Algorithms.convertMarkupNodeToPoints(targets))
        self.testCountRejectedTrajectoriesForDistanceThreshold(entriesAndTargets, distanceThreshold, total,
                                                                printOutput)
        self.testCountRejectedTrajectoriesForBloodVesselsDilate(entriesAndTargets, bloodVesselsDilate, total,
                                                                printOutput)
        self.testCountRejectedTrajectoriesForBloodVessels(entriesAndTargets, bloodVessels, total, printOutput)
        self.testCountRejectedTrajectoriesForAngle(entriesAndTargets, cortex, angle, total, printOutput)
        self.testCountRejectedTrajectoriesCombiningAllHard(entries, targets, hippocampus, bloodVesselsDilate,
                                                           bloodVessels,
                                                           cortex,
                                                           angle, total, printOutput)

    def testCountRejectedTrajectoriesForDistanceThreshold(self, entriesAndTargets, distanceThreshold, total, printOutput):
        startTime = time.time()
        filteredForDistance = Algorithms.getIncisionsWithValidLength(entriesAndTargets, distanceThreshold)
        endTime = time.time()
        totalTrajectoriesFilteredForDistance = 0
        for _, targetValues in filteredForDistance.items():
            totalTrajectoriesFilteredForDistance += len(targetValues)
        if printOutput:
            print('Filtering for distance total time: ', endTime - startTime, 'seconds')
            print("Total accepted filtering distance: ", totalTrajectoriesFilteredForDistance)
            print("Total rejected filtering distance: ", total - totalTrajectoriesFilteredForDistance)
        self.assertTrue(total - totalTrajectoriesFilteredForDistance == 65395)
        self.delayDisplay('testCountRejectedTrajectoriesForDistanceThreshold passed!')

    def testCountRejectedTrajectoriesForBloodVesselsDilate(self, entriesAndTargets, bloodVesselsDilate, total,
                                                           printOutput):
        startTime = time.time()
        filteredForBloodVessels = Algorithms.getIncisionsWithValidArea(entriesAndTargets, bloodVesselsDilate)
        endTime = time.time()
        totalTrajectoriesFilteringBloodVesselsDilate = 0
        for _, targetValues in filteredForBloodVessels.items():
            totalTrajectoriesFilteringBloodVesselsDilate += len(targetValues)
        if printOutput:
            print('Filtering for blood vessels dilate total time: ', endTime - startTime, 'seconds')
            print("Total accepted filtering blood vessels dilate: ", totalTrajectoriesFilteringBloodVesselsDilate)
            print("Total rejected filtering blood vessels dilate: ",
                  total - totalTrajectoriesFilteringBloodVesselsDilate)
        self.assertTrue(total - totalTrajectoriesFilteringBloodVesselsDilate == 9033)
        self.delayDisplay('testCountRejectedTrajectoriesForBloodVesselsDilate passed!')

    def testCountRejectedTrajectoriesForBloodVessels(self, entriesAndTargets, bloodVessels, total, printOutput):
        startTime = time.time()
        filteredForBloodVessels = Algorithms.getIncisionsWithValidArea(entriesAndTargets, bloodVessels)
        endTime = time.time()
        totalTrajectoriesFilteringBloodVessels = 0
        for _, targetValues in filteredForBloodVessels.items():
            totalTrajectoriesFilteringBloodVessels += len(targetValues)
        if printOutput:
            print('Filtering for blood vessels total time: ', endTime - startTime, 'seconds')
            print("Total accepted filtering blood vessels: ", totalTrajectoriesFilteringBloodVessels)
            print("Total rejected filtering blood vessels: ", total - totalTrajectoriesFilteringBloodVessels)
        self.assertTrue(total - totalTrajectoriesFilteringBloodVessels == 68206)
        self.delayDisplay('testCountRejectedTrajectoriesForBloodVessels passed!')

    def testCountRejectedTrajectoriesForAngle(self, entriesAndTargets, cortex, angle, total, printOutput):
        startTime = time.time()
        filteredForAngles = Algorithms.getIncisionsWithValidAngle(entriesAndTargets, cortex, angle)
        endTime = time.time()
        totalTrajectoriesFilteringAngles = 0
        for _, targetValues in filteredForAngles.items():
            totalTrajectoriesFilteringAngles += len(targetValues)
        if printOutput:
            print('Filtering for angles total time: ', endTime - startTime, 'seconds')
            print("Total accepted filtering angles: ", totalTrajectoriesFilteringAngles)
            print("Total rejected filtering angles: ", total - totalTrajectoriesFilteringAngles)
        self.assertTrue(total - totalTrajectoriesFilteringAngles == 19405)
        self.delayDisplay('testCountRejectedTrajectoriesForAngle passed!')

    def testCountRejectedTrajectoriesCombiningAllHard(self, entries, targets, hippocampus, bloodVesselsDilate,
                                                      bloodVessels,
                                                      cortex,
                                                      angle, total, printOutput):
        startTime = time.time()
        entriesAndTargets = Algorithms.entriesAndTargetsInDict(entries, Algorithms.convertMarkupNodeToPoints(targets))
        totalTrajectoriesForAllConstraints = Algorithms.combineConstraints(entriesAndTargets, hippocampus,
                                                                                      bloodVesselsDilate,
                                                                                      bloodVessels, cortex, angle, 9999.0)
        endTime = time.time()
        totalTrajectoriesFilteringAllConstraints = 0
        for _, targets in totalTrajectoriesForAllConstraints.items():
            totalTrajectoriesFilteringAllConstraints += len(targets)
        if printOutput:
            print('Filtering for all hard constraints total time: ', endTime - startTime, 'seconds')
            print("Total accepted filtering all hard constraints: ", totalTrajectoriesFilteringAllConstraints)
            print("Total rejected filtering all hard constraints: ", total - totalTrajectoriesFilteringAllConstraints)
        self.assertTrue(total - totalTrajectoriesFilteringAllConstraints == 92884)
        self.delayDisplay('testCountRejectedTrajectoriesCombiningAll passed!')

    @staticmethod
    def getNode(fileName):
        return slicer.util.getNode(fileName)
