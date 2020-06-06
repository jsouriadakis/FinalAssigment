import MathTools
import vtk, qt, ctk, slicer
import numpy as np

def combineConstraints(entriesAndTargets, ventricles, vessels, cortex, validAngleOfIntersection, maximumIncisionValue):
    validTrajectories = 0
    paths = {}
    ventriclesTree, _ = getTree(ventricles)
    vesselTree, _ = getTree(vessels)
    cortexTree, cortexPolyData = getTree(cortex, (0, 0.5))
    for entry, targets in entriesAndTargets.items():
        for target in targets:
            if not validLength(maximumIncisionValue, entry, target):
                continue
            if pointsIntersect(ventriclesTree, entry, target):
                continue
            if pointsIntersect(vesselTree, entry, target):
                continue
            if not hasIntersectionValidAngle(cortexPolyData, cortexTree, entry, target,validAngleOfIntersection):
                continue
            key = tuple(entry)
            if key in paths:
                paths[key].append(target)
            else:
                paths[key] = [target]
            validTrajectories += 1

    print('Valid Trajectories: ', validTrajectories)
    return paths


# this function was given by Rachel Sparks
def printEntryAndTargetsInDict(entriesAndTargets):
    # we are going to create a poly data defined by a set of points and lines
    points = vtk.vtkPoints()  # these are our end/start points
    lines = vtk.vtkCellArray()  # these are the lines connecting them
    for entry, targets in entriesAndTargets.items():
        entryId = points.InsertNextPoint(entry[0], entry[1], entry[2])
        for target in targets:
            targetInd = points.InsertNextPoint(target[0], target[1], target[2])

            # how to connect my points
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, entryId)
            line.GetPointIds().SetId(1, targetInd)
            lines.InsertNextCell(line)

    myPaths = vtk.vtkPolyData()
    myPaths.SetPoints(points)
    myPaths.SetLines(lines)
    return myPaths

def entriesAndTargetsInDict(entriesNode, targetsPoints):
    paths = {}
    for i in range(0, int(entriesNode.GetNumberOfMarkups())):
        entry = MathTools.getCoordinates(entriesNode, i)
        for target in targetsPoints:
            key = tuple(entry)
            if key in paths:
                paths[key].append(target)
            else:
                paths[key] = [target]

    return paths

def convertMarkupNodeToPoints(markupNode):
    newTargets = []
    for i in range(0, int(markupNode.GetNumberOfMarkups())):
        target = MathTools.getCoordinates(markupNode, i)
        newTargets.append(target)
    return newTargets


def getValidTargets(targetsNode, hippo):
    pointsRejected = 0
    newTargets = []
    [xmax, ymax, zmax] = hippo.GetImageData().GetDimensions()
    subsampleFactor = 1
    for i in range(0, int(targetsNode.GetNumberOfMarkups())):
        target = MathTools.getCoordinates(targetsNode, i)
        imageVoxelID = hippo.GetImageData().FindPoint(target[0], target[1], target[2])

        xIndex = int(imageVoxelID%xmax)
        yIndex = int(imageVoxelID%(xmax*ymax)/xmax)
        zIndex = int((imageVoxelID/(xmax*ymax)))

        if hippo.GetImageData().GetScalarComponentAsDouble(xIndex, yIndex, zIndex, 0) != 0:
            newTargets.append(target)
        else:
            pointsRejected += 1
    print('Rejected Points: ',pointsRejected)
    return newTargets


def getIncisionsWithValidLength(entriesAndTargets, maximumIncisionValue):
    paths = {}
    trajectoriesRejected = 0
    for entry, targets in entriesAndTargets.items():
        for target in targets:
            if validLength(maximumIncisionValue, entry, target):
                key = tuple(entry)
                if key in paths:
                    paths[key].append(target)
                else:
                    paths[key] = [target]
            else:
                trajectoriesRejected += 1
    print('Rejected Trajectories:',trajectoriesRejected)
    return paths

def validLength(maximumIncisionValue, entry, target):
    vector = MathTools.returnVectorFromPoints(entry,target)
    incisionDistance = MathTools.magnitudeVector(vector)
    if incisionDistance <= maximumIncisionValue and maximumIncisionValue > 0.00:
        return True
    return False

def getIncisionsWithValidArea(entriesAndTargets, area):
    paths = {}
    trajectoriesRejected = 0
    tree, _ = getTree(area)
    for entry, targets in entriesAndTargets.items():
        for target in targets:
            if not pointsIntersect(tree, entry, target):
                key = tuple(entry)
                if key in paths:
                    paths[key].append(target)
                else:
                    paths[key] = [target]
            else:
                trajectoriesRejected += 1
    print('Rejected Trajectories:',trajectoriesRejected)
    return paths

def getTree(area, value=None):
    if value is not None:
        mesh = vtk.vtkMarchingCubes()
        mesh.SetInputData(area.GetImageData())
        mesh.SetValue(value[0], value[1])
    else:
        mesh = vtk.vtkDiscreteMarchingCubes()
        mesh.SetInputData(area.GetImageData())
    mesh.Update()
    polyData = mesh.GetOutput()
    tree = vtk.vtkOBBTree()
    tree.SetDataSet(polyData)
    tree.BuildLocator()
    return tree, polyData

def pointsIntersect(tree, entryPoint, targetPoint):
    pointsWithinTriangle = vtk.vtkPoints()
    pointsIdInTriangle = vtk.vtkIdList()
    if tree.IntersectWithLine(entryPoint, targetPoint, pointsWithinTriangle, pointsIdInTriangle) != 0:
        return True
    return False

def getIncisionsWithValidAngle( entriesAndTargets, cortex, validAngleOfIntersection):
    paths = {}
    trajectoriesRejected = 0
    tree, polyData = getTree(cortex, (0, 0.5))
    for entry, targets in entriesAndTargets.items():
        for target in targets:
            if hasIntersectionValidAngle(polyData, tree, entry, target, validAngleOfIntersection):
                key = tuple(entry)
                if key in paths:
                    paths[key].append(target)
                else:
                    paths[key] = [target]
            else:
                trajectoriesRejected += 1
    print('Rejected Trajectories:',trajectoriesRejected)
    return paths

def hasIntersectionValidAngle(polyData, tree, entryPoint, targetPoint, validAngleOfIntersection):
    entry = [entryPoint[0], entryPoint[1], entryPoint[2]]
    target = [targetPoint[0], targetPoint[1], targetPoint[2]]
    pointIntersectWithinTriangle = vtk.vtkPoints()
    cellIdOfTriangle = vtk.vtkIdList()
    pointsInCell = vtk.vtkIdList()
    tree.IntersectWithLine(entry, target, pointIntersectWithinTriangle, cellIdOfTriangle)
    intersectionPoint = pointIntersectWithinTriangle.GetPoint(0)
    polyData.GetCellPoints(cellIdOfTriangle.GetId(0), pointsInCell)
    p1, p2, p3 = MathTools.getTrianglePoints(pointsInCell, polyData)
    angle = MathTools.getAngle(entryPoint, intersectionPoint, p1, p2, p3)
    return angle < validAngleOfIntersection


def printBestTrajectoryForEachEntry(precisionValue, maximumIncisionValue ,entriesAndTargets, *nodes):
    trees = treesOfNodes(*nodes)
    pointsByDistance = {}
    combinedEntriesAndTargets = {}
    pointsByDistance = dictByMaximumDistanceFromLinesToNode(precisionValue,entriesAndTargets,*trees)
    for distance in pointsByDistance:
        vector = MathTools.returnVectorFromPoints(distance[1][0],distance[1][1])
        incisionDistance = MathTools.magnitudeVector(vector)
        if incisionDistance <= maximumIncisionValue and maximumIncisionValue > 0.00:
            key = tuple(distance[1][0])
            if not key in combinedEntriesAndTargets:
                combinedEntriesAndTargets[key] = [distance[1][1]]
            # print(distance)
    allPaths = printEntryAndTargetsInDict(combinedEntriesAndTargets)
    pathNode = slicer.mrmlScene.AddNewNodeByClass('vtkMRMLModelNode', 'GoodPaths')
    pathNode.SetAndObserveMesh(allPaths)
    return combinedEntriesAndTargets


def treesOfNodes(*nodes):
    trees = []
    for node in nodes:
        mesh = vtk.vtkDiscreteMarchingCubes()
        mesh.SetInputData(node.GetImageData())
        mesh.Update()
        polyData = mesh.GetOutput()
        tree = vtk.vtkCellLocator()
        tree.SetDataSet(polyData)
        tree.BuildLocator()
        trees.append(tree)
    return trees

def dictByMaximumDistanceFromLinesToNode(precisionValue,entriesAndTargets,*trees):
    pointsByDistance = {}
    sortedPoints = {}
    for entry, targets in entriesAndTargets.items():
        for target in targets:
            #weight = weight 
            distance = 0.0
            for tree in trees:
                distance += distanceToClosestPointToLine(tree,entry,target,precisionValue)
            key = float(distance)
            if key in pointsByDistance.keys():
                pointsByDistance[key].append([entry,target])
            else:
                pointsByDistance[key] = [entry,target]
    sortedPoints = sorted(pointsByDistance.items(), reverse = True)
    return sortedPoints

def distanceToClosestPointToLine(tree,point1,point2,precision):
    minDistance = 0
    xEntry, yEntry, zEntry = point1[0], point1[1], point1[2]
    xTarget, yTarget, zTarget = point2[0], point2[1], point2[2]
    for step in np.arange(0,1,precision):
        x = xEntry + step*(xTarget - xEntry)
        y = yEntry + step*(yTarget - yEntry)
        z = zEntry + step*(zTarget - zEntry)
        testPoint = (x,y,z)
        closestPoint = [0.0,0.0,0.0]
        vector = MathTools.returnVectorFromPoints(point1, point2)
        minDistance = MathTools.magnitudeVector(vector)
        ID = 0
        cellId = vtk.reference(ID)
        subId = vtk.reference(ID)
        closestPointDist2 =  vtk.reference(ID)
        tree.FindClosestPoint(testPoint, closestPoint, cellId, subId, closestPointDist2)
        if closestPointDist2 < minDistance:
            minDistance = closestPointDist2
        minDistance
    return minDistance
    
def addEntriesAndTargetsInDictFromID(entriesID,targetsID):
    entriesNode = slicer.util.getNode("entries")
    targetsNode = slicer.util.getNode("targets")
    paths = {}
    for i in entriesID:
        entry = MathTools.getCoordinates(entriesNode, i)
        for j in targetsID:
            target = MathTools.getCoordinates(targetsNode,j)
            key = tuple(entry)
            if key in paths:
                paths[key].append(target)
            else:
                paths[key] = [target]

    return paths

def getBestAndWorstTrajectory(entriesAndTargets, precisionValue, *nodes):
    trees = treesOfNodes(*nodes)
    sortedPointsAccordingToDistance = dictByMaximumDistanceFromLinesToNode(precisionValue,entriesAndTargets,*trees)
    mostDistance = sortedPointsAccordingToDistance[0]
    leastDistance = sortedPointsAccordingToDistance[-1]
    return mostDistance, leastDistance


