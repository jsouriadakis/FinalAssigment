import numpy as np

def getCoordinates(points, pointIndex):
    pos = [0, 0, 0]
    points.GetNthFiducialPosition(pointIndex, pos)
    return pos

def getAngle(entryPoint, intersectionPoint, p1, p2, p3):
    vectorPerpendicular = returnPerpendicularVectorFromThreePoints(p1, p2, p3)
    vectorIntersection = returnVectorFromPoints(intersectionPoint, entryPoint)
    angle = angleInDegreesBetweenTwoVectors(vectorPerpendicular, vectorIntersection)
    return angle

def getTrianglePoints(pointsInCell, polyData):
    p1 = polyData.GetPoint(pointsInCell.GetId(0))
    p2 = polyData.GetPoint(pointsInCell.GetId(1))
    p3 = polyData.GetPoint(pointsInCell.GetId(2))
    return p1, p2, p3

def angleInDegreesBetweenTwoVectors(vector1, vector2):
    dotProduct = np.dot(vector1, vector2)
    magnitudeVector1 = magnitudeVector(vector1)
    magnitudeVector2 = magnitudeVector(vector2)
    angle = np.degrees(dotProduct / (magnitudeVector1 * magnitudeVector2))
    return angle

def magnitudeVector(vector):
    magnitude = np.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
    return magnitude

def returnPerpendicularVectorFromThreePoints(point1, point2, point3):
    vector1 = returnVectorFromPoints(point1, point2)
    vector2 = returnVectorFromPoints(point1, point3)
    vector3 = np.cross(vector1, vector2)
    return vector3

def returnVectorFromPoints(point1, point2):
    vector = [point2[0] - point1[0], point2[1] - point1[1], point2[2] - point1[2]]
    return vector

def getDistanceBetweenPoints(point1, point2):
    distance = magnitudeVector(returnVectorFromPoints(point1,point2))
    return distance
