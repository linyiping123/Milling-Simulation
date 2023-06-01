#include "CutterFlatEndmill.h"

OBB3Df CutterFlatEndmill::getBBox(const Posture& currentPosture) const
{
    return createBBox(currentPosture, radius(), height);
}

bool CutterFlatEndmill::isInside(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);
    return isBetweenEndFlat(z) && signedDistanceToSideSurface(x) >= 0;
}

float CutterFlatEndmill::signedDistance(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);
    return isBetweenEndFlat(z) ? signedDistanceToSideSurface(x) : signedDistanceToBothEndFlat(x, z);
}

void CutterFlatEndmill::generateMesh(std::vector<Vector3Df>& vertices, std::vector<Vector3Df>& normals) const
{
    vertices.clear();
    normals.clear();
    generateTopMesh(radius(), height, vertices, normals);
    generateSideMesh(radius(), 0.0f, height, 0.0f, vertices, normals);
    generateBottomMesh(radius(), vertices, normals);
}
