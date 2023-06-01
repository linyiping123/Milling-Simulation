#include "CutterTaperEndmill.h"

#include <algorithm>

OBB3Df CutterTaperEndmill::getBBox(const Posture& currentPosture) const
{
    return createBBox(currentPosture, topRadius(), height);
}

bool CutterTaperEndmill::isInside(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);

    return isBetweenEndFlat(z) && signedDistanceToSideSurface(x, z) >= 0;
}

float CutterTaperEndmill::signedDistance(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);

    if (z >= height) {
        return signedDistanceToTopFlat(x, z);
    }

    float d1 = signedDistanceToBottomFlat(x, z);
    float d2 = signedDistanceToSideSurface(x, z);
    return std::abs(d1) < std::abs(d2) ? d1 : d2;
}

void CutterTaperEndmill::generateMesh(std::vector<Vector3Df>& vertices, std::vector<Vector3Df>& normals) const
{
    vertices.clear();
    normals.clear();
    generateTopMesh(topRadius(), height, vertices, normals);
    generateSideMesh(bottomRadius(), 0.0f, height, beta, vertices, normals);
    generateBottomMesh(bottomRadius(), vertices, normals);
}
