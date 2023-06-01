#include "CutterTaperFilletEndmill.h"

#include <algorithm>

OBB3Df CutterTaperFilletEndmill::getBBox(const Posture& currentPosture) const
{
    return createBBox(currentPosture, radius(), height);
}

bool CutterTaperFilletEndmill::isInside(const Posture& currentPosture, const Vector3Df& p) const
{
    float z = projectToZAxisLength(currentPosture, p);

    if (z < 0 || z > height) {
        return false;
    }

    return signedDistance(currentPosture, p) > 0;
}

float CutterTaperFilletEndmill::signedDistance(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);

    if (z <= f) {
        return std::min(signedDistanceToBottomFlat(x, z), signedDistanceToFilletBallSurface(x, z));
    } else if (z <= height) {
        return signedDistanceToSideSurface(x);
    }

    return signedDistanceToTopFlat(x, z);
}
