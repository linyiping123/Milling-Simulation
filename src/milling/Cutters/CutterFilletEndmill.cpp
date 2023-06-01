#include "CutterFilletEndmill.h"

#include <algorithm>

OBB3Df CutterFilletEndmill::getBBox(const Posture& currentPosture) const
{
    return createBBox(currentPosture, radius(), height);
}

bool CutterFilletEndmill::isInside(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);

    if (z < 0 || z > height) {
        return false;
    }

    if (z <= filletRadius) {
        return x <= radius() - filletRadius + std::sqrt(filletRadius * filletRadius - (z - filletRadius) * (z - filletRadius));
    }

    return x <= radius();
}

float CutterFilletEndmill::signedDistance(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);

    if (z <= filletRadius) {
        return std::min(signedDistanceToBottomFlat(x, z), signedDistanceToFilletBallSurface(x, z));
    } else if (z <= height) {
        return signedDistanceToSideSurface(x);
    }

    return signedDistanceToTopFlat(x, z);
}

void CutterFilletEndmill::generateMesh(std::vector<Vector3Df>& vertices, std::vector<Vector3Df>& normals) const
{
    vertices.clear();
    normals.clear();
    generateTopMesh(radius(), height, vertices, normals);
    generateSideMesh(radius(), filletRadius, height, 0.0f, vertices, normals);
    generateFilletBallMesh(vertices, normals);
    generateBottomMesh(radius() - filletRadius, vertices, normals);
}

void CutterFilletEndmill::generateFilletBallMesh(std::vector<Vector3Df>& vertices, std::vector<Vector3Df>& normals) const
{
    constexpr int longitudeSegments = 128;
    constexpr int latitudeSegments = 128;

    const float e = diameter * 0.5f - filletRadius;
    const float f = filletRadius;

    std::vector<Vector3Df> verticesFilletBall;
    std::vector<Vector3Df> normalsFilletBall;

    for (int i = 0; i <= longitudeSegments; ++i) {
        const float theta = 2.0f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(longitudeSegments);
        const float sinTheta = sin(theta);
        const float cosTheta = cos(theta);
        const Vector3Df centerOfBall = Vector3Df(e * cosTheta, e * sinTheta, f);

        for (int j = 0; j <= latitudeSegments; ++j) {
            const float phi = std::numbers::pi_v<float> * 0.5f + static_cast<float>(j) * std::numbers::pi_v<float> / static_cast<float>(2 * latitudeSegments);
            const float sinPhi = sin(phi);
            const float cosPhi = cos(phi);
            const Vector3Df normal = Vector3Df(cosTheta * sinPhi, sinTheta * sinPhi, cosPhi);
            const Vector3Df vertex = centerOfBall + normal * filletRadius;

            verticesFilletBall.push_back(vertex);
            normalsFilletBall.push_back(normal);
        }
    }

    for (int i = 0; i < longitudeSegments; ++i) {
        for (int j = 0; j < latitudeSegments; ++j) {
            const int first = (latitudeSegments + 1) * i + j;
            const int second = first + latitudeSegments + 1;

            vertices.push_back(verticesFilletBall[first]);
            vertices.push_back(verticesFilletBall[first + 1]);
            vertices.push_back(verticesFilletBall[second]);

            normals.push_back(normalsFilletBall[first]);
            normals.push_back(normalsFilletBall[first + 1]);
            normals.push_back(normalsFilletBall[second]);

            vertices.push_back(verticesFilletBall[second]);
            vertices.push_back(verticesFilletBall[first + 1]);
            vertices.push_back(verticesFilletBall[second + 1]);

            normals.push_back(normalsFilletBall[second]);
            normals.push_back(normalsFilletBall[first + 1]);
            normals.push_back(normalsFilletBall[second + 1]);
        }
    }
}
