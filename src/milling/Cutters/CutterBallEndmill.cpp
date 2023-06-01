#include "CutterBallEndmill.h"

#include <execution>

OBB3Df CutterBallEndmill::getBBox(const Posture& currentPosture) const
{
    return createBBox(currentPosture, radius, height);
}

bool CutterBallEndmill::isInside(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);

    if (!isBetweenEndFlat(z)) {
        return false;
    }

    if (isBetweenEndBall(z)) {
        return signedDistanceToBallSurface(x, z) >= 0;
    } else {
        return signedDistanceToSideSurface(x) >= 0;
    }
}

float CutterBallEndmill::signedDistance(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);

    if (z <= radius) {
        return signedDistanceToBallSurface(x, z);
    } else if (z < height) {
        return signedDistanceToSideSurface(x);
    }

    return signedDistanceToTopFlat(x, z);
}

void CutterBallEndmill::generateMesh(std::vector<Vector3Df>& vertices, std::vector<Vector3Df>& normals) const
{
    vertices.clear();
    normals.clear();
    generateTopMesh(radius, height, vertices, normals);
    generateSideMesh(radius, radius, height, 0.0f, vertices, normals);
    generateBottomBallMesh(vertices, normals);
}

void CutterBallEndmill::generateBottomBallMesh(std::vector<Vector3Df>& vertices, std::vector<Vector3Df>& normals) const
{
    // Generate the vertices and normals for the bottom sphere face
    constexpr int latitudeSegments = 128;
    constexpr int longitudeSegments = 128;

    std::vector<Vector3Df> verticesBottom;
    std::vector<Vector3Df> normalsBottom;

    for (int i = 0; i <= latitudeSegments; ++i) {
        float theta = std::numbers::pi_v<float> * 0.5f + i * std::numbers::pi_v<float> / (2 * latitudeSegments);
        float sinTheta = sin(theta);
        float cosTheta = cos(theta);

        for (int j = 0; j <= longitudeSegments; ++j) {
            float phi = j * 2 * std::numbers::pi_v<float> / longitudeSegments;
            float sinPhi = sin(phi);
            float cosPhi = cos(phi);

            float x = radius * sinTheta * cosPhi;
            float y = radius * sinTheta * sinPhi;
            float z = radius * cosTheta;

            verticesBottom.emplace_back(x, y, z + radius);
            normalsBottom.push_back(Vector3Df(x, y, z).normalize());
        }
    }

    for (int i = 0; i < latitudeSegments; ++i) {
        for (int j = 0; j < longitudeSegments; ++j) {
            int first = (i * (longitudeSegments + 1)) + j;
            int second = first + longitudeSegments + 1;

            vertices.push_back(verticesBottom[first]);
            vertices.push_back(verticesBottom[second]);
            vertices.push_back(verticesBottom[first + 1]);

            normals.push_back(normalsBottom[first]);
            normals.push_back(normalsBottom[second]);
            normals.push_back(normalsBottom[first + 1]);

            vertices.push_back(verticesBottom[first + 1]);
            vertices.push_back(verticesBottom[second]);
            vertices.push_back(verticesBottom[second + 1]);

            normals.push_back(normalsBottom[first + 1]);
            normals.push_back(normalsBottom[second]);
            normals.push_back(normalsBottom[second + 1]);
        }
    }
}
