#pragma once

#include "Cutter.h"
#include "OBB3D.h"
#include "Vector3D.h"

#include <cmath>

class CutterFilletEndmill : public Cutter {
public:
    explicit CutterFilletEndmill(float _diameter, float _filletRadius, float _height)
        : diameter(_diameter)
        , filletRadius(_filletRadius)
        , height(_height)
    {
    }
    ~CutterFilletEndmill() override = default;

    inline std::unique_ptr<Cutter> clone() const override
    {
        return std::make_unique<CutterFilletEndmill>(*this);
    }
    OBB3Df getBBox(const Posture& currentPosture) const override;
    bool isInside(const Posture& currentPosture, const Vector3Df& p) const override;
    float signedDistance(const Posture& currentPosture, const Vector3Df& p) const override;
    void generateMesh(std::vector<Vector3Df>& vertices, std::vector<Vector3Df>& normals) const override;

private:
    const float diameter;
    const float filletRadius;
    const float height;

    constexpr inline float radius() const { return diameter / 2.0f; }
    inline float distanceToFilletBallCenter(const float x, const float z) const
    {
        return Vector3Df(x, 0, z).distanceToPoint(Vector3Df(radius() - filletRadius, 0, filletRadius));
    }
    inline float signedDistanceToBottomFlat(const float x, const float z) const
    {
        if (x <= radius()) {
            return z;
        }

        return -std::hypot(x - radius(), z);
    }
    inline float signedDistanceToTopFlat(const float x, const float z) const
    {
        if (x <= radius()) {
            return -(z - height);
        }

        return -std::hypot(x - radius(), z - height);
    }
    inline float signedDistanceToSideSurface(const float x) const
    {
        return radius() - x;
    }
    inline float signedDistanceToFilletBallSurface(const float x, const float z) const
    {
        return filletRadius - distanceToFilletBallCenter(x, z);
    }

    void generateFilletBallMesh(std::vector<Vector3Df>& vertices, std::vector<Vector3Df>& normals) const;
};
