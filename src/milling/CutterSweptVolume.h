#pragma once

#include "Cutter.h"
#include "OBB3D.h"
#include "Posture.h"
#include "Vector3D.h"

#include <functional>
#include <memory>
#include <vector>

class CutterSweptVolume {
public:
    CutterSweptVolume() = default;
    CutterSweptVolume(const CutterSweptVolume& other)
        : type(other.type)
        , offset(other.offset)
        , startWithOffset(other.startWithOffset)
        , endWithOffset(other.endWithOffset)
        , plane(other.plane)
        , radius(other.radius)
        , isClockwise(other.isClockwise)
        , bbox(other.bbox)
        , bboxs(other.bboxs)
        , cutter(other.cutter->clone())
    {
    }
    ~CutterSweptVolume() = default;

    using Plane = PostureArcInterpolator::Plane;

    float getLength() const;
    OBB3Df getBBox() const;
    std::vector<OBB3Df> getBBoxs() const;
    float signedDistance(const Vector3Df& p) const;
    void setOffset(const Vector3Df& offset);
    void setCutter(std::unique_ptr<Cutter> cutter);
    void setScatter(const Posture& start);
    void setLine(const Posture& start, const Posture& end);
    void setArc(const Posture& start, const Posture& end, Plane plane, float radius, bool isClockwise);
    CutterSweptVolume& operator=(const CutterSweptVolume& other);

private:
    enum class Type {
        Scatter,
        Line,
        Arc,
        Line5Axis,
    };
    Type type;

    Vector3Df offset = { 0, 0, 0 };
    Posture startWithOffset, endWithOffset;
    Plane plane = Plane::XY;
    float radius;
    bool isClockwise;
    OBB3Df bbox;
    std::vector<OBB3Df> bboxs;
    std::unique_ptr<Cutter> cutter;

    float signedDistanceScatter(const Vector3Df& p) const;
    float signedDistanceLine(const Vector3Df& p) const;
    float signedDistanceArc(const Vector3Df& p) const;
    float signedDistanceLine5Axis(const Vector3Df& p) const;
};
