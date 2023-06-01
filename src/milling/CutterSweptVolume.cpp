#include "CutterSweptVolume.h"

#include "Parameter.h"

static float bruteForceSearch(const std::function<float(float)>& fun, float step = 0.01f)
{
    float min = std::numeric_limits<float>::lowest();
    for (float t = 0; t <= 1; t += step) {
        min = std::max(min, fun(t));
    }
    return min;
}

static float goldenSectionSearch(const std::function<float(float)>& fun)
{
    float a = 0.0f;
    float b = 1.0f;
    constexpr float phi = std::numbers::phi_v<float>; // golden ratio
    while (std::abs(b - a) > 0.001f) { // set the tolerance to 0.001f
        float x1 = b - (b - a) / phi;
        float x2 = a + (b - a) / phi;
        float f1 = fun(x1);
        float f2 = fun(x2);
        if (f1 < f2) {
            a = x1;
        } else {
            b = x2;
        }
    }
    return fun((a + b) / 2); // midpoint of the final interval
}

float CutterSweptVolume::getLength() const
{
    switch (type) {
    case Type::Scatter:
        return 0;
    case Type::Line:
        return (endWithOffset.center - startWithOffset.center).length();
    case Type::Arc: {
        PostureArcInterpolator interpolator(startWithOffset, endWithOffset, plane, radius, isClockwise);
        return interpolator.getLength();
    }
    case Type::Line5Axis:
        return (endWithOffset.center - startWithOffset.center).length();
    default:
        return 0;
    }
}

OBB3Df CutterSweptVolume::getBBox() const
{
    return bbox;
}

std::vector<OBB3Df> CutterSweptVolume::getBBoxs() const
{
    return bboxs;
}

float CutterSweptVolume::signedDistance(const Vector3Df& p) const
{
    switch (type) {
    case Type::Scatter:
        return signedDistanceScatter(p);
    case Type::Line:
        return signedDistanceLine(p);
    case Type::Arc:
        return signedDistanceArc(p);
    case Type::Line5Axis:
        return signedDistanceLine5Axis(p);
    default:
        return std::numeric_limits<float>::lowest();
    }
}

void CutterSweptVolume::setOffset(const Vector3Df& offset)
{
    this->offset = offset;
}

void CutterSweptVolume::setCutter(std::unique_ptr<Cutter> cutter)
{
    this->cutter = cutter->clone();
}

void CutterSweptVolume::setScatter(const Posture& start)
{
    this->type = Type::Scatter;
    this->startWithOffset = start;
    this->startWithOffset.center -= offset;

    this->bbox = cutter->getBBox(this->startWithOffset);
    this->bboxs = { this->bbox };
}

void CutterSweptVolume::setLine(const Posture& start, const Posture& end)
{
    this->startWithOffset = start;
    this->startWithOffset.center -= offset;
    this->endWithOffset = end;
    this->endWithOffset.center -= offset;

    if (startWithOffset.direction.equal(endWithOffset.direction)) {
        this->type = Type::Line;
        OBB3Df bbox0 = cutter->getBBox(this->startWithOffset);
        OBB3Df bbox1 = cutter->getBBox(this->endWithOffset);
        float radius = bbox0.getAxis(0).length();
        Vector3Df c0 = bbox0.getCenter(), c1 = bbox1.getCenter();
        Vector3Df c = (c0 + c1) / 2.0f;
        Vector3Df axisZ = bbox0.getAxis(2);
        Vector3Df axisX = (c0 - c1).normalize() * ((c0 - c1).length() / 2.0f + radius);
        Vector3Df axisY = axisZ.cross(axisX).normalize() * radius;
        bbox = { c, axisX, axisY, axisZ };
        bboxs = { bbox };
        return;
    }

    this->type = Type::Line5Axis;
    const float centerStep = Parameter::getInstance()->getCenterStep();
    const float directionStep = Parameter::getInstance()->getDirectionStep();
    PostureLineInterpolator interpolator(startWithOffset, endWithOffset, centerStep, directionStep);
    bboxs.clear();
    Posture posture = startWithOffset;
    while (!interpolator.getPostureByStep(posture)) {
        bboxs.push_back(cutter->getBBox(posture));
    }
}

void CutterSweptVolume::setArc(const Posture& start, const Posture& end, Plane plane, float radius, bool isClockwise)
{
    this->type = Type::Arc;
    this->startWithOffset = start;
    this->startWithOffset.center -= offset;
    this->endWithOffset = end;
    this->endWithOffset.center -= offset;
    this->plane = plane;
    this->radius = radius;
    this->isClockwise = isClockwise;

    const float centerStep = Parameter::getInstance()->getCenterStep();
    const float directionStep = Parameter::getInstance()->getDirectionStep();
    PostureArcInterpolator interpolator(startWithOffset, endWithOffset, plane, radius, isClockwise, centerStep, directionStep);
    bboxs.clear();
    Posture posture = startWithOffset;
    while (!interpolator.getPostureByStep(posture)) {
        bboxs.push_back(cutter->getBBox(posture));
    }
}

CutterSweptVolume& CutterSweptVolume::operator=(const CutterSweptVolume& other)
{
    type = other.type;
    startWithOffset = other.startWithOffset;
    endWithOffset = other.endWithOffset;
    plane = other.plane;
    radius = other.radius;
    isClockwise = other.isClockwise;
    bbox = other.bbox;
    bboxs = other.bboxs;
    cutter = other.cutter->clone();
    return *this;
}

float CutterSweptVolume::signedDistanceScatter(const Vector3Df& p) const
{
    return cutter->signedDistance(startWithOffset, p);
}

float CutterSweptVolume::signedDistanceLine(const Vector3Df& p) const
{
#if 1
    const Vector3Df &pt0 = p, pt1 = p - (endWithOffset.center - startWithOffset.center);
    // auto interpolator = PostureLineInterpolator(Posture(pt0), Posture(pt1));
    // auto interpolationSignedDistance = [&](float t) -> float {
    //     Vector3Df pt = interpolator.getPosture(t).center;
    //     return cutter->signedDistance(startWithOffset, pt);
    // };
#else
    PostureLineInterpolator interpolator(startWithOffset, endWithOffset);
    auto interpolationSignedDistance = [&](float t) -> float {
        Posture posture = interpolator.getPosture(t);
        return cutter->signedDistance(posture, p);
    };
#endif

    // return bruteForceSearch(interpolationSignedDistance);

    // return goldenSectionSearch(interpolationSignedDistance);

    const Vector3Df intersectionPoint = startWithOffset.center.intersectionPointOfDistanceToLineSegment(pt0, pt1);
    return cutter->signedDistance(startWithOffset, intersectionPoint);
}

float CutterSweptVolume::signedDistanceArc(const Vector3Df& p) const
{
    const Vector3Df &pt0 = p, pt1 = p - (endWithOffset.center - startWithOffset.center);
    PostureArcInterpolator interpolator(Posture(pt0), Posture(pt1), plane, radius, isClockwise);
    auto interpolationSignedDistance = [&](float t) -> float {
        Vector3Df pt = interpolator.getPosture(t).center;
        return cutter->signedDistance(startWithOffset, pt);
    };

    // return bruteForceSearch(interpolationSignedDistance);

    return goldenSectionSearch(interpolationSignedDistance);

    // const Vector3Df intersectionPoint = startWithOffset.center.intersectionPointOfDistanceToArc(pt0, pt1, radius, isClockwise);
    // return cutter->signedDistance(startWithOffset, intersectionPoint);
}

float CutterSweptVolume::signedDistanceLine5Axis(const Vector3Df& p) const
{
#if 1
    const Vector3Df pt0 = p, pt1 = p - (endWithOffset.center - startWithOffset.center);
    const float angle = startWithOffset.direction.angleToLine(endWithOffset.direction);
    auto interpolationP = [&](float t) -> Vector3Df {
        return pt0 * (1 - t) + pt1 * t;
    };
    auto interpolationD = [&](float t) -> Vector3Df {
        return startWithOffset.direction.rotate(startWithOffset.direction.cross(endWithOffset.direction).normalize(), angle * t);
    };
    auto interpolationSignedDistance = [&](float t) -> float {
        Vector3Df pt = interpolationP(t);
        Vector3Df directiont = interpolationD(t);
        Posture posture = Posture(startWithOffset.center, directiont);
        return cutter->signedDistance(posture, pt);
    };
#else
    PostureLineInterpolator interpolator(startWithOffset, endWithOffset);
    auto interpolationSignedDistance = [&](float t) -> float {
        return cutter->signedDistance(interpolator.getPosture(t), p);
    };
#endif

    // return bruteForceSearch(interpolationSignedDistance);

    return goldenSectionSearch(interpolationSignedDistance);
}
