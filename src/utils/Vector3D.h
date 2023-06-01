#pragma once

#include "utils.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <ostream>

template <class T>
class Vector3D {
public:
    constexpr inline Vector3D() noexcept
        : x(0)
        , y(0)
        , z(0)
    {
    }
    constexpr inline Vector3D(T _x, T _y, T _z) noexcept
        : x(_x)
        , y(_y)
        , z(_z)
    {
    }
    constexpr inline Vector3D(const Vector3D& v) noexcept
        : x(v.x)
        , y(v.y)
        , z(v.z)
    {
    }
    ~Vector3D() = default;

    constexpr inline bool isZero() const noexcept
    {
        return x == 0 && y == 0 && z == 0;
    }
    constexpr inline bool equal(const Vector3D& v) const noexcept
    {
        return floatEqual(x, v.x) && floatEqual(y, v.y) && floatEqual(z, v.z);
    }
    constexpr inline T length() const noexcept
    {
        return (T)std::hypot(x, y, z);
    }
    constexpr inline T dot(const Vector3D& v) const noexcept
    {
        return x * v.x + y * v.y + z * v.z;
    }
    constexpr inline Vector3D<T> abs() const noexcept
    {
        return Vector3D<T>(std::abs(x), std::abs(y), std::abs(z));
    }
    constexpr inline Vector3D<T> cross(const Vector3D& v) const noexcept
    {
        return Vector3D<T>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    constexpr inline Vector3D<T> normalize() const noexcept
    {
        return *this / length();
    }
    constexpr inline Vector3D<T> rotate(const Vector3D<T>& axis, const T angle) const noexcept
    {
        const T c = (T)std::cos(angle);
        const T s = (T)std::sin(angle);
        const T t = 1 - c;
        const Vector3D<T> n = axis.normalize();
        return Vector3D<T>(
            c * x + t * (n.x * n.x * x + n.y * n.x * y + n.z * n.x * z) + s * (-n.z * y + n.y * z),
            c * y + t * (n.x * n.y * x + n.y * n.y * y + n.y * n.z * z) + s * (n.z * x - n.x * z),
            c * z + t * (n.x * n.z * x + n.y * n.z * y + n.z * n.z * z) + s * (-n.y * x + n.x * y));
    }
    constexpr inline T angleToLine(const Vector3D& v) const noexcept
    {
        return (T)std::acos(dot(v) / (length() * v.length()));
    }
    constexpr inline T distanceToPoint(const Vector3D<T>& p) const noexcept
    {
        return (*this - p).length();
    }
    constexpr inline T distanceToLine(const Vector3D<T>& p, const Vector3D<T>& q) const noexcept
    {
        if (p.equal(q)) {
            return distanceToPoint(p);
        }

        Vector3D<T> a = *this - p;
        Vector3D<T> b = (q - p).normalize();
        Vector3D<T> c = b * a.projectionLength(b);
        return (a - c).length();
    }
    constexpr inline T distanceToLineSegment(const Vector3D<T>& p, const Vector3D<T>& q) const noexcept
    {
        if (p.equal(q)) {
            return distanceToPoint(p);
        }

        Vector3D<T> a = *this - p;
        Vector3D<T> b = q - p;
        T t = a.projection(b);
        if (t < 0) {
            return a.length();
        }
        if (t > b.length()) {
            return (*this - q).length();
        }
        return (a - b.normalize() * t).length();
    }
    constexpr inline Vector3D<T> intersectionPointOfDistanceToLineSegment(const Vector3D<T>& p, const Vector3D<T>& q) const noexcept
    {
        if (p.equal(q)) {
            return p;
        }

        Vector3D<T> a = *this - p;
        Vector3D<T> b = q - p;
        T t = a.projection(b);
        if (t < 0) {
            return p;
        }
        if (t > b.length()) {
            return q;
        }
        return p + b.normalize() * t;
    }
    constexpr inline T distanceToArc(const Vector3D<T>& p, const Vector3D<T>& q, const T r, bool isClockwise = true) const noexcept
    {
        if (p.equal(q)) {
            return distanceToPoint(p);
        }

        if (floatEqual(r, 0)) {
            return distanceToLineSegment(p, q);
        }

        if (floatEqual(x, p.x) && floatEqual(x, q.x)) {
            return Vector3D<T>(y, z, 0).distanceToArc(Vector3D<T>(p.y, p.z, 0), Vector3D<T>(q.y, q.z, 0), r, isClockwise);
        }

        if (floatEqual(y, p.y) && floatEqual(y, q.y)) {
            return Vector3D<T>(x, z, 0).distanceToArc(Vector3D<T>(p.x, p.z, 0), Vector3D<T>(q.x, q.z, 0), r, isClockwise);
        }

        Vector3D<T> c = centerOfCircle(p, q, r, isClockwise);

        Vector3D<T> cp = p - c;
        Vector3D<T> cq = q - c;
        Vector3D<T> ca = *this - c;
        T angle = cp.angleToLine(cq);
        T angle1 = cp.angleToLine(ca);
        T angle2 = cq.angleToLine(ca);
        if ((r > 0 && angle1 < angle && angle2 < angle) || (r < 0 && !(angle1 < angle && angle2 < angle))) {
            return std::abs(ca.length() - std::abs(r));
        }

        return std::min(distanceToPoint(p), distanceToPoint(q));
    }

    static constexpr inline Vector3D<T> centerOfCircle(const Vector3D<T>& p, const Vector3D<T>& q, const T r, bool isClockwise = true) noexcept
    {
        if (p.equal(q)) {
            return p;
        }

        if (floatEqual(r, 0)) {
            return (p + q) / (T)2;
        }

        if (floatEqual(p.x, q.x)) {
            T px = p.x;
            Vector3D<T> center = centerOfCircleXOY(Vector3D<T>(p.y, p.z, 0), Vector3D<T>(q.y, q.z, 0), r, isClockwise);
            return Vector3D<T>(px, center.x, center.y);
        }

        if (floatEqual(p.y, q.y)) {
            T py = p.y;
            Vector3D<T> center = centerOfCircleXOY(Vector3D<T>(p.x, p.z, 0), Vector3D<T>(q.x, q.z, 0), r, isClockwise);
            return Vector3D<T>(center.x, py, center.y);
        }

        return centerOfCircleXOY(p, q, r, isClockwise);
    }

    static constexpr inline Vector3D<T> centerOfCircleXOY(const Vector3D<T>& p, const Vector3D<T>& q, const T r, bool isClockwise = true) noexcept
    {
        const Vector3D<T> pqc = (p + q) / (T)2;
        const Vector3D<T> pq = q - p;
        const Vector3D<T> pqn = pq.cross(Vector3D<T>(0, 0, 1)).normalize();
        return pqc + pqn * (isClockwise ? 1 : -1) * (r > 0 ? 1 : -1) * std::sqrt(r * r - pq.length() * pq.length() / (T)4);
    }
    constexpr inline Vector3D<T> intersectionPointOfDistanceToArc(const Vector3D<T>& p, const Vector3D<T>& q, const T r, bool isClockwise = true) const noexcept
    {
        if (p.equal(q)) {
            return p;
        }

        if (floatEqual(r, 0)) {
            return intersectionPointOfDistanceToLineSegment(p, q);
        }

        Vector3D<T> c = centerOfCircle(p, q, r, isClockwise);
        if (equal(c)) {
            return p;
        }

        Vector3D<T> cp = p - c;
        Vector3D<T> cq = q - c;
        Vector3D<T> ca = *this - c;
        T angle = cp.angleToLine(cq);
        T angle1 = cp.angleToLine(ca);
        T angle2 = cq.angleToLine(ca);
        if ((r > 0 && angle1 < angle && angle2 < angle) || (r < 0 && !(angle1 < angle && angle2 < angle))) {
            return c + ca.normalize() * std::abs(r);
        }

        return distanceToPoint(p) > distanceToPoint(q) ? p : q;
    }

    constexpr inline T projection(const Vector3D<T>& v) const noexcept
    {
        return dot(v) / v.length();
    }
    constexpr inline T projectionLength(const Vector3D<T>& v) const noexcept
    {
        return std::abs(projection(v));
    }

    constexpr inline T operator[](int i) const noexcept
    {
        return i == 0 ? x : (i == 1 ? y : z);
    }
    constexpr inline T& operator[](int i) noexcept
    {
        return i == 0 ? x : (i == 1 ? y : z);
    }
    constexpr inline Vector3D& operator=(const Vector3D& v) noexcept
    {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }
    constexpr inline Vector3D operator+(const Vector3D& v) const noexcept
    {
        return Vector3D(x + v.x, y + v.y, z + v.z);
    }
    constexpr inline Vector3D operator-(const Vector3D& v) const noexcept
    {
        return Vector3D(x - v.x, y - v.y, z - v.z);
    }
    constexpr inline Vector3D operator+(T v) const noexcept
    {
        return Vector3D(x + v, y + v, z + v);
    }
    constexpr inline Vector3D operator-(T v) const noexcept
    {
        return Vector3D(x - v, y - v, z - v);
    }
    constexpr inline Vector3D operator*(T v) const noexcept
    {
        return Vector3D(x * v, y * v, z * v);
    }
    constexpr inline Vector3D operator/(T v) const noexcept
    {
        return Vector3D(x / v, y / v, z / v);
    }
    constexpr inline bool operator==(T v) const noexcept
    {
        return x == v && y == v && z == v;
    }
    constexpr inline bool operator!=(T v) const noexcept
    {
        return x != v || y != v || z != v;
    }
    constexpr inline bool operator==(const Vector3D& v) const noexcept
    {
        return (x == v.x && y == v.y && z == v.z);
    }
    constexpr inline bool operator!=(const Vector3D& v) const noexcept
    {
        return (x != v.x || y != v.y || z != v.z);
    }
    constexpr inline bool operator<(const Vector3D& v) const noexcept
    {
        return (x < v.x && y < v.y && z < v.z);
    }
    constexpr inline bool operator>(const Vector3D& v) const noexcept
    {
        return (x > v.x && y > v.y && z > v.z);
    }
    constexpr inline bool operator<=(const Vector3D& v) const noexcept
    {
        return (x <= v.x && y <= v.y && z <= v.z);
    }
    constexpr inline bool operator>=(const Vector3D& v) const noexcept
    {
        return (x >= v.x && y >= v.y && z >= v.z);
    }
    constexpr inline bool operator<(T v) const noexcept
    {
        return (x < v && y < v && z < v);
    }
    constexpr inline bool operator>(T v) const noexcept
    {
        return (x > v && y > v && z > v);
    }
    constexpr inline bool operator<=(T v) const noexcept
    {
        return (x <= v && y <= v && z <= v);
    }
    constexpr inline bool operator>=(T v) const noexcept
    {
        return (x >= v && y >= v && z >= v);
    }
    constexpr inline Vector3D& operator+=(const Vector3D& v) noexcept
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }
    constexpr inline Vector3D& operator-=(const Vector3D& v) noexcept
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }
    constexpr inline Vector3D& operator*=(const Vector3D& v) noexcept
    {
        x *= v.x;
        y *= v.y;
        z *= v.z;
        return *this;
    }
    constexpr inline Vector3D& operator/=(const Vector3D& v) noexcept
    {
        x /= v.x;
        y /= v.y;
        z /= v.z;
        return *this;
    }
    constexpr inline Vector3D& operator+=(T s) noexcept
    {
        x += s;
        y += s;
        z += s;
        return *this;
    }
    constexpr inline Vector3D& operator-=(T s) noexcept
    {
        x -= s;
        y -= s;
        z -= s;
        return *this;
    }
    constexpr inline Vector3D& operator*=(T s) noexcept
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    constexpr inline Vector3D& operator/=(T s) noexcept
    {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }
    constexpr inline Vector3D operator-() const noexcept
    {
        return Vector3D(-x, -y, -z);
    }

    T x, y, z;
};

template <class T>
std::ostream& operator<<(std::ostream& os, const Vector3D<T>& v)
{
    os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

using Vector3Df = Vector3D<float>;
using Vector3Du = Vector3D<unsigned int>;

struct PointAndDistance {
    Vector3Df point;
    float distance;
};
