#include "CutterAPT.h"

#include <algorithm>

OBB3Df CutterAPT::getBBox(const Posture& currentPosture) const
{
    return createBBox(currentPosture, R, totalHeight());
}

bool CutterAPT::isInside(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);

    if (z < 0 || z > totalHeight()) {
        return false;
    }

    if (z <= Hlc) {
        return x <= distanceToZAxisOnLowerConeSurface(z);
    }

    if (z <= Hlc + Ht) {
        return x <= distanceToZAxisOnTorusBallSurface(z);
    }

    if (z <= h) {
        return x <= distanceToZAxisOnUpperConeSurface(z);
    }

    return x <= R;
}

float CutterAPT::signedDistance(const Posture& currentPosture, const Vector3Df& p) const
{
    float x = distanceToZAxis(currentPosture, p);
    float z = projectToZAxisLength(currentPosture, p);

    if (z >= totalHeight()) {
        return signedDistanceToTopFlat(x, z);
    }

    if (z >= h) {
        return R - x;
    }

    float d1 = distanceToLowerConeSurface(x, z);
    float d2 = distanceToCornerTorusBallSurface(x, z);
    float d3 = distanceToUpperConeSurface(x, z);
    float d4 = distanceToCylinderSurface(x, z);
    float d5 = signedDistanceToTopFlat(x, z);

    float d = std::min({ d1, d2, d3, d4, std::abs(d5) });
    return isInside(currentPosture, p) ? d : -d;
}

#define PI acos(-1)
class PoinOfCutterO {
public:
    PoinOfCutterO() {};
    PoinOfCutterO(float theta, float b, float h, float R)
    {
        x = b * R * cos(theta);
        y = b * R * sin(theta);
        z = h;
        x_normal = 0;
        y_normal = 0;
        z_normal = 1;
    };
    float x;
    float y;
    float z;
    float x_normal;
    float y_normal;
    float z_normal;
};

class PoinOfCutterU {
public:
    PoinOfCutterU() {};
    PoinOfCutterU(float beta, float theta, float h, float f, float e, float r, float a)
    {
        x = (e + r * cos(beta)) * cos(theta) + a * (h - f + r * sin(beta)) * tan(beta) * cos(theta);
        y = (e + r * cos(beta)) * sin(theta) + a * (h - f + r * sin(beta)) * tan(beta) * sin(theta);
        z = f - r * sin(beta) + a * (h - f + r * sin(beta));
        x_normal = cos(theta) * (h - f + r * sin(beta)) * (e + r * cos(beta) + a * (h - f + r * sin(beta)) * tan(beta));
        y_normal = sin(theta) * (h - f + r * sin(beta)) * (e + r * cos(beta) + a * (h - f + r * sin(beta)) * tan(beta));
        z_normal = -(e + r * cos(beta) + a * (h - f + r * sin(beta)) * tan(beta)) * (h - f + r * sin(beta)) * tan(beta);
    };
    float x;
    float y;
    float z;
    float x_normal;
    float y_normal;
    float z_normal;
};

class PoinOfCutterT {
public:
    PoinOfCutterT() {};
    PoinOfCutterT(float phi, float theta, float f, float e, float r)
    {
        x = (e + r * cos(phi)) * cos(theta);
        y = (e + r * cos(phi)) * sin(theta);
        z = f - r * sin(phi);
        x_normal = r * cos(phi) * cos(theta) * (e + r * cos(phi));
        y_normal = r * cos(phi) * sin(theta) * (e + r * cos(phi));
        z_normal = -r * sin(phi) * (e + r * cos(phi));
    };
    float x;
    float y;
    float z;
    float x_normal;
    float y_normal;
    float z_normal;
};

class PoinOfCutterL {
public:
    PoinOfCutterL() {};
    PoinOfCutterL(float alpha, float theta, float e, float r, float b)
    {
        x = b * (e + r * sin(alpha)) * cos(theta);
        y = b * (e + r * sin(alpha)) * sin(theta);
        z = b * (e + r * sin(alpha)) * tan(alpha);
        x_normal = b * (e + r * sin(alpha)) * (e + r * sin(alpha)) * tan(alpha) * cos(theta);
        y_normal = b * (e + r * sin(alpha)) * (e + r * sin(alpha)) * tan(alpha) * sin(theta);
        z_normal = -b * (e + r * sin(alpha)) * (e + r * sin(alpha));
    };
    float x;
    float y;
    float z;
    float x_normal;
    float y_normal;
    float z_normal;
};

void CutterAPT::generateMesh(std::vector<Vector3Df>& vertices, std::vector<Vector3Df>& normals) const
{
    vertices.clear();
    normals.clear();
    // 生成顶点数据
    // 遍历参数范围，计算顶点坐标并发送顶点数据
    int stepTheta = 50; // theta 参数范围
    int stepA = 20; // a 参数范围
    int stepPhi = 20; // phi 参数值
    int stepB = 20; // b 参数范围
    float thetaStep = (2 * PI )/ stepTheta; // theta的步长
    float aStep = 1.0f / stepA; // a的步长
    float phiStep = (PI / 2 - alpha - beta) / stepPhi; // phi的步长
    float bStep = 1.0f / stepB; // b的步长
    float a = 1.0f; // a 初始值
    float theta = 0.0f; // theta 初始值
    float phi = beta; // phi的初始值
    float b = 0.0f; // b 初始值
    
    // 参数约束关系
    const float f1 = r/cos(alpha)+e*tan(alpha);  // f的参数值，是固定的，根据e、r、alpha得出
    const float R = e + r * cos(beta) + (h - f1 + r * sin(beta)) * tan(beta); // R的参数值，是固定的，根据e、r、alpha、beta、h、f1得出
    for(int stepb = 0; stepb < stepB; stepb++) {
        for(int steptheta = 0; steptheta < stepTheta; steptheta++){
            PoinOfCutterO point1 = PoinOfCutterO(theta,b,h,R);
            PoinOfCutterO point2 = PoinOfCutterO(theta,b + bStep,h,R);
            PoinOfCutterO point3 = PoinOfCutterO(theta + thetaStep,b + bStep,h,R);
            PoinOfCutterO point4 = PoinOfCutterO(theta + thetaStep,b,h,R);
            vertices.push_back({ point1.x, point1.y, point1.z });
            normals.push_back({ point1.x_normal, point1.y_normal, point1.z_normal });
            vertices.push_back({ point2.x, point2.y, point2.z });
            normals.push_back({ point2.x_normal, point2.y_normal, point2.z_normal });
            vertices.push_back({ point3.x, point3.y, point3.z });
            normals.push_back({ point3.x_normal, point3.y_normal, point3.z_normal });

            vertices.push_back({ point1.x, point1.y, point1.z });
            normals.push_back({ point1.x_normal, point1.y_normal, point1.z_normal });
            vertices.push_back({ point3.x, point3.y, point3.z });
            normals.push_back({ point3.x_normal, point3.y_normal, point3.z_normal });
            vertices.push_back({ point4.x, point4.y, point4.z });
            normals.push_back({ point4.x_normal, point4.y_normal, point4.z_normal });
            theta += thetaStep;
        }
        b += bStep;
        theta = 0.0f;
    }
    for (int stepa = 0; stepa < stepA; stepa++) {
        for (int steptheta = 0; steptheta < stepTheta; steptheta++) {
            PoinOfCutterU point1 = PoinOfCutterU(beta, theta, h, f1, e, r, a);
            PoinOfCutterU point2 = PoinOfCutterU(beta, theta, h, f1, e, r, a - aStep);
            PoinOfCutterU point3 = PoinOfCutterU(beta, theta + thetaStep, h, f1, e, r, a - aStep);
            PoinOfCutterU point4 = PoinOfCutterU(beta, theta + thetaStep, h, f1, e, r, a);
            vertices.push_back({ point1.x, point1.y, point1.z });
            normals.push_back({ point1.x_normal, point1.y_normal, point1.z_normal });
            vertices.push_back({ point2.x, point2.y, point2.z });
            normals.push_back({ point2.x_normal, point2.y_normal, point2.z_normal });
            vertices.push_back({ point3.x, point3.y, point3.z });
            normals.push_back({ point3.x_normal, point3.y_normal, point3.z_normal });
            vertices.push_back({ point1.x, point1.y, point1.z });
            normals.push_back({ point1.x_normal, point1.y_normal, point1.z_normal });
            vertices.push_back({ point3.x, point3.y, point3.z });
            normals.push_back({ point3.x_normal, point3.y_normal, point3.z_normal });
            vertices.push_back({ point4.x, point4.y, point4.z });
            normals.push_back({ point4.x_normal, point4.y_normal, point4.z_normal });
            theta += thetaStep;
        }
        a -= aStep;
        theta = 0.0f;
    }
    theta = 0.0f; // theta恢复初始值
    for (int stepphi = 0; stepphi < stepPhi; stepphi++) {
        for (int steptheta = 0; steptheta < stepTheta; steptheta++) {
            PoinOfCutterT point5 = PoinOfCutterT(phi, theta, f1, e, r);
            PoinOfCutterT point6 = PoinOfCutterT(phi + phiStep, theta, f1, e, r);
            PoinOfCutterT point7 = PoinOfCutterT(phi + phiStep, theta + thetaStep, f1, e, r);
            PoinOfCutterT point8 = PoinOfCutterT(phi, theta + thetaStep, f1, e, r);
            vertices.push_back({ point5.x, point5.y, point5.z });
            normals.push_back({ point5.x_normal, point5.y_normal, point5.z_normal });
            vertices.push_back({ point6.x, point6.y, point6.z });
            normals.push_back({ point6.x_normal, point6.y_normal, point6.z_normal });
            vertices.push_back({ point7.x, point7.y, point7.z });
            normals.push_back({ point7.x_normal, point7.y_normal, point7.z_normal });
            vertices.push_back({ point5.x, point5.y, point5.z });
            normals.push_back({ point5.x_normal, point5.y_normal, point5.z_normal });
            vertices.push_back({ point7.x, point7.y, point7.z });
            normals.push_back({ point7.x_normal, point7.y_normal, point7.z_normal });
            vertices.push_back({ point8.x, point8.y, point8.z });
            normals.push_back({ point8.x_normal, point8.y_normal, point8.z_normal });
            theta += thetaStep;
        }
        phi += phiStep;
        theta = 0.0f;
    }
    theta = 0.0f;
    for (int stepb = 0; stepb < stepB; stepb++) {
        for (int steptheta = 0; steptheta < stepTheta; steptheta++) {
            PoinOfCutterL point9 = PoinOfCutterL(alpha, theta, e, r, b);
            PoinOfCutterL point10 = PoinOfCutterL(alpha, theta, e, r, b - bStep);
            PoinOfCutterL point11 = PoinOfCutterL(alpha, theta + thetaStep, e, r, b - bStep);
            PoinOfCutterL point12 = PoinOfCutterL(alpha, theta + thetaStep, e, r, b);
            vertices.push_back({ point9.x, point9.y, point9.z });
            normals.push_back({ point9.x_normal, point9.y_normal, point9.z_normal });
            vertices.push_back({ point10.x, point10.y, point10.z });
            normals.push_back({ point10.x_normal, point10.y_normal, point10.z_normal });
            vertices.push_back({ point11.x, point11.y, point11.z });
            normals.push_back({ point11.x_normal, point11.y_normal, point11.z_normal });
            vertices.push_back({ point9.x, point9.y, point9.z });
            normals.push_back({ point9.x_normal, point9.y_normal, point9.z_normal });
            vertices.push_back({ point11.x, point11.y, point11.z });
            normals.push_back({ point11.x_normal, point11.y_normal, point11.z_normal });
            vertices.push_back({ point12.x, point12.y, point12.z });
            normals.push_back({ point12.x_normal, point12.y_normal, point12.z_normal });
            theta += thetaStep;
        }
        b -= bStep;
        theta = 0.0f;
    }
}
