#pragma once
#include <Eigen/Core>
#include <array>
#include <vector>
namespace flask {
struct Sphere { Eigen::Vector3d center=Eigen::Vector3d::Zero(); double radius=0; };
struct Box { Eigen::Vector3d center=Eigen::Vector3d::Zero(),half_size=Eigen::Vector3d::Zero(); };
enum class BodyKind { Sphere, Box2D };
struct BodyGeometry {
    BodyKind kind=BodyKind::Sphere;
    double radius=.1;
    Eigen::Vector2d half_size=Eigen::Vector2d::Zero();
};
inline constexpr double geometryTolerance=1e-10;
struct PositionBatch {
    std::array<Eigen::Vector3d,8> position;
    std::array<double,8> cosine,sine;
    unsigned active=0;
    PositionBatch() { for(auto &p:position) p.setZero();cosine.fill(1);sine.fill(0); }
};
bool avx2Available();
void validateGeometry(const std::vector<Sphere> &,const std::vector<Box> &,const BodyGeometry &);
unsigned collisionMask(const PositionBatch &,const std::vector<Sphere> &,const std::vector<Box> &,
                       const BodyGeometry &,bool useSIMD=true);
} // namespace flask
