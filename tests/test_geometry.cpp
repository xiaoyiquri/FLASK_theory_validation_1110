#include "flask/geometry.h"
#include "theory_test_support.h"
#include <random>
using namespace theory;
int main() {return run([] {
    flask::BodyGeometry rectangle;rectangle.kind=flask::BodyKind::Box2D;rectangle.half_size<<.3,.1;
    flask::PositionBatch poses;poses.active=1;
    std::vector<flask::Sphere> spheres;
    std::vector<flask::Box> boxes{{Eigen::Vector3d(.4,0,0),Eigen::Vector3d(.1,.1,0)}};
    auto hit=[&](bool simd){return flask::collisionMask(poses,spheres,boxes,rectangle,simd);};
    require(hit(false)==1 && hit(true)==1,"box contact is collision");
    boxes[0].center.x()+=1e-5;require(hit(false)==0 && hit(true)==0,"separated boxes remain clear");
    poses.cosine[0]=0;poses.sine[0]=1;boxes[0]={{0,.31,0},{.02,.02,0}};
    require(hit(false)==1 && hit(true)==1,"ninety-degree body pose used");
    boxes[0].center.y()=.35;require(hit(false)==0 && hit(true)==0,"rotated box extent respected");
    poses.cosine[0]=std::sqrt(.5);poses.sine[0]=std::sqrt(.5);boxes[0]={{.3,-.3,0},{.02,.02,0}};
    require(hit(false)==0 && hit(true)==0,"local separating axis rejects overlapping world bounding boxes");
    boxes.clear();poses.cosine[0]=1;poses.sine[0]=0;spheres={{{.35,0,0},.06}};
    require(hit(false)==1 && hit(true)==1,"box against circle contact");
    spheres={{{.34,.14,0},.05}};require(hit(false)==0 && hit(true)==0,"circle clears rectangle corner");
    // Actual Hole seed20 false positive from the former point/circle model.
    spheres.clear();boxes={{{5,3,0},{1.7,.15,0}},{{1,3,0},{1.7,.15,0}}};
    poses.position[0]<<3.072601155998915,2.811188597345026,0;
    poses.cosine[0]=std::cos(.0342943283689855);poses.sine[0]=std::sin(.0342943283689855);
    require(hit(false)==1 && hit(true)==1,"native Hole body overlaps right wall");
    flask::BodyGeometry circle;circle.radius=.1;
    require(flask::collisionMask(poses,spheres,boxes,circle,false)==0,"same pose small planning circle clears wall");
    for(int i=1;i<8;++i) {poses.position[i].setConstant(NAN);poses.cosine[i]=NAN;poses.sine[i]=NAN;}
    require(hit(false)==1 && hit(true)==1,"inactive NaN poses ignored");
    poses.active=0;require(hit(false)==0 && hit(true)==0,"zero activity mask remains empty");
    poses.active=1;poses.cosine[0]=NAN;require(hit(false)==1 && hit(true)==1,"active nonfinite orientation rejected");
    poses=flask::PositionBatch{};poses.active=1;poses.position[0]<<1.2,0,0;
    boxes={{{0,0,0},{1,1,1}}};circle.radius=.25;
    require(flask::collisionMask(poses,{},boxes,circle,false)==1,"sphere intersects AABB face");
    poses.position[0].x()=1.26;require(flask::collisionMask(poses,{},boxes,circle,true)==0,"sphere clears AABB face");
    std::mt19937 rng(9321);std::uniform_real_distribution<double> position(-3,3),angle(-3.14,3.14);
    boxes={{{0,0,0},{.7,.4,0}},{{1,1,0},{.1,.5,0}}};spheres={{{-1,.5,0},.3}};
    for(int k=0;k<1000;++k) {
        poses.active=rng()&255;
        for(int i=0;i<8;++i) {poses.position[i]<<position(rng),position(rng),0;const double a=angle(rng);poses.cosine[i]=std::cos(a);poses.sine[i]=std::sin(a);}
        require(hit(false)==hit(true),"random native-box scalar/SIMD differential");
        require(flask::collisionMask(poses,spheres,boxes,circle,false)==flask::collisionMask(poses,spheres,boxes,circle,true),"random sphere-box scalar/SIMD differential");
    }
    bool rejected=false;boxes[0].half_size.x()=-1;
    try {flask::collisionMask({},spheres,boxes,rectangle,false);} catch(const std::invalid_argument &){rejected=true;}
    require(rejected,"invalid world geometry rejected even without active samples");
    std::cout<<"native geometry AVX2="<<flask::avx2Available()<<'\n';
});}
