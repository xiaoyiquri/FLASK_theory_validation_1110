#include "fixtures.h"
#include <random>
using namespace theory;
int main() { return run([] {
    auto native=fixture<2,2>(flask::Robot::Quad2D);
    native.body.kind=flask::BodyKind::Box2D;native.body.half_size<<.3,.1;
    native.boxes={{{5,3,0},{1.7,.15,0}},{{1,3,0},{1.7,.15,0}}};
    flask::Model<2,2> nativeModel(native);
    flask::BVP<2,2>::Curve nativeCurve;nativeCurve.T=.1;
    nativeCurve.coefficients.col(0)<<3.072601155998915,2.811188597345026;
    nativeCurve.coefficients(0,2)=-9.81*std::tan(.0342943283689855)/2;
    require(nativeModel.nodeValid(nativeCurve.evaluateState(0)),"r2 node is not assigned a fictitious horizontal attitude");
    for(bool simd:{false,true}) {
        flask::ValidationConfig v;v.simd=simd;
        flask::Validator<2,2> validator(nativeModel,v);
        require(!validator.edge(nativeCurve) && validator.stats.collision_rejections==1,
                "full edge native body rejects Hole seed20 pose on both backends");
    }
    auto circle=native;circle.body.kind=flask::BodyKind::Sphere;circle.body.radius=.1;
    flask::Model<2,2> circleModel(circle);flask::Validator<2,2> circleValidator(circleModel);
    require(circleValidator.edge(nativeCurve),"old small circle clears the same physically valid curve");
    auto stationary=native;
    stationary.physical_start=Eigen::VectorXd::Zero(6);
    stationary.physical_start.head<2>()=nativeCurve.coefficients.col(0);
    stationary.physical_goal=stationary.physical_start;
    flask::Model<2,2> stationaryModel(stationary);flask::Validator<2,2> stationaryValidator(stationaryModel);
    const auto stationaryFlat=nativeCurve.evaluateState(0);
    require(!stationaryValidator.path({},stationaryFlat,stationaryFlat),"zero-duration reduced path cannot bypass native-body collision");
    bool unsupported=false;
    try {flask::ValidationConfig v;v.continuous_sphere_collision=true;flask::Validator<2,2> invalid(nativeModel,v);}
    catch(const std::invalid_argument &) {unsupported=true;}
    require(unsupported,"sphere-only certificate cannot silently certify native boxes");
    flask::PositionBatch initialized;
    for(const auto &p:initialized.position) require(p.isZero(0),"inactive lanes and unused spatial axes initialized");
    auto c=fixture<1,1>(); c.lower(0)=0; c.upper(0)=1;
    c.obstacles={{Eigen::Vector3d(.05,0,0),.001}};
    flask::Model<1,1> model(c); flask::BVP<1,1> bvp;
    flask::BVP<1,1>::State a,b; b.values(0)=1;
    const auto curve=bvp.fixed(a,b,1); require(bool(curve),"linear thin obstacle trajectory");
    flask::ValidationConfig cfg; cfg.step=.1;
    flask::Validator<1,1> discrete(model,cfg);
    require(discrete.edge(*curve),"actual finite validator reproduces tunneling");
    cfg.continuous_sphere_collision=true;
    flask::Validator<1,1> continuous(model,cfg);
    require(!continuous.edge(*curve),"Bernstein hull subdivision rejects between-sample collision");
    c.obstacles.clear(); flask::Model<1,1> clear(c); flask::Validator<1,1> safe(clear,cfg);
    require(safe.edge(*curve),"continuous check accepts separated curve");
    auto normalized=*curve;
    normalized.T=.1; normalized.normalized=true;
    // Same geometric interval, duration-scaled derivatives. The Bernstein
    // hull must use normalized coefficients rather than multiplying by T.
    cfg.step=.02;
    flask::Validator<1,1> normalizedValidator(model,cfg);
    require(!normalizedValidator.edge(normalized),"normalized-time hull cannot miss the thin obstacle");
    cfg.max_samples=2; cfg.step=.01; flask::Validator<1,1> capped(clear,cfg);
    require(!capped.edge(*curve),"sample limit rejects instead of coarsening silently");
    require(capped.stats.sample_budget_rejections==1,"sample cap is a resource rejection, not a physical violation");
    flask::BVP<1,1>::Curve outside;outside.T=1;outside.coefficients(0,0)=20;
    flask::Validator<1,1> bounded(clear);
    require(!bounded.edge(outside) && bounded.stats.flat_bound_rejections>0 &&
            bounded.stats.reasons[static_cast<std::size_t>(rrtpff::FeasibilityFailure::Numeric)]==0,
            "finite workspace-bound violations are separate from Numeric errors");
    std::mt19937 rng(19); std::uniform_real_distribution<double> distribution(-2,2);
    std::vector<flask::Sphere> spheres{{Eigen::Vector3d(0,0,0),.3},{Eigen::Vector3d(1,.2,0),.2}};
    for(int iteration=0;iteration<200;++iteration) {
        flask::PositionBatch batch; batch.active=unsigned(iteration)%256;
        for(auto &p:batch.position) p << distribution(rng),distribution(rng),distribution(rng);
        require(flask::collisionMask(batch,spheres,{},flask::BodyGeometry{},false)==flask::collisionMask(batch,spheres,{},flask::BodyGeometry{},true),
                "scalar/AVX2 collision parity including inactive lanes");
    }
    flask::PositionBatch nonfinite; nonfinite.active=1; nonfinite.position[0].x()=NAN;
    require(flask::collisionMask(nonfinite,{},{},flask::BodyGeometry{},true)==1,"nonfinite position must be rejected");
    // Exercise the complete grid, including endpoints and partially active
    // batches. Previously (T*i)/intervals could exceed T by one rounding step.
    auto gridConfig=fixture<1,1>();
    flask::Model<1,1> gridModel(gridConfig);
    flask::ValidationConfig scalarConfig; scalarConfig.simd=false;
    scalarConfig.step=.07;
    auto vectorConfig=scalarConfig; vectorConfig.simd=true;
    flask::Validator<1,1> scalarValidator(gridModel,scalarConfig),vectorValidator(gridModel,vectorConfig);
    for(int i=1;i<=150;++i) {
        auto edge=bvp.fixed(a,b,.4+.01*i);
        require(edge && scalarValidator.edge(*edge),"scalar endpoint-grid edge accepted");
        require(vectorValidator.edge(*edge),"SIMD endpoint-grid edge accepted without time overshoot");
    }
    flask::BVP<1,1>::Curve tiny; tiny.T=std::numeric_limits<double>::denorm_min();
    require(scalarValidator.edge(tiny) && vectorValidator.edge(tiny),"positive subnormal duration has both endpoint samples");
    if(flask::avx2Available()) require(vectorValidator.stats.simd_batches>0,"full SIMD validator exercised");
    std::cout<<"AVX2 available="<<flask::avx2Available()<<'\n';
}); }
