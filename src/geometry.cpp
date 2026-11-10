#include "flask/geometry.h"
#include <cmath>
#include <stdexcept>
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#elif (defined(__GNUC__) || defined(__clang__)) && (defined(__x86_64__) || defined(__i386__))
#include <cpuid.h>
#endif
namespace flask {
#ifdef FLASK_HAS_AVX2
unsigned collisionMaskAVX2(const PositionBatch &,const std::vector<Sphere> &,const std::vector<Box> &,const BodyGeometry &);
#endif
static bool detectAVX2() {
#if defined(FLASK_HAS_AVX2) && defined(_MSC_VER)
    int regs[4]; __cpuid(regs,0); if(regs[0]<7) return false;
    __cpuid(regs,1);
    if(!(regs[2]&(1<<27)) || !(regs[2]&(1<<28)) || (_xgetbv(0)&6)!=6) return false;
    __cpuidex(regs,7,0); return (regs[1]&(1<<5))!=0;
#elif defined(FLASK_HAS_AVX2) && (defined(__x86_64__) || defined(__i386__))
    // Direct CPUID also works with compiler-rt Windows toolchains that do
    // not supply GCC's __cpu_model runtime. Check OS YMM state first.
    if(__get_cpuid_max(0,nullptr)<7) return false;
    unsigned a,b,c,d; __cpuid_count(1,0,a,b,c,d);
    if(!(c&(1u<<27)) || !(c&(1u<<28))) return false;
    unsigned low,high;
    __asm__ volatile("xgetbv" : "=a"(low),"=d"(high) : "c"(0));
    if((low&6u)!=6u) return false;
    __cpuid_count(7,0,a,b,c,d); return (b&(1u<<5))!=0;
#else
    return false;
#endif
}
bool avx2Available() { static const bool supported=detectAVX2(); return supported; }
void validateGeometry(const std::vector<Sphere> &spheres,const std::vector<Box> &boxes,const BodyGeometry &body) {
    if(body.kind==BodyKind::Sphere) {
        if(!std::isfinite(body.radius)||body.radius<0) throw std::invalid_argument("Invalid sphere robot radius");
    } else if(body.kind==BodyKind::Box2D) {
        if(!body.half_size.allFinite() || (body.half_size.array()<=0).any()) throw std::invalid_argument("Invalid box robot dimensions");
    } else throw std::invalid_argument("Unknown robot geometry");
    for(const auto &s:spheres) {
        if(!s.center.allFinite() || !std::isfinite(s.radius) || s.radius<0) throw std::invalid_argument("Invalid obstacle sphere");
        if(body.kind==BodyKind::Box2D && std::abs(s.center.z())>geometryTolerance) throw std::invalid_argument("Box2D requires planar obstacles");
    }
    for(const auto &b:boxes) {
        if(!b.center.allFinite() || !b.half_size.allFinite() || (b.half_size.array()<0).any() || b.half_size.maxCoeff()<=0)
            throw std::invalid_argument("Invalid obstacle box");
        if(body.kind==BodyKind::Box2D && (std::abs(b.center.z())>geometryTolerance || b.half_size.z()!=0))
            throw std::invalid_argument("Box2D requires planar obstacle boxes");
    }
}
unsigned collisionMask(const PositionBatch &batch,const std::vector<Sphere> &spheres,
                       const std::vector<Box> &boxes,const BodyGeometry &body,bool simd) {
    validateGeometry(spheres,boxes,body);
    if(batch.active>255) throw std::invalid_argument("Invalid pose activity mask");
    unsigned mask=0;
    for(int i=0;i<8;++i) if(batch.active&(1u<<i)) {
        bool valid=batch.position[i].allFinite();
        if(body.kind==BodyKind::Box2D) {
            const double c=batch.cosine[i],s=batch.sine[i];
            valid=valid && std::isfinite(c) && std::isfinite(s) && std::abs(c*c+s*s-1)<=1e-8 && std::abs(batch.position[i].z())<=geometryTolerance;
        }
        if(!valid) mask|=1u<<i;
    }
#ifdef FLASK_HAS_AVX2
    if(simd && avx2Available()) return mask|collisionMaskAVX2(batch,spheres,boxes,body);
#else
    (void)simd;
#endif
    for(int i=0;i<8;++i) if((batch.active&(1u<<i)) && !(mask&(1u<<i))) {
        const double c=batch.cosine[i],s=batch.sine[i];
        for(const auto &o:spheres) {
            const Eigen::Vector3d d=o.center-batch.position[i];
            double distance=0,radius=o.radius+geometryTolerance;
            if(body.kind==BodyKind::Sphere) { distance=(d.x()*d.x()+d.y()*d.y())+d.z()*d.z();radius+=body.radius; }
            else {
                const double x=std::max(0.,std::abs(d.x()*c+d.y()*s)-body.half_size.x());
                const double y=std::max(0.,std::abs(-d.x()*s+d.y()*c)-body.half_size.y());distance=x*x+y*y;
            }
            if(distance<=radius*radius) { mask|=1u<<i;break; }
        }
        if(mask&(1u<<i)) continue;
        for(const auto &o:boxes) {
            const Eigen::Vector3d d=o.center-batch.position[i];bool hit=false;
            if(body.kind==BodyKind::Sphere) {
                const Eigen::Vector3d q=(d.cwiseAbs()-o.half_size).cwiseMax(0.);
                const double radius=body.radius+geometryTolerance;hit=(q.x()*q.x()+q.y()*q.y())+q.z()*q.z()<=radius*radius;
            } else {
                const double a=body.half_size.x(),b=body.half_size.y(),hx=o.half_size.x(),hy=o.half_size.y(),ac=std::abs(c),as=std::abs(s),e=geometryTolerance;
                hit=std::abs(d.x())<=hx+a*ac+b*as+e && std::abs(d.y())<=hy+a*as+b*ac+e &&
                    std::abs(d.x()*c+d.y()*s)<=a+hx*ac+hy*as+e && std::abs(-d.x()*s+d.y()*c)<=b+hx*as+hy*ac+e;
            }
            if(hit) { mask|=1u<<i;break; }
        }
    }
    return mask;
}
} // namespace flask
