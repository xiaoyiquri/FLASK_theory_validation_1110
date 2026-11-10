#include "flask/geometry.h"
#include <immintrin.h>
namespace flask {
namespace {
struct V {
    __m256d v;
    explicit V(double x):v(_mm256_set1_pd(x)) {}
    explicit V(__m256d x):v(x) {}
    static V load(const double *x) { return V(_mm256_loadu_pd(x)); }
};
V operator+(V a,V b){return V(_mm256_add_pd(a.v,b.v));}
V operator-(V a,V b){return V(_mm256_sub_pd(a.v,b.v));}
V operator*(V a,V b){return V(_mm256_mul_pd(a.v,b.v));}
V operator<=(V a,V b){return V(_mm256_cmp_pd(a.v,b.v,_CMP_LE_OQ));}
V operator&(V a,V b){return V(_mm256_and_pd(a.v,b.v));}
V abs(V a){return V(_mm256_andnot_pd(_mm256_set1_pd(-0.),a.v));}
V positive(V a){return V(_mm256_max_pd(a.v,_mm256_setzero_pd()));}
unsigned bits(V a){return unsigned(_mm256_movemask_pd(a.v));}
}
unsigned collisionMaskAVX2(const PositionBatch &batch,const std::vector<Sphere> &spheres,const std::vector<Box> &boxes,const BodyGeometry &body) {
    unsigned result=0;
    for(int offset=0;offset<8;offset+=4) {
        double px[4],py[4],pz[4];
        for(int i=0;i<4;++i) {px[i]=batch.position[offset+i].x();py[i]=batch.position[offset+i].y();pz[i]=batch.position[offset+i].z();}
        const V x=V::load(px),y=V::load(py),z=V::load(pz),c=V::load(batch.cosine.data()+offset),s=V::load(batch.sine.data()+offset),ac=abs(c),as=abs(s),e(geometryTolerance);
        for(const auto &o:spheres) {
            const V dx=V(o.center.x())-x,dy=V(o.center.y())-y,dz=V(o.center.z())-z;V distance(0);double radius=o.radius+geometryTolerance;
            if(body.kind==BodyKind::Sphere) {distance=(dx*dx+dy*dy)+dz*dz;radius+=body.radius;}
            else {const V qx=positive(abs(dx*c+dy*s)-V(body.half_size.x())),qy=positive(abs(V(0)-dx*s+dy*c)-V(body.half_size.y()));distance=qx*qx+qy*qy;}
            result|=bits(distance<=V(radius*radius))<<offset;
        }
        for(const auto &o:boxes) {
            const V dx=V(o.center.x())-x,dy=V(o.center.y())-y,dz=V(o.center.z())-z;V hit(0);
            if(body.kind==BodyKind::Sphere) {
                const V qx=positive(abs(dx)-V(o.half_size.x())),qy=positive(abs(dy)-V(o.half_size.y())),qz=positive(abs(dz)-V(o.half_size.z()));
                const double radius=body.radius+geometryTolerance;hit=((qx*qx+qy*qy)+qz*qz)<=V(radius*radius);
            } else {
                const V a(body.half_size.x()),b(body.half_size.y()),hx(o.half_size.x()),hy(o.half_size.y());
                hit=(abs(dx)<=hx+a*ac+b*as+e)&(abs(dy)<=hy+a*as+b*ac+e)&(abs(dx*c+dy*s)<=a+hx*ac+hy*as+e)&(abs(V(0)-dx*s+dy*c)<=b+hx*as+hy*ac+e);
            }
            result|=bits(hit)<<offset;
        }
    }
    return result&batch.active;
}
} // namespace flask
