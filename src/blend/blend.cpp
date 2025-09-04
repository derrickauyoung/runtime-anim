#include "blend.hpp"
#include <cmath>
#include <algorithm>

// ------------------- Math -------------------
Quat::Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
void Quat::normalize() {
    float len = std::sqrt(x*x + y*y + z*z + w*w);
    if(len>0){ x/=len; y/=len; z/=len; w/=len; }
    else { x=y=z=0; w=1; }
}

Vec3 lerp(const Vec3& a, const Vec3& b, float u) {
    return {a.x + (b.x - a.x)*u, a.y + (b.y - a.y)*u, a.z + (b.z - a.z)*u};
}

Quat slerp(const Quat& a, const Quat& b, float u, float threshold=0.9995f) {
    float dot = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
    Quat end = b;
    if(dot < 0){ dot=-dot; end = {-b.x,-b.y,-b.z,-b.w}; }
    if(dot > threshold){
        Quat q = {a.x + u*(end.x - a.x),
                  a.y + u*(end.y - a.y),
                  a.z + u*(end.z - a.z),
                  a.w + u*(end.w - a.w)};
        q.normalize(); return q;
    }
    float theta0 = std::acos(dot);
    float theta  = theta0 * u;
    float sinTheta = std::sin(theta), sinTheta0 = std::sin(theta0);
    float s0 = std::cos(theta) - dot*sinTheta/sinTheta0;
    float s1 = sinTheta / sinTheta0;
    Quat q = {s0*a.x + s1*end.x, s0*a.y + s1*end.y, s0*a.z + s1*end.z, s0*a.w + s1*end.w};
    return q;
}

// ------------------- Sampling -------------------
int findKeyIndex(const std::vector<float>& times,float t){
    for(int i=0;i<(int)times.size()-1;++i) if(t<times[i+1]) return i;
    return (int)times.size()-2;
}

Quat sampleTrack(const TrackR& tr,float t){
    if(tr.t.empty()) return {0,0,0,1};
    if(tr.t.size()==1) return tr.v[0];
    int i0=findKeyIndex(tr.t,t), i1=i0+1;
    float u=(t-tr.t[i0])/(tr.t[i1]-tr.t[i0]);
    return slerp(tr.v[i0],tr.v[i1],u);
}

Vec3 sampleTrack(const Track3& tr,float t){
    if(tr.t.empty()) return {0,0,0};
    if(tr.t.size()==1) return tr.v[0];
    int i0=findKeyIndex(tr.t,t), i1=i0+1;
    float u=(t-tr.t[i0])/(tr.t[i1]-tr.t[i0]);
    return lerp(tr.v[i0],tr.v[i1],u);
}

void sampleClip(const Clip& c,float tNorm, Quat& r, Vec3& p, Vec3& s){
    float t = std::fmod(tNorm,1.0f) * c.duration;
    r = sampleTrack(c.rot,t);
    p = sampleTrack(c.pos,t);
    s = sampleTrack(c.scl,t);
}

// ------------------- Blending -------------------
Quat blendRot(const Quat& a, const Quat& b, float alpha){
    return slerp(a,b,alpha);
}

Vec3 blendVec(const Vec3& a, const Vec3& b, float alpha){
    return lerp(a,b,alpha);
}
