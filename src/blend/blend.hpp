#pragma once
#include <vector>

// ------------------- Math Types -------------------
struct Vec3 {
    float x,y,z;
};

struct Quat {
    float x,y,z,w;
    Quat(float x_=0,float y_=0,float z_=0,float w_=1);
    void normalize();
};

// ------------------- Tracks & Clips -------------------
struct TrackR { std::vector<float> t; std::vector<Quat> v; };
struct Track3 { std::vector<float> t; std::vector<Vec3> v; };
struct Clip   { float duration; TrackR rot; Track3 pos; Track3 scl; };

// ------------------- Sampling -------------------
int findKeyIndex(const std::vector<float>& times,float t);
Quat sampleTrack(const TrackR& tr,float t);
Vec3 sampleTrack(const Track3& tr,float t);
void sampleClip(const Clip& c,float tNorm, Quat& r, Vec3& p, Vec3& s);

// ------------------- Blending -------------------
Quat blendRot(const Quat& a, const Quat& b, float alpha);
Vec3 blendVec(const Vec3& a, const Vec3& b, float alpha);
