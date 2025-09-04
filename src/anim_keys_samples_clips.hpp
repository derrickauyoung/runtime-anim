#pragma once
#include <vector>

// ------------------- Math Types -------------------
struct Vec3 {
    float x, y, z;
    Vec3(float x = 0.0f, float y = 0.0f, float z = 0.0f) { this->x=x; this->y=y; this->z=z; }
};

struct Quat {
    float x, y, z, w;

    // Constructor for convenience
    Quat();
    Quat(float x, float y, float z, float w);

    // Normalize quaternion
    void normalize() {
        float len = std::sqrt(x*x + y*y + z*z + w*w);
        if (len > 0.0f) {
            x /= len; y /= len; z /= len; w /= len;
        } else {
            x = y = z = 0.0f; w = 1.0f; // Default to identity
        }
    }
};

// ------------------- Tracks -------------------
struct TrackR {
    std::vector<float> t;   // keyframe times
    std::vector<Quat> v;    // values
};

struct Track3 {
    std::vector<float> t;
    std::vector<Vec3> v;
};

// One clip for one joint (for simplicity)
struct Clip {
    float duration;
    TrackR rot;
    Track3 pos;
    Track3 scl;
};

// Sampling functions
Vec3 lerp(const Vec3& a, const Vec3& b, float u);
Quat slerp(const Quat& a, const Quat& b, float u, float threshold=0.9995f);
int findKeyIndex(const std::vector<float>& times, float t);
Quat sampleTrack(const TrackR& tr, float t);
Vec3 sampleTrack(const Track3& tr, float t);
void sampleClip(const Clip& c, float tNorm, Quat& outRot, Vec3& outPos, Vec3& outScl);