
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>
#include "anim_keys_samples_clips.hpp"

// Constructors
Quat::Quat() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}  // identity quat

// Parameterized constructor
Quat::Quat(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}

// Lerp for Vec3
Vec3 lerp(const Vec3& a, const Vec3& b, float u) {
    return {
        a.x + (b.x - a.x) * u,
        a.y + (b.y - a.y) * u,
        a.z + (b.z - a.z) * u
    };
}

// Slerp for Quat
Quat slerp(const Quat& a, const Quat& b, float u, float threshold) {
    // Compute cosine of angle
    float dot = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
    Quat end = b;

    // If dot < 0, flip for shortest path
    if (dot < 0.0f) {
        dot = -dot;
        end = Quat(-b.x, -b.y, -b.z, -b.w);
    }

    if (dot > threshold) {
        // Very close: use linear interpolation
        Quat q(
            a.x + u * (end.x - a.x),
            a.y + u * (end.y - a.y),
            a.z + u * (end.z - a.z),
            a.w + u * (end.w - a.w)
        );
        q.normalize();
        return q;
    }

    float theta0 = std::acos(dot);
    float theta = theta0 * u;
    float sinTheta = std::sin(theta);
    float sinTheta0 = std::sin(theta0);

    float s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
    float s1 = sinTheta / sinTheta0;

    Quat q(
        s0 * a.x + s1 * end.x,
        s0 * a.y + s1 * end.y,
        s0 * a.z + s1 * end.z,
        s0 * a.w + s1 * end.w
    );
    return q;
}


// ------------------- Sampling -------------------

// Helper: find key indices around sample time
int findKeyIndex(const std::vector<float>& times, float t) {
    for (int i = 0; i < (int)times.size() - 1; ++i) {
        if (t < times[i+1]) return i;
    }
    return (int)times.size() - 2;
}

Quat sampleTrack(const TrackR& tr, float t) {
    if (tr.t.empty()) return {0,0,0,1};
    if (tr.t.size() == 1) return tr.v[0];

    int i0 = findKeyIndex(tr.t, t);
    int i1 = i0 + 1;
    float u = (t - tr.t[i0]) / (tr.t[i1] - tr.t[i0]);
    return slerp(tr.v[i0], tr.v[i1], u);
}

Vec3 sampleTrack(const Track3& tr, float t) {
    if (tr.t.empty()) return {0,0,0};
    if (tr.t.size() == 1) return tr.v[0];

    int i0 = findKeyIndex(tr.t, t);
    int i1 = i0 + 1;
    float u = (t - tr.t[i0]) / (tr.t[i1] - tr.t[i0]);
    return lerp(tr.v[i0], tr.v[i1], u);
}

// Sample a clip at normalized time
void sampleClip(const Clip& c, float tNorm, Quat& outRot, Vec3& outPos, Vec3& outScl) {
    float t = std::fmod(tNorm, 1.0f) * c.duration;
    outRot = sampleTrack(c.rot, t);
    outPos = sampleTrack(c.pos, t);
    outScl = sampleTrack(c.scl, t);
}

// ------------------- Test Program -------------------
int main() {
    Clip walk;
    walk.duration = 2.0f; // 2 seconds

    // Rotation track: 0s and 2s
    walk.rot.t = {0.0f, 2.0f};
    Quat q(0.0f, 0.0f, 0.0f, 1.0f); q.normalize();
    Quat r(0.0f, 0.0f, 0.7071f, 0.7071f); r.normalize();
    walk.rot.v = { q, r };

    // Translation: move along X
    walk.pos.t = {0.0f, 2.0f};
    walk.pos.v = {{0,0,0}, {2,0,0}};

    // Scale: uniform
    walk.scl.t = {0.0f, 2.0f};
    walk.scl.v = {{1,1,1}, {1,1,1}};

    std::cout << "Sampling walk clip...\n";
    for (float tn=0; tn<=1.0f; tn+=0.25f) {
        Quat r; Vec3 p,s;
        sampleClip(walk, tn, r, p, s);
        std::cout << "t=" << tn << "s "
                  << "Rot=("<<r.x<<","<<r.y<<","<<r.z<<","<<r.w<<") "
                  << "Pos=("<<p.x<<","<<p.y<<","<<p.z<<")\n";
    }
}
