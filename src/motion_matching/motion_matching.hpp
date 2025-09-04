#pragma once
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>

// --- Simple math types ---
struct Vec3 { float x,y,z; };
struct Quat { float x,y,z,w; };

// Euclidean distance for Vec3
inline float distance(const Vec3& a, const Vec3& b) {
    float dx = a.x - b.x;
    float dy = a.y - b.y;
    float dz = a.z - b.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// Pose with positions and rotations per joint
struct Pose {
    std::vector<Vec3> positions;
    std::vector<Quat> rotations;
};

// Motion database storing poses
struct MotionDatabase {
    std::vector<Pose> poses;
};

// --- Motion Matching ---
Pose findBestMatch(const MotionDatabase& db, const Pose& query);
float poseDistance(const Pose& a, const Pose& b);
