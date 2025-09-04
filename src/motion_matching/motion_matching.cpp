#include "motion_matching.hpp"

// Simple distance: sum of position differences
float poseDistance(const Pose& a, const Pose& b) {
    float dist = 0.0f;
    size_t n = std::min(a.positions.size(), b.positions.size());
    for (size_t i=0;i<n;i++)
        dist += distance(a.positions[i], b.positions[i]);
    return dist;
}

// Find the closest pose
Pose findBestMatch(const MotionDatabase& db, const Pose& query) {
    float bestDist = std::numeric_limits<float>::max();
    const Pose* bestPose = nullptr;
    for (const auto& pose : db.poses) {
        float d = poseDistance(pose, query);
        if (d < bestDist) {
            bestDist = d;
            bestPose = &pose;
        }
    }
    return bestPose ? *bestPose : Pose{};
}
