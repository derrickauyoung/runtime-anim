#include "motion_matching.hpp"

int main() {
    MotionDatabase db;

    // Create a simple database of 3 poses (1 joint each)
    db.poses = {
        {{{0,0,0}}, {{0,0,0,1}}},
        {{{1,0,0}}, {{0,0,0,1}}},
        {{{0,1,0}}, {{0,0,0,1}}}
    };

    Pose query{{{0.9f,0.1f,0}}, {{0,0,0,1}}};

    Pose best = findBestMatch(db, query);

    std::cout << "Best match position: ";
    for (auto& p : best.positions)
        std::cout << "(" << p.x << "," << p.y << "," << p.z << ") ";
    std::cout << "\n";

    return 0;
}
