#include "blend.hpp"
#include <iostream>

int main() {
    // --- Define two simple clips ---
    Clip clipA, clipB;
    clipA.duration = 2.0f;
    clipB.duration = 2.0f;

    // Rotation track: identity and 90° around Z
    clipA.rot.t = {0.0f, 2.0f};
    clipA.rot.v = { Quat(0,0,0,1), Quat(0,0,0.7071f,0.7071f) };

    clipB.rot.t = {0.0f, 2.0f};
    clipB.rot.v = { Quat(0,0,0,1), Quat(0,0.7071f,0,0.7071f) }; // 90° around Y

    // Translation: move along X for clipA, along Y for clipB
    clipA.pos.t = {0.0f,2.0f};
    clipA.pos.v = { {0,0,0}, {2,0,0} };

    clipB.pos.t = {0.0f,2.0f};
    clipB.pos.v = { {0,0,0}, {0,2,0} };

    // Scale: uniform
    clipA.scl.t = clipB.scl.t = {0.0f,2.0f};
    clipA.scl.v = clipB.scl.v = { {1,1,1}, {1,1,1} };

    std::cout << "Module 4 Blending Demo:\n";

    for(float tNorm=0.0f; tNorm<=1.0f; tNorm+=0.25f) {
        Quat rA,rB,rBlend;
        Vec3 pA,pB,pBlend,sA,sB,sBlend;

        // Sample each clip
        sampleClip(clipA, tNorm, rA, pA, sA);
        sampleClip(clipB, tNorm, rB, pB, sB);

        // Blend 50/50
        rBlend = blendRot(rA,rB,0.5f);
        pBlend = blendVec(pA,pB,0.5f);
        sBlend = blendVec(sA,sB,0.5f);

        std::cout << "tNorm=" << tNorm << "\n";
        std::cout << "  ClipA Pos=("<<pA.x<<","<<pA.y<<","<<pA.z<<") Rot=("
                  <<rA.x<<","<<rA.y<<","<<rA.z<<","<<rA.w<<")\n";
        std::cout << "  ClipB Pos=("<<pB.x<<","<<pB.y<<","<<pB.z<<") Rot=("
                  <<rB.x<<","<<rB.y<<","<<rB.z<<","<<rB.w<<")\n";
        std::cout << "  Blend Pos=("<<pBlend.x<<","<<pBlend.y<<","<<pBlend.z<<") Rot=("
                  <<rBlend.x<<","<<rBlend.y<<","<<rBlend.z<<","<<rBlend.w<<")\n\n";
    }

    return 0;
}
