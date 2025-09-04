#define CATCH_CONFIG_MAIN
#include "include/catch_amalgamated.hpp"

// Include your animation library headers
#include "anim_keys_samples_clips.hpp"  // <- contains Vec3, Quat, Clip, sampleClip, etc.

TEST_CASE("Clip sampling works for rotation & translation","[animation]") {
    Clip walk;
    walk.duration = 2.0f;

    // Rotation
    walk.rot.t = {0.0f, 2.0f};
    Quat q(0,0,0,1); q.normalize();
    Quat r(0,0,0.7071f,0.7071f); r.normalize();
    walk.rot.v = {q,r};

    // Translation
    walk.pos.t = {0.0f,2.0f};
    walk.pos.v = {{0,0,0},{2,0,0}};

    // Scale
    walk.scl.t = {0.0f,2.0f};
    walk.scl.v = {{1,1,1},{1,1,1}};

    SECTION("At t=0.0 we are at start pose") {
        Quat rot; Vec3 pos,scl;
        sampleClip(walk,0.0f,rot,pos,scl);
        REQUIRE(pos.x == Catch::Approx(0.0f));
        REQUIRE(rot.w == Catch::Approx(1.0f)); // identity quat
    }

    SECTION("At t=0.5 we are halfway") {
        Quat rot; Vec3 pos,scl;
        sampleClip(walk,0.5f,rot,pos,scl);
        REQUIRE(pos.x == Catch::Approx(1.0f));
        // halfway between identity and 90° about Z ≈ 45°
        // so w ≈ cos(22.5°) ≈ 0.9239
        REQUIRE(rot.w == Catch::Approx(0.9239).margin(0.001));
    }

    SECTION("At t=1.0 we are at end pose") {
        Quat rot; Vec3 pos,scl;
        sampleClip(walk,1.0f,rot,pos,scl);
        REQUIRE(pos.x == Catch::Approx(2.0f));
        REQUIRE(rot.z == Catch::Approx(0.7071).margin(0.001));
        REQUIRE(rot.w == Catch::Approx(0.7071).margin(0.001));
    }
}
