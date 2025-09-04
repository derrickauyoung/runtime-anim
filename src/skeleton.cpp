#include <iostream>
#include <vector>
#include <span>
#include <cmath>
#include <algorithm>

// To compile: g++ -std=c++20 -o ../bin/transform_test skeleton.cpp

// Core data structures for skeletal animation
struct Quat {
    float x,y,z,w;

    // Constructor for convenience
    Quat(float x = 0.0f, float y = 0.0f, float z = 0.0f, float w = 1.0f)
        : x(x), y(y), z(z), w(w) {}

    // Normalize the quaternion to unit length
    void normalize() {
        float len = std::sqrt(x*x + y*y + z*z + w*w);
        if (len > 0.0f) {
            x /= len; y /= len; z /= len; w /= len;
        } else {
            x = y = z = 0.0f; w = 1.0f; // Default to identity
        }
    }
};

struct Vec3 {
    float x,y,z;

    // Constructor for convenience
    Vec3(float x = 0.0f, float y = 0.0f, float z = 0.0f)
        : x(x), y(y), z(z) {}
};

struct Transform {
    Quat r;  // rotation
    Vec3 t;  // translation
    Vec3 s;  // scale

    // Constructor for convenience
    Transform(const Quat& r = Quat(),
                const Vec3& t = Vec3(),
                const Vec3& s = Vec3(1.0f, 1.0f, 1.0f))
        : r(r), t(t), s(s) {}
};

struct Skeleton {
    std::vector<int16_t> parent; // -1 for root
    // bind pose in local space
    std::vector<Transform> bindLocal;
};

Quat multiplyQuaternions(const Quat& a, const Quat& b) {
    // Quaternion multiplication follows:
    // (a.w + a.x*i + a.y*j + a.z*k) * 
    // (b.w + b.x*i + b.y*j + b.z*k)
    // Where i*i = j*j = k*k = -1,
    // i*j = k, j*k = i, k*i = j
    // and j*i = -k, k*j = -i, i*k = -j
    
    return Quat{
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,  // i component
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,  // j component
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,  // k component
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z   // scalar component
    };
};

Vec3 crossProduct(const Vec3& a, const Vec3& b) {
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

Vec3 rotateVector(const Quat& q, const Vec3& v) {
    // To rotate vector v by quaternion q, we compute: q * v * q^(-1)
    // But there's a more efficient formula that avoids creating intermediate quaternions
    
    // Extract the vector part and scalar part of the quaternion
    Vec3 qvec = {q.x, q.y, q.z};
    float qw = q.w;
    
    // Prepare cross products
    Vec3 cross1 = crossProduct(qvec, v);
    Vec3 cross2 = crossProduct(qvec, cross1);
    
    // For Rodrigues' rotation formula:
    // v' = v + 2 * (qw * cross1 + cross2)
    return Vec3{
        v.x + 2.0f * (qw * cross1.x + cross2.x),
        v.y + 2.0f * (qw * cross1.y + cross2.y),
        v.z + 2.0f * (qw * cross1.z + cross2.z)
    };
}

Vec3 scaleVector(const Vec3& v, const Vec3& scale) {
    return Vec3{v.x * scale.x, v.y * scale.y, v.z * scale.z};
}

inline Transform compose(const Transform& a, const Transform& b) {
    Transform result;
    
    // Compose rotations by multiplying quaternions (apply 'a' first, then 'b')
    result.r = multiplyQuaternions(a.r, b.r);
    
    // Compose scales by component-wise multiplication
    result.s.x = a.s.x * b.s.x;
    result.s.y = a.s.y * b.s.y;
    result.s.z = a.s.z * b.s.z;
    
    // Compose translations: transform b's translation by a's rotation and scale, then add a's translation
    Vec3 rotatedAndScaledTransB = rotateVector(a.r, scaleVector(b.t, a.s));
    result.t.x = a.t.x + rotatedAndScaledTransB.x;
    result.t.y = a.t.y + rotatedAndScaledTransB.y;
    result.t.z = a.t.z + rotatedAndScaledTransB.z;
    
    return result;
}

// for undoing a transformation
inline Transform inverse(const Transform& t) {
    Transform result;
    
    // Define minimum scale to prevent division by zero and numerical instability
    const float MIN_SCALE = 0.001f;
    
    // Clamp each scale component and compute its inverse
    // We use max() to ensure scale never goes below our minimum
    float safeScaleX = std::max(std::abs(t.s.x), MIN_SCALE);
    float safeScaleY = std::max(std::abs(t.s.y), MIN_SCALE);
    float safeScaleZ = std::max(std::abs(t.s.z), MIN_SCALE);
    
    result.s.x = 1.0f / safeScaleX;
    result.s.y = 1.0f / safeScaleY;
    result.s.z = 1.0f / safeScaleZ;
    
    // Handle the sign separately if needed - negative scales flip geometry
    if (t.s.x < 0.0f) result.s.x = -result.s.x;
    if (t.s.y < 0.0f) result.s.y = -result.s.y;
    if (t.s.z < 0.0f) result.s.z = -result.s.z;
    
    // Inverse of quaternion rotation is its conjugate (for unit quaternions)
    result.r.x = -t.r.x;
    result.r.y = -t.r.y;
    result.r.z = -t.r.z;
    result.r.w = t.r.w;  // Scalar part stays the same
    
    // Inverse translation is more complex
    // We need to undo the translation, but in the coordinate system
    // that existed before the rotation and scaling
    Vec3 negatedTranslation = {-t.t.x, -t.t.y, -t.t.z};
    Vec3 rotatedNegTranslation = rotateVector(result.r, negatedTranslation);
    result.t = scaleVector(rotatedNegTranslation, result.s);
    
    return result;
}

void localToModel(const Skeleton& skel,
                    std::span<const Transform> local,
                    std::span<Transform> model)
{
    for (size_t i=0; i<skel.parent.size(); ++i) {
        int p = skel.parent[i];
        model[i] = (p>=0) ? compose(model[p], local[i]) : local[i];
    }
}

// Testing and utility functions
Transform identity() {
    return Transform{
        Quat(0.0f, 0.0f, 0.0f, 1.0f),  // Identity quaternion (no rotation)
        Vec3(0.0f, 0.0f, 0.0f),        // No translation
        Vec3(1.0f, 1.0f, 1.0f)         // No scaling
    };
}

bool isApproximatelyEqual(float a, float b, float epsilon = 1e-5f) {
    return std::abs(a - b) < epsilon;
}

bool isApproximatelyEqual(const Vec3& a, const Vec3& b, float epsilon = 1e-5f) {
    return isApproximatelyEqual(a.x, b.x, epsilon) &&
           isApproximatelyEqual(a.y, b.y, epsilon) &&
           isApproximatelyEqual(a.z, b.z, epsilon);
}

bool isApproximatelyEqual(const Quat& a, const Quat& b, float epsilon = 1e-5f) {
    // Quaternions q and -q represent the same rotation, so we check both possibilities
    bool sameSign = isApproximatelyEqual(a.x, b.x, epsilon) &&
                    isApproximatelyEqual(a.y, b.y, epsilon) &&
                    isApproximatelyEqual(a.z, b.z, epsilon) &&
                    isApproximatelyEqual(a.w, b.w, epsilon);
    
    bool oppositeSign = isApproximatelyEqual(a.x, -b.x, epsilon) &&
                        isApproximatelyEqual(a.y, -b.y, epsilon) &&
                        isApproximatelyEqual(a.z, -b.z, epsilon) &&
                        isApproximatelyEqual(a.w, -b.w, epsilon);
    
    return sameSign || oppositeSign;
}

bool isApproximatelyEqual(const Transform& a, const Transform& b, float epsilon = 1e-5f) {
    return isApproximatelyEqual(a.r, b.r, epsilon) &&
           isApproximatelyEqual(a.t, b.t, epsilon) &&
           isApproximatelyEqual(a.s, b.s, epsilon);
}

// Pretty printing functions for debugging
void printVec3(const Vec3& v, const std::string& name) {
    std::cout << name << ": (" << v.x << ", " << v.y << ", " << v.z << ")" << std::endl;
}

void printQuat(const Quat& q, const std::string& name) {
    std::cout << name << ": (" << q.x << ", " << q.y << ", " << q.z << ", " << q.w << ")" << std::endl;
}

void printTransform(const Transform& t, const std::string& name) {
    std::cout << "=== " << name << " ===" << std::endl;
    printQuat(t.r, "Rotation");
    printVec3(t.t, "Translation");
    printVec3(t.s, "Scale");
    std::cout << std::endl;
}

// Test functions to verify our mathematical implementations
void testIdentityProperties() {
    std::cout << "=== Testing Identity Properties ===" << std::endl;
    
    Transform id = identity();
    Transform testTransform(
        Quat(0.1f, 0.2f, 0.3f, 0.9f),  // Some rotation
        Vec3(5.0f, -2.0f, 10.0f),      // Some translation
        Vec3(2.0f, 0.5f, 1.5f)         // Some scale
    );
    
    // Test: compose(identity, transform) should equal transform
    Transform result1 = compose(id, testTransform);
    bool test1 = isApproximatelyEqual(result1, testTransform);
    std::cout << "Identity * Transform = Transform: " << (test1 ? "PASS" : "FAIL") << std::endl;
    
    // Test: compose(transform, identity) should equal transform  
    Transform result2 = compose(testTransform, id);
    bool test2 = isApproximatelyEqual(result2, testTransform);
    std::cout << "Transform * Identity = Transform: " << (test2 ? "PASS" : "FAIL") << std::endl;
    
    std::cout << std::endl;
}

void testInverseProperties() {
    std::cout << "=== Testing Inverse Properties ===" << std::endl;
    
    Transform testTransform(
        Quat(0.5f, 2.0f, 3.0f, 1.0f),  // Some rotation
        Vec3(5.0f, -2.0f, 10.0f),      // Some translation
        Vec3(2.0f, 0.5f, 1.5f)         // Some scale
    );

    testTransform.r.normalize(); // Ensure it's a unit quaternion

    //printTransform(testTransform, "Original Transform");

    Transform inv = inverse(testTransform);
    //printTransform(inv, "Computed Inverse");

    Transform shouldBeIdentity1 = compose(testTransform, inv);
    Transform actualIdentity = identity();

    std::cout << "Results:" << std::endl;
    printTransform(shouldBeIdentity1, "Transform * Inverse");
    printTransform(actualIdentity, "Expected Identity");

    bool test1 = isApproximatelyEqual(shouldBeIdentity1, actualIdentity, 1e-4f);

    std::cout << "Transform * Inverse = Identity: " << (test1 ? "PASS" : "FAIL") << std::endl;

    /*
    std::cout << "\nComponent-by-component analysis:" << std::endl;
    std::cout << "Rotation check: " << (isApproximatelyEqual(shouldBeIdentity1.r, actualIdentity.r) ? "PASS" : "FAIL") << std::endl;
    std::cout << "Translation check: " << (isApproximatelyEqual(shouldBeIdentity1.t, actualIdentity.t) ? "PASS" : "FAIL") << std::endl;
    std::cout << "Scale check: " << (isApproximatelyEqual(shouldBeIdentity1.s, actualIdentity.s) ? "PASS" : "FAIL") << std::endl;
    */

    std::cout << std::endl;
}

void testScaleClamping() {
    std::cout << "=== Testing Scale Clamping ===" << std::endl;
    
    // Test with very small scale that should trigger clamping
    Transform tinyScale(
        Quat(0.0f, 0.0f, 0.0f, 1.0f),  // No rotation
        Vec3(0.0f, 0.0f, 0.0f),        // No translation
        Vec3(0.0001f, 0.0001f, 0.0001f) // Very small scale
    );
    
    Transform inv = inverse(tinyScale);
    std::cout << "Original tiny scale: " << tinyScale.s.x << std::endl;
    std::cout << "Inverse scale (should be clamped): " << inv.s.x << std::endl;
    std::cout << "Clamp triggered: " << (inv.s.x <= 1000.0f ? "YES" : "NO") << std::endl;
    std::cout << std::endl;
}

int main() {
    std::cout << "==============================" << std::endl;
    std::cout << "3D Transform Math Test Program" << std::endl;
    std::cout << "==============================" << std::endl << std::endl;
    
    // Run our test suite
    testIdentityProperties();
    testInverseProperties();
    testScaleClamping();
    
    std::cout << "All tests completed!" << std::endl;
    std::cout << "If there are any FAIL results above, there may be bugs in the implementation." << std::endl;
    
    return 0;
}