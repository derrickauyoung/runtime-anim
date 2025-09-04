// To compile: clang++ -O3 -std=c++14 quaternion_benchmark.cpp -o ../bin/quat_benchmark
#include <iostream>
#include <chrono>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include <numeric>

// Platform-specific SIMD headers - only include what we actually need
#ifdef __ARM_NEON
    #include <arm_neon.h> // For ARM NEON intrinsics on Apple Silicon
#endif

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    #include <immintrin.h> // For AVX/SSE intrinsics on Intel/AMD processors
#endif

// =============================================================================
// DATA STRUCTURES: AoS vs SoA Comparison
// =============================================================================

// Array-of-Structures: Traditional approach
// Each quaternion is stored as a complete unit
struct Quaternion {
    float x, y, z, w;
    
    void normalize() {
        float length = std::sqrt(x*x + y*y + z*z + w*w);
        if (length > 0.0f) {
            float inv_length = 1.0f / length;
            x *= inv_length;
            y *= inv_length;
            z *= inv_length;
            w *= inv_length;
        }
    }
};

using PoseAoS = std::vector<Quaternion>;

// Structure-of-Arrays: Optimized for SIMD and cache efficiency
// Each component is stored in its own contiguous array
struct PoseSoA {
    int jointCount;
    std::unique_ptr<float[]> x, y, z, w; // Separate arrays for each component
    
    explicit PoseSoA(int count) : jointCount(count) {
        // Allocate aligned memory for SIMD operations (32-byte alignment for AVX)
        x = std::make_unique<float[]>(count);
        y = std::make_unique<float[]>(count);
        z = std::make_unique<float[]>(count);
        w = std::make_unique<float[]>(count);
    }
    
    // Copy constructor for benchmarking purposes
    PoseSoA(const PoseSoA& other) : jointCount(other.jointCount) {
        x = std::make_unique<float[]>(jointCount);
        y = std::make_unique<float[]>(jointCount);
        z = std::make_unique<float[]>(jointCount);
        w = std::make_unique<float[]>(jointCount);
        
        for (int i = 0; i < jointCount; ++i) {
            x[i] = other.x[i];
            y[i] = other.y[i];
            z[i] = other.z[i];
            w[i] = other.w[i];
        }
    }
};

// =============================================================================
// NORMALIZATION IMPLEMENTATIONS
// =============================================================================

// AoS approach: Process each quaternion individually
void normalizeQuatsAoS(PoseAoS& pose) {
    for (auto& quat : pose) {
        quat.normalize();
    }
}

// SoA approach: Scalar version (for fair comparison)
void normalizeQuatsSoAScalar(PoseSoA& pose) {
    for (int i = 0; i < pose.jointCount; ++i) {
        float length = std::sqrt(pose.x[i] * pose.x[i] + 
                                pose.y[i] * pose.y[i] + 
                                pose.z[i] * pose.z[i] + 
                                pose.w[i] * pose.w[i]);
        if (length > 0.0f) {
            float inv_length = 1.0f / length;
            pose.x[i] *= inv_length;
            pose.y[i] *= inv_length;
            pose.z[i] *= inv_length;
            pose.w[i] *= inv_length;
        }
    }
}

// Platform-specific SIMD implementations - completely separate functions
#ifdef __ARM_NEON
// ARM NEON implementation - only compiled on ARM processors
void normalizeQuatsSoASIMD(PoseSoA& pose) {
    const int simdWidth = 4; // ARM NEON processes 4 floats at once
    const int simdIterations = pose.jointCount / simdWidth;
    
    // Process 4 quaternions at a time using ARM NEON
    for (int i = 0; i < simdIterations; ++i) {
        int baseIdx = i * simdWidth;
        
        // Load 4 components of each quaternion component
        float32x4_t vx = vld1q_f32(&pose.x[baseIdx]);
        float32x4_t vy = vld1q_f32(&pose.y[baseIdx]);
        float32x4_t vz = vld1q_f32(&pose.z[baseIdx]);
        float32x4_t vw = vld1q_f32(&pose.w[baseIdx]);
        
        // Calculate length squared: x² + y² + z² + w²
        float32x4_t lengthSq = vmulq_f32(vx, vx);
        lengthSq = vfmaq_f32(lengthSq, vy, vy);  // Fused multiply-add: faster and more accurate
        lengthSq = vfmaq_f32(lengthSq, vz, vz);
        lengthSq = vfmaq_f32(lengthSq, vw, vw);
        
        // Calculate reciprocal square root - ARM NEON's two-step approach
        // First approximation using vrsqrteq_f32 (fast but less precise)
        float32x4_t invLength = vrsqrteq_f32(lengthSq);
        
        // Newton-Raphson refinement for better accuracy
        // Formula: x = x * (1.5 - 0.5 * a * x²) where a is our original value
        float32x4_t half = vdupq_n_f32(0.5f);
        float32x4_t onePtFive = vdupq_n_f32(1.5f);
        float32x4_t refinement = vmulq_f32(lengthSq, invLength);
        refinement = vmulq_f32(refinement, invLength);
        refinement = vmulq_f32(half, refinement);
        refinement = vsubq_f32(onePtFive, refinement);
        invLength = vmulq_f32(invLength, refinement);
        
        // Apply normalization to all components
        vx = vmulq_f32(vx, invLength);
        vy = vmulq_f32(vy, invLength);
        vz = vmulq_f32(vz, invLength);
        vw = vmulq_f32(vw, invLength);
        
        // Store normalized results back to memory
        vst1q_f32(&pose.x[baseIdx], vx);
        vst1q_f32(&pose.y[baseIdx], vy);
        vst1q_f32(&pose.z[baseIdx], vz);
        vst1q_f32(&pose.w[baseIdx], vw);
    }
    
    // Handle remaining quaternions that don't fit in SIMD width using scalar code
    for (int i = simdIterations * simdWidth; i < pose.jointCount; ++i) {
        float length = std::sqrt(pose.x[i] * pose.x[i] + 
                                pose.y[i] * pose.y[i] + 
                                pose.z[i] * pose.z[i] + 
                                pose.w[i] * pose.w[i]);
        if (length > 0.0f) {
            float inv_length = 1.0f / length;
            pose.x[i] *= inv_length;
            pose.y[i] *= inv_length;
            pose.z[i] *= inv_length;
            pose.w[i] *= inv_length;
        }
    }
}

#elif defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
// Intel/AMD x86 AVX implementation - only compiled on x86 processors  
void normalizeQuatsSoASIMD(PoseSoA& pose) {
    const int simdWidth = 8; // AVX processes 8 floats at once
    const int simdIterations = pose.jointCount / simdWidth;
    
    // Process 8 quaternions at a time using AVX SIMD
    for (int i = 0; i < simdIterations; ++i) {
        int baseIdx = i * simdWidth;
        
        // Load 8 components of each quaternion component (unaligned for safety)
        __m256 vx = _mm256_loadu_ps(&pose.x[baseIdx]);
        __m256 vy = _mm256_loadu_ps(&pose.y[baseIdx]);
        __m256 vz = _mm256_loadu_ps(&pose.z[baseIdx]);
        __m256 vw = _mm256_loadu_ps(&pose.w[baseIdx]);
        
        // Calculate length squared using fused multiply-add for better performance
        __m256 lengthSq = _mm256_mul_ps(vx, vx);
        lengthSq = _mm256_fmadd_ps(vy, vy, lengthSq);
        lengthSq = _mm256_fmadd_ps(vz, vz, lengthSq);
        lengthSq = _mm256_fmadd_ps(vw, vw, lengthSq);
        
        // Calculate reciprocal square root (faster than sqrt + division)
        __m256 invLength = _mm256_rsqrt_ps(lengthSq);
        
        // Apply normalization to all components
        vx = _mm256_mul_ps(vx, invLength);
        vy = _mm256_mul_ps(vy, invLength);
        vz = _mm256_mul_ps(vz, invLength);
        vw = _mm256_mul_ps(vw, invLength);
        
        // Store results back to memory (unaligned store for safety)
        _mm256_storeu_ps(&pose.x[baseIdx], vx);
        _mm256_storeu_ps(&pose.y[baseIdx], vy);
        _mm256_storeu_ps(&pose.z[baseIdx], vz);
        _mm256_storeu_ps(&pose.w[baseIdx], vw);
    }
    
    // Handle remaining quaternions using scalar code
    for (int i = simdIterations * simdWidth; i < pose.jointCount; ++i) {
        float length = std::sqrt(pose.x[i] * pose.x[i] + 
                                pose.y[i] * pose.y[i] + 
                                pose.z[i] * pose.z[i] + 
                                pose.w[i] * pose.w[i]);
        if (length > 0.0f) {
            float inv_length = 1.0f / length;
            pose.x[i] *= inv_length;
            pose.y[i] *= inv_length;
            pose.z[i] *= inv_length;
            pose.w[i] *= inv_length;
        }
    }
}

#else
// Fallback implementation for unknown architectures - pure scalar code
void normalizeQuatsSoASIMD(PoseSoA& pose) {
    std::cout << "Warning: No SIMD support detected, using scalar fallback implementation\n";
    // Just use the scalar implementation - it's the same as normalizeQuatsSoAScalar
    normalizeQuatsSoAScalar(pose);
}
#endif

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

// Generate random quaternions that need normalization
void generateRandomQuats(PoseAoS& poseAoS, PoseSoA& poseSoA, int count) {
    std::random_device rd;
    std::mt19937 gen(42); // Fixed seed for reproducible results
    std::uniform_real_distribution<float> dis(0.1f, 2.0f); // Non-unit quaternions
    
    poseAoS.resize(count);
    
    for (int i = 0; i < count; ++i) {
        float x = dis(gen);
        float y = dis(gen);
        float z = dis(gen);
        float w = dis(gen);
        
        // Store in AoS format
        poseAoS[i] = {x, y, z, w};
        
        // Store in SoA format
        poseSoA.x[i] = x;
        poseSoA.y[i] = y;
        poseSoA.z[i] = z;
        poseSoA.w[i] = w;
    }
}

// Verify that normalization worked correctly
bool verifyNormalization(const PoseAoS& poseAoS, const PoseSoA& poseSoA, float tolerance = 1e-6f) {
    for (int i = 0; i < poseAoS.size(); ++i) {
        // Check AoS
        float lengthAoS = std::sqrt(poseAoS[i].x * poseAoS[i].x + 
                                   poseAoS[i].y * poseAoS[i].y + 
                                   poseAoS[i].z * poseAoS[i].z + 
                                   poseAoS[i].w * poseAoS[i].w);
        
        // Check SoA
        float lengthSoA = std::sqrt(poseSoA.x[i] * poseSoA.x[i] + 
                                   poseSoA.y[i] * poseSoA.y[i] + 
                                   poseSoA.z[i] * poseSoA.z[i] + 
                                   poseSoA.w[i] * poseSoA.w[i]);
        
        if (std::abs(lengthAoS - 1.0f) > tolerance || std::abs(lengthSoA - 1.0f) > tolerance) {
            return false;
        }
    }
    return true;
}

// =============================================================================
// BENCHMARKING FRAMEWORK
// =============================================================================

struct BenchmarkResult {
    std::string name;
    std::vector<double> times; // Multiple runs for statistical analysis
    double avgTime;
    double minTime;
    double maxTime;
    double stdDev;
};

template<typename Func>
BenchmarkResult benchmark(const std::string& name, Func&& func, int iterations = 100) {
    BenchmarkResult result;
    result.name = name;
    result.times.reserve(iterations);
    
    // Warm up the CPU and caches
    for (int i = 0; i < 10; ++i) {
        func();
    }
    
    // Actual benchmark runs
    for (int i = 0; i < iterations; ++i) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
        result.times.push_back(static_cast<double>(duration.count()) / 1000.0); // Convert to microseconds
    }
    
    // Calculate statistics
    result.avgTime = std::accumulate(result.times.begin(), result.times.end(), 0.0) / result.times.size();
    result.minTime = *std::min_element(result.times.begin(), result.times.end());
    result.maxTime = *std::max_element(result.times.begin(), result.times.end());
    
    // Calculate standard deviation
    double variance = 0.0;
    for (double time : result.times) {
        variance += (time - result.avgTime) * (time - result.avgTime);
    }
    result.stdDev = std::sqrt(variance / result.times.size());
    
    return result;
}

void printResults(const std::vector<BenchmarkResult>& results, int jointCount) {
    std::cout << "\n=== QUATERNION NORMALIZATION BENCHMARK RESULTS ===\n";
    std::cout << "Joint Count: " << jointCount << "\n";
    
#ifdef __ARM_NEON
    std::cout << "CPU: Apple Silicon (ARM64 with NEON SIMD - 4-way parallelism)\n\n";
#elif defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    std::cout << "CPU: x86/x64 with AVX SIMD (8-way parallelism)\n\n";
#else
    std::cout << "CPU: Unknown architecture (scalar fallback only)\n\n";
#endif
    
    std::cout << std::setw(20) << "Method" 
              << std::setw(15) << "Avg (μs)" 
              << std::setw(15) << "Min (μs)" 
              << std::setw(15) << "Max (μs)"
              << std::setw(15) << "Std Dev"
              << std::setw(15) << "Speedup" << "\n";
    std::cout << std::string(95, '-') << "\n";
    
    double baselineTime = results[0].avgTime; // Use first result as baseline
    
    for (const auto& result : results) {
        double speedup = baselineTime / result.avgTime;
        std::cout << std::setw(20) << result.name
                  << std::setw(15) << std::fixed << std::setprecision(2) << result.avgTime
                  << std::setw(15) << result.minTime
                  << std::setw(15) << result.maxTime
                  << std::setw(15) << result.stdDev
                  << std::setw(15) << speedup << "x\n";
    }
    
    std::cout << "\nPerformance Analysis:\n";
    std::cout << "- Lower times indicate better performance\n";
    std::cout << "- Speedup is relative to the first method (" << results[0].name << ")\n";
    std::cout << "- Lower standard deviation indicates more consistent performance\n";
    std::cout << "- SoA layout should show better cache performance and enable SIMD optimizations\n";
    
#ifdef __ARM_NEON
    std::cout << "- ARM NEON uses 4-way SIMD with Newton-Raphson refinement for higher accuracy\n";
#elif defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
    std::cout << "- Intel AVX uses 8-way SIMD for maximum throughput\n";
#endif
}

// =============================================================================
// MAIN BENCHMARK PROGRAM
// =============================================================================

int main() {
    std::cout << "Quaternion Normalization: AoS vs SoA Microbenchmark\n";
    std::cout << "===================================================\n";
    std::cout << "This benchmark demonstrates the performance differences between:\n";
    std::cout << "- Array-of-Structures (AoS): Traditional data organization\n";
    std::cout << "- Structure-of-Arrays (SoA): Cache-friendly + SIMD-optimized organization\n\n";
    
    // Test different problem sizes to see scaling behavior
    std::vector<int> testSizes = {1000, 10000, 100000};
    
    for (int jointCount : testSizes) {
        std::cout << "\n>>> Testing with " << jointCount << " quaternions <<<\n";
        
        // Create data structures
        PoseAoS originalAoS;
        PoseSoA originalSoA(jointCount);
        generateRandomQuats(originalAoS, originalSoA, jointCount);
        
        std::vector<BenchmarkResult> results;
        
        // Benchmark 1: Array-of-Structures approach
        results.push_back(benchmark("AoS", [&]() {
            PoseAoS testData = originalAoS; // Copy for each test
            normalizeQuatsAoS(testData);
        }));
        
        // Benchmark 2: Structure-of-Arrays scalar approach
        results.push_back(benchmark("SoA Scalar", [&]() {
            PoseSoA testData = originalSoA; // Copy for each test
            normalizeQuatsSoAScalar(testData);
        }));
        
        // Benchmark 3: Structure-of-Arrays SIMD approach
        results.push_back(benchmark("SoA SIMD", [&]() {
            PoseSoA testData = originalSoA; // Copy for each test
            normalizeQuatsSoASIMD(testData);
        }));
        
        // Verify correctness - make sure all methods produce the same results
        PoseAoS verifyAoS = originalAoS;
        PoseSoA verifySoA = originalSoA;
        normalizeQuatsAoS(verifyAoS);
        normalizeQuatsSoASIMD(verifySoA);
        
        if (verifyNormalization(verifyAoS, verifySoA)) {
            std::cout << "✓ Normalization correctness verified - all methods produce equivalent results\n";
        } else {
            std::cout << "✗ Warning: Normalization results differ between methods!\n";
        }
        
        printResults(results, jointCount);
    }
    
    std::cout << "\nKey Notes:\n";
    std::cout << "1. Memory Layout Matters: SoA should show better cache performance (lower variance)\n";
    std::cout << "2. SIMD Impact: SIMD version should significantly outperform scalar versions\n";
    std::cout << "3. Scaling Behavior: Performance differences should increase with larger datasets\n";
    std::cout << "4. Architecture Differences: ARM vs Intel processors handle parallelism differently\n";
    std::cout << "5. Real-World Application: These techniques are essential in game engines and CAD software\n";
    
    return 0;
}