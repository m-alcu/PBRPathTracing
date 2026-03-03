#pragma once
#include <cstdint>

// PCG32 random number generator (stateful, high quality)
struct RNG {
    uint64_t state;
    explicit RNG(uint64_t seed = 0xcafef00dd15ea5e5ull) : state(seed) {}

    uint32_t next() {
        uint64_t old = state;
        state = old * 6364136223846793005ull + 1442695040888963407ull;
        uint32_t xorshifted = (uint32_t)(((old >> 18u) ^ old) >> 27u);
        uint32_t rot        = (uint32_t)(old >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((-(int)rot) & 31));
    }

    // Returns a float in [0, 1)
    float nextFloat() { return (float)(next() >> 8) * (1.0f / (1 << 24)); }
};

// Hash-based RNG — stateless scramble of a counter (fast, no 64-bit state)
struct HashRNG {
    uint32_t index;
    explicit HashRNG(uint32_t seed = 0) : index(seed) {}

    uint32_t next() {
        uint32_t i = index++;
        i = (i << 13) ^ i;
        return (i * (i * i * 15731u + 789221u) + 1376312589u) & 0x7fffffffu;
    }

    // Returns a float in [0, 1)
    float nextFloat() { return (float)next() * (1.0f / (float)0x80000000u); }
};
