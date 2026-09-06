#pragma once
#include <math.h>

#define DEFINE_FIXED_TYPE(NAME, FRAC_BITS)                                     \
    typedef signed int NAME;                                                   \
    enum { NAME##_frac_bits = (FRAC_BITS) };                                   \
    static const float _##NAME##_float_div = (1 << (NAME##_frac_bits));        \
    static inline NAME int_to_##NAME(signed int x) {                           \
        return (NAME)((signed long long)x *                                    \
                      (signed long long)(1ULL << NAME##_frac_bits));           \
    }                                                                          \
    static inline signed int NAME##_to_int(NAME x) {                           \
        return (signed int)(x / (signed long long)(1ULL << NAME##_frac_bits)); \
    }                                                                          \
    static inline NAME mult##NAME(NAME a, NAME b) {                            \
        return (NAME)(((signed long long)a * (signed long long)b) /            \
                      (signed long long)(1ULL << NAME##_frac_bits));           \
    }                                                                          \
                                                                               \
    static inline NAME div##NAME(NAME a, NAME b) {                             \
        return (NAME)(((signed long long)a *                                   \
                       (signed long long)(1ULL << NAME##_frac_bits)) /         \
                      (signed long long)b);                                    \
    }                                                                          \
    static inline float NAME##_to_float(NAME x) {                              \
        return (float)x / _##NAME##_float_div;                                 \
    }                                                                          \
    static inline NAME float_to_##NAME(float x) {                              \
        return (NAME)(x * _##NAME##_float_div);                                \
    }                                                                          \
    static inline NAME sqrt##NAME(NAME x) {                                    \
        return float_to_##NAME(sqrtf(NAME##_to_float(x)));                     \
    }                                                                          \
    static inline NAME NAME##_half(NAME x) { return x >> 1; }

DEFINE_FIXED_TYPE(fix8, 8)
