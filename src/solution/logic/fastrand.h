#pragma once

extern unsigned int RANDOM_SEED;

inline int fastrand() {
    //fastrand routine returns one integer, similar output value range as C lib.
    RANDOM_SEED = 214013 * RANDOM_SEED + 2531011;
    return (RANDOM_SEED >> 16) & 0x7FFF;
}

inline int rand_int(int max_size) {
    return fastrand() % max_size;
}

inline int fast_rand_int(int a, int b) {
    return (a + rand_int(b - a));
}

inline double fast_rand_double() {
    return static_cast<double>(fastrand()) / 0x7FFF;
}

inline double fast_rand_double(double a, double b) {
    return a + (static_cast<double>(fastrand()) / 0x7FFF) * (b - a);
}
