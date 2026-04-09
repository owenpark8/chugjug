#pragma once
#include <cstddef>

template<typename T, size_t N>
class RunningMeanFilter {
    static_assert((N & (N - 1)) == 0, "N must be a power of two");
    T buffer[N] = {};
    T sum = {};
    size_t index = 0;

public:
    void add(T value) {
        sum -= buffer[index];
        buffer[index] = value;
        sum += value;
        index = (index + 1) & (N - 1);
    }

    T mean() const {
        return sum / static_cast<T>(N);
    }
};
