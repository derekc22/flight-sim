#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>

namespace util {

    template <typename T, std::size_t N>
    void fill_arr(std::array<T, N>& x, std::size_t start, std::size_t stop, const T& val) {
        if (start > stop || start > N || stop > N) throw std::out_of_range("util::fill_arr indices are invalid");
        std::fill(x.begin() + start, x.begin() + stop, val);
    }

}
