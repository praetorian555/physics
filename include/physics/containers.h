#pragma once

#include <array>
#include <span>
#include <vector>

#include "physics/base.h"

namespace Physics
{

template <typename T>
using Array = std::vector<T>;

template <typename T, size_t N>
using StackArray = std::array<T, N>;

template <typename T>
using Span = std::span<T>;

}  // namespace Physics