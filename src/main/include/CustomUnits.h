#pragma once

#include <units/angular_velocity.h>
#include <units/voltage.h>

using rad_per_s_t = decltype(1_rad_per_s);

using Ks_t = decltype(1_V);
template <typename T>
using Kv_t = decltype(1_V / (T(1) / 1_s));
template <typename T>
using Ka_t = decltype(Kv_t<T>(1) * 1_s);