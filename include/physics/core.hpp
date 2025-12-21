#pragma once

#include "opal/defines.h"
#include "opal/types.h"

namespace Physics
{

using i8 = Opal::i8;
using i16 = Opal::i16;
using i32 = Opal::i32;
using i64 = Opal::i64;

using u8 = Opal::u8;
using u16 = Opal::u16;
using u32 = Opal::u32;
using u64 = Opal::u64;

using f32 = Opal::f32;
using f64 = Opal::f64;

using char8 = Opal::char8;
using char16 = Opal::char16;
using uchar32 = Opal::uchar32;

#ifdef PHYSICS_REAL_AS_DOUBLE
using real = Opal::f64;
#else
using real = Opal::f32;
#endif

}