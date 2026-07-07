// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The single translation unit that compiles stb_image's implementation. Kept
// apart from image_io.cpp (which uses only the declarations) so this
// third-party code can be built with warnings disabled -- it does not survive
// the tier's -Werror surface. CMake attaches `-w` to this file alone.

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_ONLY_PNG
#include "stb_image.h"
