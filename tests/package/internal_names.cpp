// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// recon_package_headers compiles this against the installed recon and never
// runs it. The hash table's internals stay out of the install: volume/hash.hpp
// is not installed; every installed volume header, and every installed recon
// header that includes one, compiles without it; and none of them declares one
// of the names below in the volume namespace.

#if __has_include("volumetric_kit/recon/volume/hash.hpp")
#error "volume/hash.hpp is installed, but the hash table's layout is internal"
#endif

// Those headers, as package/CMakeLists.txt finds them in the install.
#include "vr_volume_headers.hpp"

// An alias is ill-formed after any other declaration of its name: a class or
// its forward declaration, a variable, a function or another alias.
namespace volumetric_kit::recon::volume {
struct InternalNameProbe {};
using HashEntry = InternalNameProbe;
using hash_bucket = InternalNameProbe;
using kFreeEntry = InternalNameProbe;
using kLockEntry = InternalNameProbe;
using kNoOffset = InternalNameProbe;
using kHashPrimeX = InternalNameProbe;
using kHashPrimeY = InternalNameProbe;
using kHashPrimeZ = InternalNameProbe;
using kFailTotal = InternalNameProbe;
using kFailLock = InternalNameProbe;
using kFailChain = InternalNameProbe;
using kFailHeap = InternalNameProbe;
using kFailTerminal = InternalNameProbe;
using kFailTable = InternalNameProbe;
using kFailSlots = InternalNameProbe;
}  // namespace volumetric_kit::recon::volume
