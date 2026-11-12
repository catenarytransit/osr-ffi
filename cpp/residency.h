#pragma once

#include <cerrno>
#include <cstddef>

#if defined(__linux__)
#include <sys/mman.h>
#endif

namespace catenary_osr::detail {
// Only call on an owned, page-aligned file mapping. These operations preserve
// private COW modifications (including cista's relocated pointers).
inline int advise_mapping(void const* address, std::size_t size, bool pageout) {
  if (size == 0U) {
    return 0;
  }
#if defined(__linux__) && defined(MADV_COLD) && defined(MADV_PAGEOUT)
  if (::madvise(const_cast<void*>(address), size,
                pageout ? MADV_PAGEOUT : MADV_COLD) != 0) {
    return errno;
  }
  return 0;
#else
  (void)address;
  (void)pageout;
  return ENOTSUP;
#endif
}
}  // namespace catenary_osr::detail
