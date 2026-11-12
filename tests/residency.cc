// Standalone Linux regression test: g++ -std=c++23 -I. tests/residency.cc -o ...
#include "cpp/residency.h"

#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <unistd.h>

int main() {
  auto const page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  auto const length = page * 3U;
  auto original = std::vector<unsigned char>(length, 42U);
  char path[] = "/tmp/osr-residency-XXXXXX";
  auto const fd = ::mkstemp(path);
  assert(fd != -1);
  assert(::unlink(path) == 0);
  assert(::write(fd, original.data(), length) == static_cast<ssize_t>(length));
  auto* mapping = static_cast<unsigned char*>(
      ::mmap(nullptr, length, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0));
  assert(mapping != MAP_FAILED);
  assert(std::memcmp(mapping, original.data(), length) == 0);

  // Model cista relocation: a pointer written into a private file page must
  // survive advice. Its target and all clean pages must stay readable too.
  auto* target = mapping + page;
  std::memcpy(mapping, &target, sizeof(target));
  for (auto pageout : {false, true}) {
    assert(catenary_osr::detail::advise_mapping(mapping, length, pageout) == 0);
    unsigned char* relocated = nullptr;
    std::memcpy(&relocated, mapping, sizeof(relocated));
    assert(relocated == target);
    assert(*relocated == 42U);
    assert(std::memcmp(mapping + sizeof(target),
                       original.data() + sizeof(target),
                       length - sizeof(target)) == 0);
    auto disk = std::vector<unsigned char>(length);
    assert(::pread(fd, disk.data(), length, 0) == static_cast<ssize_t>(length));
    assert(disk == original);
  }
  assert(catenary_osr::detail::advise_mapping(mapping + 1, page, false) == EINVAL);
  assert(catenary_osr::detail::advise_mapping(nullptr, 0, true) == 0);
  assert(::munmap(mapping, length) == 0);
  assert(::close(fd) == 0);
}
#else
int main() {
  assert(catenary_osr::detail::advise_mapping(nullptr, 1, false) == ENOTSUP);
}
#endif
