#pragma once
#include <filesystem>
#include <utility>
#include "cista/io.h"
namespace osr::ffi_metadata_loading {
template <typename T> cista::wrapped<T> read_private(std::filesystem::path const& path) {
  auto filename = path.generic_string();
  auto mapped = cista::mmap{filename.c_str(), cista::mmap::protection::COPY_ON_WRITE};
  auto* value = cista::deserialize<T, cista::kDefaultMode>(mapped);
  auto holder = cista::memory_holder{cista::buf{std::move(mapped)}};
  return cista::wrapped<T>{std::move(holder), value};
}
template <typename T> T read_value(std::filesystem::path const& path) {
  auto holder = read_private<T>(path);
  return *holder;
}
}
