#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "mvs_raw_capture/cli.hpp"

namespace mvs_raw_capture
{

struct ConversionResult
{
  std::uint64_t exported_frames = 0;
  std::vector<std::uint64_t> acquired_indices;
  std::filesystem::path metadata_path;
  std::filesystem::path status_path;
};

ConversionResult convert_session(const ConvertOptions & options);

}  // namespace mvs_raw_capture
