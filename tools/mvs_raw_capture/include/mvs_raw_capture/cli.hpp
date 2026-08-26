#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace mvs_raw_capture
{
namespace fs = std::filesystem;

struct CaptureOptions
{
  bool help = false;
  double duration_seconds = 0.0;
  fs::path output_dir;
  std::string session_name;
};

struct ConvertOptions
{
  bool help = false;
  fs::path session_dir;
  fs::path output_dir;
  bool allow_incomplete = false;
  std::vector<std::uint64_t> indices;
};

CaptureOptions parse_capture_options(const std::vector<std::string> & arguments);
ConvertOptions parse_convert_options(const std::vector<std::string> & arguments);
std::string capture_usage();
std::string convert_usage();

}  // namespace mvs_raw_capture
