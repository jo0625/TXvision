#include "mvs_raw_capture/cli.hpp"

#include <charconv>
#include <cmath>
#include <set>
#include <stdexcept>

#include "mvs_raw_capture/capture_result.hpp"
#include "mvs_raw_capture/session.hpp"

namespace mvs_raw_capture
{
namespace
{
double parse_duration(const std::string & text)
{
  std::size_t consumed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &consumed);
  } catch (const std::exception &) {
    throw std::invalid_argument("--duration must be in (0, 86400] seconds");
  }
  if (consumed != text.size() || !std::isfinite(value) || value <= 0.0 || value > 86400.0) {
    throw std::invalid_argument("--duration must be in (0, 86400] seconds");
  }
  return value;
}

std::uint64_t parse_index(const std::string & text)
{
  std::uint64_t value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    throw std::invalid_argument("invalid acquired_index: " + text);
  }
  return value;
}

std::vector<std::uint64_t> parse_indices(const std::string & text)
{
  if (text.empty()) throw std::invalid_argument("--indices cannot be empty");
  std::vector<std::uint64_t> values;
  std::set<std::uint64_t> unique;
  std::size_t start = 0;
  while (true) {
    const std::size_t comma = text.find(',', start);
    const std::string item = text.substr(start, comma == std::string::npos ? comma : comma - start);
    const std::uint64_t value = parse_index(item);
    if (!unique.insert(value).second) throw std::invalid_argument("duplicate --indices value");
    values.push_back(value);
    if (comma == std::string::npos) break;
    start = comma + 1U;
  }
  return values;
}

const std::string & require_value(
  const std::vector<std::string> & arguments, std::size_t & index, const char * option)
{
  if (index + 1U >= arguments.size()) {
    throw std::invalid_argument(std::string(option) + " requires a value");
  }
  return arguments[++index];
}

}  // namespace

std::string capture_usage()
{
  return "Usage: mvs_raw_capture --duration SECONDS --output-dir PATH --session-name NAME\n";
}

std::string convert_usage()
{
  return "Usage: mvs_bayer_convert --session-dir PATH --output-dir PATH "
         "[--indices I,J,...] [--allow-incomplete]\n";
}

CaptureOptions parse_capture_options(const std::vector<std::string> & arguments)
{
  CaptureOptions options;
  bool duration_seen = false;
  bool output_seen = false;
  bool session_seen = false;
  for (std::size_t index = 0; index < arguments.size(); ++index) {
    const std::string & argument = arguments[index];
    if (argument == "--help") {
      options.help = true;
      continue;
    }
    if (argument == "--duration") {
      if (duration_seen) throw std::invalid_argument("duplicate --duration");
      options.duration_seconds = parse_duration(require_value(arguments, index, "--duration"));
      duration_seen = true;
    } else if (argument == "--output-dir") {
      if (output_seen) throw std::invalid_argument("duplicate --output-dir");
      options.output_dir = require_value(arguments, index, "--output-dir");
      output_seen = true;
    } else if (argument == "--session-name") {
      if (session_seen) throw std::invalid_argument("duplicate --session-name");
      options.session_name = require_value(arguments, index, "--session-name");
      validate_session_name(options.session_name);
      session_seen = true;
    } else {
      throw std::invalid_argument("unknown capture option: " + argument);
    }
  }
  if (options.help) return options;
  if (!duration_seen || !output_seen || !session_seen) {
    throw std::invalid_argument("--duration, --output-dir and --session-name are required");
  }
  if (options.output_dir.empty()) throw std::invalid_argument("--output-dir cannot be empty");
  (void)calculate_disk_budget(options.duration_seconds);
  return options;
}

ConvertOptions parse_convert_options(const std::vector<std::string> & arguments)
{
  ConvertOptions options;
  bool session_seen = false;
  bool output_seen = false;
  bool indices_seen = false;
  for (std::size_t index = 0; index < arguments.size(); ++index) {
    const std::string & argument = arguments[index];
    if (argument == "--help") {
      options.help = true;
    } else if (argument == "--session-dir") {
      if (session_seen) throw std::invalid_argument("duplicate --session-dir");
      options.session_dir = require_value(arguments, index, "--session-dir");
      session_seen = true;
    } else if (argument == "--output-dir") {
      if (output_seen) throw std::invalid_argument("duplicate --output-dir");
      options.output_dir = require_value(arguments, index, "--output-dir");
      output_seen = true;
    } else if (argument == "--indices") {
      if (indices_seen) throw std::invalid_argument("duplicate --indices");
      options.indices = parse_indices(require_value(arguments, index, "--indices"));
      indices_seen = true;
    } else if (argument == "--allow-incomplete") {
      if (options.allow_incomplete) throw std::invalid_argument("duplicate --allow-incomplete");
      options.allow_incomplete = true;
    } else {
      throw std::invalid_argument("unknown converter option: " + argument);
    }
  }
  if (options.help) return options;
  if (!session_seen || !output_seen) {
    throw std::invalid_argument("--session-dir and --output-dir are required");
  }
  if (options.session_dir.empty() || options.output_dir.empty()) {
    throw std::invalid_argument("session and output directories cannot be empty");
  }
  return options;
}

}  // namespace mvs_raw_capture
