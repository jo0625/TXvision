#include <charconv>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/imgproc.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include "mvs_raw_capture/session.hpp"

namespace fs = std::filesystem;
using mvs_raw_capture::FrameRecord;

namespace
{
std::vector<std::string> split(const std::string & text, char delimiter)
{
  std::vector<std::string> values;
  std::size_t start = 0;
  while (true) {
    const std::size_t found = text.find(delimiter, start);
    values.push_back(text.substr(start, found == std::string::npos ? found : found - start));
    if (found == std::string::npos) break;
    start = found + 1U;
  }
  return values;
}

std::uint64_t parse_uint(const std::string & text)
{
  std::uint64_t value = 0;
  const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    throw std::runtime_error("invalid integer: " + text);
  }
  return value;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    if (
      argc != 7 || std::string(argv[1]) != "--bayer" || std::string(argv[3]) != "--csv" ||
      std::string(argv[5]) != "--indices") {
      throw std::invalid_argument(
        "usage: test_diag26_decode --bayer PATH --csv PATH --indices I,J,K");
    }
    const fs::path bayer_path = argv[2];
    const fs::path csv_path = argv[4];
    std::set<std::uint64_t> requested;
    for (const std::string & value : split(argv[6], ',')) requested.insert(parse_uint(value));
    if (requested.empty()) throw std::invalid_argument("no indices requested");

    std::ifstream csv(csv_path);
    if (!csv) throw std::runtime_error("cannot open FPS_DIAG_26 CSV");
    std::string line;
    if (!std::getline(csv, line)) throw std::runtime_error("FPS_DIAG_26 CSV header missing");
    const std::string expected =
      "written_index,acquired_index,sensor_frame_id,sensor_timestamp,host_receive_timestamp_ns,"
      "file_byte_offset,payload_size,source_crc32";
    if (line != expected) throw std::runtime_error("FPS_DIAG_26 CSV header mismatch");
    std::map<std::uint64_t, FrameRecord> records;
    while (std::getline(csv, line) && records.size() < requested.size()) {
      const std::vector<std::string> fields = split(line, ',');
      if (fields.size() != 8U) throw std::runtime_error("FPS_DIAG_26 CSV row mismatch");
      const std::uint64_t acquired_index = parse_uint(fields[1]);
      if (requested.count(acquired_index) == 0U) continue;
      FrameRecord record;
      record.acquired_index = acquired_index;
      record.sensor_frame_id = static_cast<std::uint32_t>(parse_uint(fields[2]));
      record.sensor_timestamp = parse_uint(fields[3]);
      record.host_receive_timestamp_ns = parse_uint(fields[4]);
      record.file_byte_offset = parse_uint(fields[5]);
      record.payload_size = parse_uint(fields[6]);
      record.crc32 = static_cast<std::uint32_t>(parse_uint(fields[7]));
      records.emplace(acquired_index, record);
    }
    if (records.size() != requested.size())
      throw std::runtime_error("requested CSV rows not found");

    for (const std::uint64_t index : requested) {
      const std::vector<unsigned char> raw =
        mvs_raw_capture::read_frame_at(bayer_path, records.at(index), true);
      if (raw.size() != mvs_raw_capture::kBaselinePayloadSize) {
        throw std::runtime_error("FPS_DIAG_26 payload size mismatch");
      }
      cv::Mat bayer(
        static_cast<int>(mvs_raw_capture::kBaselineHeight),
        static_cast<int>(mvs_raw_capture::kBaselineWidth), CV_8UC1,
        const_cast<unsigned char *>(raw.data()));
      cv::Mat bgr;
      cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2BGR);
      if (
        bgr.rows != static_cast<int>(mvs_raw_capture::kBaselineHeight) ||
        bgr.cols != static_cast<int>(mvs_raw_capture::kBaselineWidth) || bgr.channels() != 3) {
        throw std::runtime_error("FPS_DIAG_26 Bayer decode shape mismatch");
      }
      std::cout << "decoded acquired_index=" << index << " crc32=" << records.at(index).crc32
                << '\n';
    }
    std::cout << "test_diag26_decode: PASS frames=" << requested.size() << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "test_diag26_decode: FAIL: " << error.what() << '\n';
    return 1;
  }
}
