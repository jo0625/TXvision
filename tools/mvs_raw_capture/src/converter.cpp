#include "mvs_raw_capture/converter.hpp"

#include <algorithm>
#include <iomanip>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <stdexcept>

#include "mvs_raw_capture/session.hpp"

namespace mvs_raw_capture
{
namespace
{
std::string conversion_status_json(
  const char * status, const LoadedSession & session, std::uint64_t exported_frames,
  const std::string & reason)
{
  std::ostringstream output;
  output << "{\n"
         << "  \"status\": \"" << json_escape(status) << "\",\n"
         << "  \"source_session\": \"" << json_escape(session.paths.directory.string()) << "\",\n"
         << "  \"source_session_status\": \"" << json_escape(session.manifest.status) << "\",\n"
         << "  \"exported_frames\": " << exported_frames << ",\n"
         << "  \"naming_key\": \"acquired_index\",\n"
         << "  \"bayer_conversion\": \"BayerRG8_to_BGR\",\n"
         << "  \"reason\": \"" << json_escape(reason) << "\"\n"
         << "}\n";
  return output.str();
}

std::string output_name(std::uint64_t acquired_index)
{
  std::ostringstream output;
  output << "frame_" << std::setw(12) << std::setfill('0') << acquired_index << ".png";
  return output.str();
}

}  // namespace

ConversionResult convert_session(const ConvertOptions & options)
{
  LoadedSession session = load_session(options.session_dir, options.allow_incomplete);
  if (
    session.manifest.pixel_format != kBaselinePixelFormat ||
    session.manifest.payload_size !=
      static_cast<std::uint64_t>(session.manifest.width) * session.manifest.height) {
    throw std::runtime_error("session is not unpacked BayerRG8");
  }
  std::error_code error;
  if (!fs::create_directory(options.output_dir, error)) {
    if (error) throw std::runtime_error("cannot create conversion output: " + error.message());
    throw std::runtime_error("conversion output directory already exists");
  }

  ConversionResult result;
  result.metadata_path = options.output_dir / "metadata.csv";
  result.status_path = options.output_dir / "conversion.json";
  write_text_atomic_synced(
    result.status_path,
    conversion_status_json("incomplete", session, 0, "conversion_not_completed"));

  std::vector<std::uint64_t> selected = options.indices;
  if (selected.empty()) {
    selected.reserve(session.records.size());
    for (const FrameRecord & record : session.records) selected.push_back(record.acquired_index);
  }
  std::sort(selected.begin(), selected.end());
  if (std::adjacent_find(selected.begin(), selected.end()) != selected.end()) {
    throw std::invalid_argument("duplicate acquired_index in conversion selection");
  }

  std::ostringstream metadata;
  metadata << frame_csv_header() << ",output_file\n";
  for (const std::uint64_t acquired_index : selected) {
    if (acquired_index >= session.records.size()) {
      throw std::runtime_error(
        "requested acquired_index is outside the session: " + std::to_string(acquired_index));
    }
    const FrameRecord & record = session.records[acquired_index];
    if (record.acquired_index != acquired_index) {
      throw std::runtime_error("CSV lookup is not keyed by acquired_index");
    }
    std::vector<unsigned char> raw = read_frame_at(session.paths.data, record, true);
    cv::Mat bayer(
      static_cast<int>(session.manifest.height), static_cast<int>(session.manifest.width), CV_8UC1,
      raw.data());
    cv::Mat bgr;
    cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2BGR);
    const std::string filename = output_name(acquired_index);
    if (!cv::imwrite((options.output_dir / filename).string(), bgr)) {
      throw std::runtime_error(
        "image export failed for acquired_index=" + std::to_string(acquired_index));
    }
    metadata << frame_record_csv(record) << ',' << filename << '\n';
    result.acquired_indices.push_back(acquired_index);
    ++result.exported_frames;
  }

  write_text_atomic_synced(result.metadata_path, metadata.str());
  write_text_atomic_synced(
    result.status_path, conversion_status_json("complete", session, result.exported_frames, ""));
  return result;
}

}  // namespace mvs_raw_capture
