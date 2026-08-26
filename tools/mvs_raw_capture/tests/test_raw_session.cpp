#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <vector>

#include "mvs_raw_capture/cli.hpp"
#include "mvs_raw_capture/converter.hpp"
#include "mvs_raw_capture/session.hpp"

namespace fs = std::filesystem;
using namespace mvs_raw_capture;

namespace
{
void require(bool condition, const std::string & message)
{
  if (!condition) throw std::runtime_error(message);
}

template <typename Function>
void require_throws(Function function, const std::string & message)
{
  try {
    function();
  } catch (const std::exception &) {
    return;
  }
  throw std::runtime_error(message);
}

void write_binary(const fs::path & path, const std::vector<std::vector<unsigned char>> & frames)
{
  std::ofstream output(path, std::ios::binary | std::ios::trunc);
  require(static_cast<bool>(output), "cannot create synthetic Bayer data");
  for (const auto & frame : frames) {
    output.write(reinterpret_cast<const char *>(frame.data()), frame.size());
  }
  output.close();
  require(static_cast<bool>(output), "cannot finish synthetic Bayer data");
}

}  // namespace

int main()
{
  const fs::path root = fs::current_path() / ("test_raw_session_" + std::to_string(getpid()));
  fs::remove_all(root);
  try {
    const CaptureOptions capture = parse_capture_options(
      {"--duration", "2.5", "--output-dir", root.string(), "--session-name", "unit_001"});
    require(capture.duration_seconds == 2.5, "capture duration parse failed");
    require_throws(
      []() {
        parse_capture_options({"--duration", "1"});
      },
      "missing capture CLI was accepted");
    require_throws(
      []() {
        parse_capture_options(
          {"--duration", "1", "--output-dir", "/tmp", "--session-name", "../bad"});
      },
      "unsafe session name was accepted");
    require_throws(
      [&]() {
        parse_capture_options(
          {"--duration", "1e9999", "--output-dir", root.string(), "--session-name", "bad"});
      },
      "out-of-range duration was accepted");

    SessionManifest exponent_manifest;
    exponent_manifest.session_name = "exponent_roundtrip";
    exponent_manifest.created_at_utc = utc_timestamp();
    exponent_manifest.requested_duration_seconds = 1.0e-9;
    const SessionManifest exponent_parsed = parse_manifest_json(manifest_json(exponent_manifest));
    require(
      exponent_parsed.requested_duration_seconds == exponent_manifest.requested_duration_seconds,
      "scientific-notation duration did not round-trip");

    SensorCounterTracker tracker;
    const std::vector<std::uint32_t> sensor_ids = {100U, 101U, 103U, 5U, 5U, 6U};
    const std::vector<CounterEvent> expected = {CounterEvent::none,   CounterEvent::none,
                                                CounterEvent::gap,    CounterEvent::reset,
                                                CounterEvent::repeat, CounterEvent::none};
    std::vector<CounterObservation> observations;
    for (std::size_t index = 0; index < sensor_ids.size(); ++index) {
      observations.push_back(tracker.observe(sensor_ids[index]));
      require(observations.back().event == expected[index], "counter event mismatch");
    }
    require(tracker.gap_events() == 1U, "gap event count mismatch");
    require(tracker.missing_frames() == 1U, "missing frame count mismatch");
    require(tracker.reset_events() == 1U, "reset event count mismatch");

    fs::create_directories(root);
    const SessionPaths paths = create_session_directory(root, "synthetic");
    require_throws(
      [&]() { create_session_directory(root, "synthetic"); },
      "existing session directory was overwritten");

    SessionManifest manifest;
    manifest.status = "complete";
    manifest.session_name = "synthetic";
    manifest.created_at_utc = utc_timestamp();
    manifest.completed_at_utc = manifest.created_at_utc;
    manifest.incomplete_reason.clear();
    manifest.requested_duration_seconds = 1.0;
    manifest.current_reference_estimated_bytes = 144U;
    manifest.worst_case_estimated_bytes = 576U;
    manifest.width = 8U;
    manifest.height = 6U;
    manifest.payload_size = 48U;
    manifest.written_frames = 3U;
    manifest.acquired_frames = 3U;

    std::vector<std::vector<unsigned char>> frames(3U, std::vector<unsigned char>(48U));
    for (std::size_t frame = 0; frame < frames.size(); ++frame) {
      for (std::size_t byte = 0; byte < frames[frame].size(); ++byte) {
        frames[frame][byte] = static_cast<unsigned char>((frame * 31U + byte * 7U) % 256U);
      }
    }
    write_binary(paths.data, frames);

    std::ofstream csv(paths.frames_csv);
    csv << frame_csv_header() << '\n';
    for (std::size_t index = 0; index < frames.size(); ++index) {
      FrameRecord record;
      record.acquired_index = index;
      record.sensor_frame_id = sensor_ids[index == 2U ? 2U : index];
      record.counter_source = CounterSource::frame_number;
      record.sensor_timestamp = 1000U + index;
      record.host_receive_timestamp_ns = 2000U + index;
      record.file_byte_offset = index * 48U;
      record.payload_size = 48U;
      record.crc32 = crc32_bytes(frames[index].data(), frames[index].size());
      record.counter_event = index == 2U ? CounterEvent::gap : CounterEvent::none;
      record.previous_sensor_frame_id = index == 2U ? 101U : 0U;
      record.missing_count = index == 2U ? 1U : 0U;
      csv << frame_record_csv(record) << '\n';
    }
    csv.close();
    std::ofstream(paths.counter_events_csv) << "acquired_index,event\n2,GAP\n";
    write_text_atomic_synced(paths.capture_result, "{}\n");
    write_text_atomic_synced(paths.manifest, manifest_json(manifest));

    LoadedSession loaded = load_session(paths.directory, false);
    require(loaded.records.size() == 3U, "session record count mismatch");
    for (std::size_t index = 0; index < loaded.records.size(); ++index) {
      require(loaded.records[index].acquired_index == index, "acquired_index lost continuity");
    }
    const std::vector<unsigned char> selected = read_frame_at(paths.data, loaded.records[1], true);
    require(selected == frames[1], "offset frame read mismatch");
    cv::Mat bayer(6, 8, CV_8UC1, const_cast<unsigned char *>(selected.data()));
    cv::Mat bgr;
    cv::cvtColor(bayer, bgr, cv::COLOR_BayerRG2BGR);
    require(bgr.rows == 6 && bgr.cols == 8 && bgr.channels() == 3, "synthetic decode failed");

    manifest.status = "incomplete";
    manifest.incomplete_reason = "unit_test";
    write_text_atomic_synced(paths.manifest, manifest_json(manifest));
    require_throws(
      [&]() { load_session(paths.directory, false); },
      "incomplete session was accepted by default");
    require(load_session(paths.directory, true).records.size() == 3U, "incomplete override failed");

    manifest.status = "complete";
    manifest.incomplete_reason.clear();
    write_text_atomic_synced(paths.manifest, manifest_json(manifest));
    ConvertOptions convert;
    convert.session_dir = paths.directory;
    convert.output_dir = root / "converted";
    convert.indices = {2U, 0U};
    const ConversionResult conversion = convert_session(convert);
    require(conversion.exported_frames == 2U, "converter frame count mismatch");
    require(
      conversion.acquired_indices == std::vector<std::uint64_t>({0U, 2U}),
      "converter did not sort acquired indices");
    require(fs::exists(convert.output_dir / "frame_000000000002.png"), "index 2 name missing");
    require(fs::exists(convert.output_dir / "frame_000000000000.png"), "index 0 name missing");
    std::ifstream metadata(conversion.metadata_path);
    std::string metadata_header;
    std::string metadata_first;
    std::string metadata_second;
    require(
      static_cast<bool>(std::getline(metadata, metadata_header)) &&
        static_cast<bool>(std::getline(metadata, metadata_first)) &&
        static_cast<bool>(std::getline(metadata, metadata_second)),
      "converter metadata rows missing");
    require(metadata_first.rfind("0,", 0U) == 0U, "metadata first index is not 0");
    require(metadata_second.rfind("2,", 0U) == 0U, "metadata second index is not 2");
    require_throws(
      [&]() { convert_session(convert); }, "existing conversion output was overwritten");

    fs::remove_all(root);
    std::cout << "test_raw_session: PASS\n";
    return 0;
  } catch (const std::exception & error) {
    fs::remove_all(root);
    std::cerr << "test_raw_session: FAIL: " << error.what() << '\n';
    return 1;
  }
}
