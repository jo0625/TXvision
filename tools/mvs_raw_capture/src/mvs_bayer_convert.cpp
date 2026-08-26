#include <iostream>
#include <string>
#include <vector>

#include "mvs_raw_capture/cli.hpp"
#include "mvs_raw_capture/converter.hpp"

int main(int argc, char ** argv)
{
  std::vector<std::string> arguments;
  for (int index = 1; index < argc; ++index) arguments.emplace_back(argv[index]);
  try {
    const mvs_raw_capture::ConvertOptions options =
      mvs_raw_capture::parse_convert_options(arguments);
    if (options.help) {
      std::cout << mvs_raw_capture::convert_usage();
      return 0;
    }
    const mvs_raw_capture::ConversionResult result = mvs_raw_capture::convert_session(options);
    std::cout << "status=PASS exported_frames=" << result.exported_frames
              << " metadata=" << result.metadata_path << '\n';
    return 0;
  } catch (const std::invalid_argument & error) {
    std::cerr << "CLI_ERROR: " << error.what() << '\n' << mvs_raw_capture::convert_usage();
    return 2;
  } catch (const std::exception & error) {
    std::cerr << "CONVERSION_ERROR: " << error.what() << '\n';
    return 1;
  }
}
