#include <boost/interprocess/file_mapping.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <nlohmann/json.hpp>
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>

using namespace boost::interprocess;
using json = nlohmann::json;
/*
int main() {
    const char* filename = "Pythonhalf/shared_memory.dat";

    for (int attempt = 0; attempt < 10; ++attempt) {
        try {
            file_mapping fm(filename, read_only);
            mapped_region region(fm, read_only);

            const char* mem = static_cast<const char*>(region.get_address());
            std::string json_data(mem, strnlen(mem, region.get_size()));

            if (json_data.empty()) {
                std::cout << "No data yet.\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            auto parsed = json::parse(json_data);
            for (const auto& hand : parsed) {
                std::cout << "Hand: " << hand["handedness"] << "\n";
                const auto& landmarks = hand["landmarks"];
                std::cout << "Landmark[0] x: " << landmarks[0]["x"] << "\n";
            }

            break;
        }
        catch (const interprocess_exception& e) {
            std::cerr << "Shared memory error: " << e.what() << "\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        catch (const json::parse_error& e) {
            std::cerr << "JSON parse error: " << e.what() << "\n";
        }
    }

    return 0;
}
*/