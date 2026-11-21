#pragma once

#include <array>
#include <string>
#include <vector>

class DataFile {
public:
    struct Measurement {
        std::string timestamp;  // OEM epoch as written in the file.
        std::array<double, 6> state;  // x, y, z (km), vx, vy, vz (km/s).
    };

    std::vector<Measurement> data_parser(const std::string& path) const;
};
