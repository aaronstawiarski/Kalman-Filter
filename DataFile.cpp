#include "DataFile.h"

#include <cmath>
#include <fstream>
#include <locale>
#include <regex>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {
bool read_number(std::istream& row, double& value) {
    std::string field;
    if (!(row >> field)) {
        return false;
    }

    std::istringstream number(field);
    number.imbue(std::locale::classic());
    return (number >> value) && number.eof() && std::isfinite(value);
}
}

std::vector<DataFile::Measurement> DataFile::data_parser(const std::string& path) const {
    std::ifstream file(path);
    if (!file) {
        throw std::runtime_error("Could not open " + path);
    }

    static const std::regex epoch(
        R"([0-9]{4}-([0-9]{2}-[0-9]{2}|[0-9]{3})T[0-9]{2}:[0-9]{2}:[0-9]{2}(\.[0-9]+)?Z?)");
    enum class Section { header, metadata, ephemeris, covariance };
    Section section = Section::header;
    std::vector<Measurement> measurements;
    std::string line;
    std::size_t line_number = 0;
    const auto error_at_line = [&](const char* message) {
        return std::runtime_error(path + ":" + std::to_string(line_number) + ": " + message);
    };

    while (std::getline(file, line)) {
        ++line_number;
        std::istringstream row(line);
        std::string first;
        if (!(row >> first) || first == "COMMENT") {
            continue;
        }

        if (first == "META_START") {
            if (section == Section::metadata || section == Section::covariance) {
                throw error_at_line("Unexpected META_START");
            }
            section = Section::metadata;
        } else if (first == "META_STOP") {
            if (section != Section::metadata) {
                throw error_at_line("META_STOP without META_START");
            }
            section = Section::ephemeris;
        } else if (first == "COVARIANCE_START") {
            if (section != Section::ephemeris) {
                throw error_at_line("Unexpected COVARIANCE_START");
            }
            section = Section::covariance;
        } else if (first == "COVARIANCE_STOP") {
            if (section != Section::covariance) {
                throw error_at_line("COVARIANCE_STOP without COVARIANCE_START");
            }
            section = Section::ephemeris;
        } else if (section == Section::ephemeris) {
            if (!std::regex_match(first, epoch)) {
                throw error_at_line("Invalid measurement timestamp");
            }

            Measurement measurement{std::move(first), {}};
            for (double& component : measurement.state) {
                if (!read_number(row, component)) {
                    throw error_at_line("Expected six numeric position and velocity components");
                }
            }

            row >> std::ws;
            if (!row.eof()) {
                // An OEM row may also supply three acceleration components.
                std::array<double, 3> acceleration;
                for (double& component : acceleration) {
                    if (!read_number(row, component)) {
                        throw error_at_line("Expected three optional acceleration components");
                    }
                }
                std::string extra;
                if (row >> extra) {
                    throw error_at_line("Unexpected field after acceleration components");
                }
            }

            measurements.push_back(std::move(measurement));
        }
    }

    if (file.bad() || !file.eof()) {
        throw error_at_line("Could not read file");
    }
    if (section == Section::metadata || section == Section::covariance) {
        throw error_at_line("Unterminated metadata or covariance block");
    }
    if (measurements.empty()) {
        throw std::runtime_error(path + ": No ephemeris measurements found");
    }

    return measurements;
}
