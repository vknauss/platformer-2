#pragma once

#include <fstream>
#include <stdexcept>
#include <string>


namespace utils {

// forward decls

inline std::string file_string(const std::string& path);

inline std::string file_string(const std::string& path) {
    if (auto ifs = std::ifstream(path)) {
        return std::string {std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>()};
    }
    throw std::runtime_error("Failed to load text file " + path);
}


}  // namespace utils