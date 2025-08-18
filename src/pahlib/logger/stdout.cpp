#include <iostream>
#include <ostream>

#include "pahlib/logger/stdout.hpp"

namespace pahlib {
BufferedStdout::BufferedStdout()
    : Buffer([](const std::string& text) { std::cout << text << std::flush; }) {
    setRate(50);
}

BufferedStdout& bufferedStdout() {
    static BufferedStdout bufferedStdout;
    return bufferedStdout;
}
} // namespace pahlib