#include <iostream>
#include <ostream>

#include "lemlib/logger/stdout.hpp"

namespace lemlib {
BufferedStdout::BufferedStdout()
    // Do NOT write std::cout on this branch. USB serial COBS (ser_write_r →
    // cobs_encode_measure) data-aborts; LemLib's unnamed Buffer task shows as
    // CURRENT TASK: "." on the brain when that happens (e.g. IMU calibrate warn).
    : Buffer([](const std::string& text) { (void)text; }) {
    setRate(50);
}

BufferedStdout& bufferedStdout() {
    static BufferedStdout bufferedStdout;
    return bufferedStdout;
}
} // namespace lemlib