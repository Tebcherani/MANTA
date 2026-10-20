#ifndef MANTA_INTERFACES__STATE_LOG_HPP_
#define MANTA_INTERFACES__STATE_LOG_HPP_

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace manta {
inline void write_json(std::ostream &out, const std::string &value) {
    out << '"';
    for (unsigned char ch : value) {
        switch (ch) {
            case '"': out << "\\\""; break;
            case '\\': out << "\\\\"; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if (ch < 0x20) {
                    const char *hex = "0123456789abcdef";
                    out << "\\u00" << hex[ch >> 4] << hex[ch & 15];
                } else out << ch;
        }
    }
    out << '"';
}
inline void write_json(std::ostream &out, bool value) { out << (value ? "true" : "false"); }
template<class T, std::enable_if_t<std::is_integral<T>::value && !std::is_same<T, bool>::value, int> = 0>
void write_json(std::ostream &out, T value) { out << value; }
inline void write_json(std::ostream &out, double value) {
    if (std::isfinite(value)) out << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    else write_json(out, std::isnan(value) ? std::string("NaN") : (value > 0 ? std::string("Infinity") : std::string("-Infinity")));
}
template<class T> void write_random_json(std::ostream &out, const T &value) {
    std::ostringstream encoded;
    encoded.imbue(std::locale::classic());
    encoded << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    write_json(out, encoded.str());
}
template<class T, std::size_t W, std::size_t N, std::size_t M, std::size_t R, T A, std::size_t U, T D, std::size_t S, T B, std::size_t TT, T C, std::size_t L, T F>
void write_json(std::ostream &out, const std::mersenne_twister_engine<T, W, N, M, R, A, U, D, S, B, TT, C, L, F> &value) { write_random_json(out, value); }
template<class T> void write_json(std::ostream &out, const std::normal_distribution<T> &value) { write_random_json(out, value); }
// ADL finds serializers defined beside simulation structs and ROS messages.
template<class T> void write_json(std::ostream &out, const std::vector<T> &values) {
    out << '[';
    bool first = true;
    for (const auto &value : values) {
        if (!first) out << ',';
        first = false;
        write_json(out, value);
    }
    out << ']';
}
template<class T> void write_json(std::ostream &out, const std::unordered_map<std::string, T> &values) {
    std::vector<std::string> keys;
    for (const auto &entry : values) keys.push_back(entry.first);
    std::sort(keys.begin(), keys.end());
    out << '{';
    bool first = true;
    for (const auto &key : keys) {
        if (!first) out << ',';
        first = false;
        write_json(out, key);
        out << ':';
        write_json(out, values.at(key));
    }
    out << '}';
}
template<class T> void json_field(std::ostream &out, const char *name, const T &value, bool &first) {
    if (!first) out << ',';
    first = false;
    write_json(out, std::string(name));
    out << ':';
    write_json(out, value);
}

// One complete JSON object per changed state, flushed before returning.
class StateLog {
public:
    void open_in_log_directory(const std::string &filename) {
        const char *configured = std::getenv("ROS_LOG_DIR");
        const std::filesystem::path directory = configured && *configured
            ? configured : "/odyssey/output/Logs";
        std::filesystem::create_directories(directory);
        open((directory / filename).string());
    }
    void open(const std::string &path) {
        stream_.open(path, std::ios::out | std::ios::trunc);
        if (!stream_) throw std::runtime_error("Cannot open state log: " + path);
        stream_.imbue(std::locale::classic());
        previous_.clear();
    }
    template<class T> void record(const T &state) {
        const auto encode = [](std::ostream &out, const T &value) { write_json(out, value); };
        record(state, encode, encode);
    }
    // Compare compact runtime snapshots, but write precomputed data on the first record.
    template<class T, class RuntimeEncoder, class InitialEncoder>
    void record(const T &state, RuntimeEncoder runtime_encoder, InitialEncoder initial_encoder) {
        std::ostringstream snapshot;
        snapshot.imbue(std::locale::classic());
        runtime_encoder(snapshot, state);
        const auto current = snapshot.str();
        if (current == previous_) return;
        if (previous_.empty()) {
            std::ostringstream initial;
            initial.imbue(std::locale::classic());
            initial_encoder(initial, state);
            stream_ << initial.str() << '\n';
        } else {
            stream_ << current << '\n';
        }
        stream_.flush();
        if (!stream_) throw std::runtime_error("Cannot write simulation state log.");
        previous_ = current;
    }

private:
    std::ofstream stream_;
    std::string previous_;
};
}  // namespace manta

#endif  // MANTA_INTERFACES__STATE_LOG_HPP_
