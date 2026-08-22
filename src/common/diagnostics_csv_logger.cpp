#include "legged_robot_mpc_controller/common/diagnostics_csv_logger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <utility>

namespace legged_robot_mpc_controller
{

namespace
{
//! Ring slack. One slot is sacrificed so head == tail means empty.
constexpr std::size_t kMinimumBufferRows = 16;
//! How long the writer sleeps when the ring is empty. Long enough not to spin,
//! short enough that close() does not stall noticeably.
constexpr auto kWriterIdleSleep = std::chrono::milliseconds(2);
}  // namespace

DiagnosticsCsvLogger::~DiagnosticsCsvLogger()
{
  close();
}

void DiagnosticsCsvLogger::open(
  const std::string & path, std::vector<std::string> columns, std::size_t buffer_rows)
{
  close();

  if (columns.empty()) {
    throw std::runtime_error("[DiagnosticsCsvLogger] refusing to open with an empty column list");
  }

  const std::filesystem::path file_path(path);
  if (file_path.has_parent_path()) {
    std::error_code ec;
    std::filesystem::create_directories(file_path.parent_path(), ec);
    if (ec) {
      throw std::runtime_error(
              "[DiagnosticsCsvLogger] cannot create directory '" +
              file_path.parent_path().string() + "': " + ec.message());
    }
  }

  // Write the header eagerly so a failure surfaces at activation rather than
  // silently producing an empty log after a long run.
  {
    std::ofstream header_stream(path, std::ios::out | std::ios::trunc);
    if (!header_stream) {
      throw std::runtime_error("[DiagnosticsCsvLogger] cannot open '" + path + "' for writing");
    }
    for (std::size_t i = 0; i < columns.size(); ++i) {
      if (i != 0) {
        header_stream << ',';
      }
      header_stream << columns[i];
    }
    header_stream << '\n';
    if (!header_stream) {
      throw std::runtime_error("[DiagnosticsCsvLogger] failed writing the header of '" + path + "'");
    }
  }

  path_ = path;
  width_ = columns.size();
  capacity_ = std::max(buffer_rows, kMinimumBufferRows);
  ring_.assign(capacity_ * width_, std::numeric_limits<double>::quiet_NaN());
  head_.store(0, std::memory_order_relaxed);
  tail_.store(0, std::memory_order_relaxed);
  pending_head_ = 0;
  dropped_.store(0, std::memory_order_relaxed);
  written_.store(0, std::memory_order_relaxed);

  running_.store(true, std::memory_order_release);
  writer_ = std::thread([this] {writerLoop();});
}

void DiagnosticsCsvLogger::close()
{
  if (!running_.exchange(false, std::memory_order_acq_rel)) {
    return;
  }
  if (writer_.joinable()) {
    writer_.join();
  }
}

double * DiagnosticsCsvLogger::beginRow()
{
  if (!running_.load(std::memory_order_acquire)) {
    return nullptr;
  }
  const std::size_t head = head_.load(std::memory_order_relaxed);
  const std::size_t next = (head + 1) % capacity_;
  if (next == tail_.load(std::memory_order_acquire)) {
    // The writer has fallen behind. Dropping the sample keeps the control loop
    // deterministic; the count makes the gap visible afterwards.
    dropped_.fetch_add(1, std::memory_order_relaxed);
    return nullptr;
  }
  pending_head_ = head;
  double * row = ring_.data() + head * width_;
  // Pre-fill so a caller that sets only some columns still produces a row whose
  // unset fields read back as NaN rather than as the previous occupant's values.
  for (std::size_t i = 0; i < width_; ++i) {
    row[i] = std::numeric_limits<double>::quiet_NaN();
  }
  return row;
}

void DiagnosticsCsvLogger::commitRow()
{
  head_.store((pending_head_ + 1) % capacity_, std::memory_order_release);
}

void DiagnosticsCsvLogger::formatRow(const double * row, std::string & out_buffer) const
{
  char field[40];
  for (std::size_t i = 0; i < width_; ++i) {
    if (i != 0) {
      out_buffer.push_back(',');
    }
    const double value = row[i];
    // NaN is how an unset column is represented; write it as an empty field so
    // pandas/numpy read it back as NaN without a converter.
    if (std::isnan(value)) {
      continue;
    }
    // %.10g keeps a double's decimal significance for the magnitudes involved
    // here without the width of a full round-trip representation.
    const int n = std::snprintf(field, sizeof(field), "%.10g", value);
    if (n > 0) {
      out_buffer.append(field, static_cast<std::size_t>(n));
    }
  }
  out_buffer.push_back('\n');
}

void DiagnosticsCsvLogger::writerLoop()
{
  std::ofstream stream(path_, std::ios::out | std::ios::app);
  if (!stream) {
    running_.store(false, std::memory_order_release);
    return;
  }

  std::string out_buffer;
  // Roughly one flush worth of rows, so the common case appends without growing.
  out_buffer.reserve(width_ * 16 * 64);

  const auto drain = [&] {
      std::size_t drained = 0;
      std::size_t tail = tail_.load(std::memory_order_relaxed);
      while (tail != head_.load(std::memory_order_acquire)) {
        formatRow(ring_.data() + tail * width_, out_buffer);
        tail = (tail + 1) % capacity_;
        // Publishing the tail per row (rather than once at the end) frees ring
        // slots while a long drain is still in progress.
        tail_.store(tail, std::memory_order_release);
        ++drained;
      }
      if (!out_buffer.empty()) {
        stream.write(out_buffer.data(), static_cast<std::streamsize>(out_buffer.size()));
        out_buffer.clear();
        written_.fetch_add(drained, std::memory_order_relaxed);
      }
      return drained;
    };

  while (running_.load(std::memory_order_acquire)) {
    if (drain() == 0) {
      std::this_thread::sleep_for(kWriterIdleSleep);
    }
  }
  // running_ is already false here, so no new rows can be committed; drain what
  // the producer left behind before closing.
  drain();
  stream.flush();
}

}  // namespace legged_robot_mpc_controller
