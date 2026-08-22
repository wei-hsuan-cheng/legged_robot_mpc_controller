#ifndef LEGGED_ROBOT_MPC_CONTROLLER__COMMON__DIAGNOSTICS_CSV_LOGGER_HPP_
#define LEGGED_ROBOT_MPC_CONTROLLER__COMMON__DIAGNOSTICS_CSV_LOGGER_HPP_

#include <atomic>
#include <cstddef>
#include <string>
#include <thread>
#include <vector>

namespace legged_robot_mpc_controller
{

/**
 * Fixed-schema CSV logger that can be fed from the real-time control loop.
 *
 * The controller runs at 1 kHz and the questions this logging exists to answer
 * (filter consistency, contact transitions, per-cost-term balance) are all
 * per-tick questions, so the samples have to be taken in the control thread. But
 * that thread must not touch the filesystem, allocate, or take a lock that a
 * writer could be holding. So the control thread only ever memcpys doubles into
 * a preallocated single-producer/single-consumer ring buffer, and a background
 * thread does the formatting and the write().
 *
 * The schema is fixed at open(): every row has the same width as the column
 * list, which is what lets rows be plain double spans with no per-row metadata.
 * Unset entries default to NaN and are written as empty fields, which pandas and
 * numpy both read back as NaN.
 *
 * Usage from the control thread:
 * @code
 *   if (double* row = logger.beginRow()) {
 *     row[kTimeColumn] = time;
 *     ...
 *     logger.commitRow();
 *   }
 * @endcode
 * beginRow() returns nullptr when the consumer has fallen behind; the sample is
 * then dropped and counted rather than blocking the control loop. Check
 * droppedRows() after a run - a non-zero count means the log has gaps and any
 * rate-derived quantity computed from it is suspect.
 *
 * Exactly one producer thread may call beginRow()/commitRow(). open() and
 * close() reallocate the ring and join the writer, so they must not overlap a
 * beginRow()/commitRow() pair - which holds when they are called from
 * on_activate()/on_deactivate(), since ros2_control serializes those against
 * update().
 */
class DiagnosticsCsvLogger
{
public:
  DiagnosticsCsvLogger() = default;
  ~DiagnosticsCsvLogger();

  DiagnosticsCsvLogger(const DiagnosticsCsvLogger &) = delete;
  DiagnosticsCsvLogger & operator=(const DiagnosticsCsvLogger &) = delete;

  /**
   * Fix the schema, create the file (with parent directories) and start the
   * writer thread. Not real-time safe - call from on_activate().
   *
   * @param path        output file; parent directories are created if missing
   * @param columns     column names, also fixing the row width
   * @param buffer_rows ring capacity in rows; at 1 kHz a few thousand rows is
   *                    several seconds of slack against a stalled disk write
   * @throws std::runtime_error if the file cannot be opened
   */
  void open(const std::string & path, std::vector<std::string> columns, std::size_t buffer_rows);

  /// Flush, stop the writer thread and close the file. Safe to call twice.
  void close();

  bool isOpen() const { return running_.load(std::memory_order_acquire); }

  /// Column count, i.e. the width of every row returned by beginRow().
  std::size_t width() const { return width_; }

  /**
   * Claim the next row. Real-time safe: no allocation, no locks, no syscalls.
   * The returned span is width() doubles, pre-filled with NaN. Must be followed
   * by commitRow() before the next beginRow(). Returns nullptr when the logger
   * is closed or the ring is full (the sample is dropped and counted).
   */
  double * beginRow();

  /// Publish the row claimed by the last beginRow() to the writer thread.
  void commitRow();

  /// Samples dropped because the ring was full. Non-zero means the log has gaps.
  std::size_t droppedRows() const { return dropped_.load(std::memory_order_relaxed); }

  /// Rows handed to the operating system so far.
  std::size_t writtenRows() const { return written_.load(std::memory_order_relaxed); }

  const std::string & path() const { return path_; }

private:
  void writerLoop();
  /// Append one row to out_buffer_ as CSV. Runs on the writer thread only.
  void formatRow(const double * row, std::string & out_buffer) const;

  std::string path_;
  std::size_t width_{0};
  std::size_t capacity_{0};   //!< ring capacity in rows

  //! capacity_ * width_ doubles. One slot is always left empty so that
  //! head == tail unambiguously means "empty" without a separate count.
  std::vector<double> ring_;

  std::atomic<std::size_t> head_{0};   //!< next slot the producer will fill
  std::atomic<std::size_t> tail_{0};   //!< next slot the consumer will read
  std::size_t pending_head_{0};        //!< producer-only: slot claimed by beginRow()

  std::atomic<bool> running_{false};
  std::atomic<std::size_t> dropped_{0};
  std::atomic<std::size_t> written_{0};

  std::thread writer_;
};

}  // namespace legged_robot_mpc_controller

#endif  // LEGGED_ROBOT_MPC_CONTROLLER__COMMON__DIAGNOSTICS_CSV_LOGGER_HPP_
