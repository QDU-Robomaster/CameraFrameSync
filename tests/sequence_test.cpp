#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "CameraFrameSyncCore.hpp"

namespace
{
void Expect(bool condition, const char* message)
{
  if (!condition)
  {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void TestFixedPeriodStride()
{
  using CameraFrameSyncCore::MatchImageGapStride;
  Expect(MatchImageGapStride(10000U, 10000U, 128U) == 1U,
         "one fixed-period gap must produce stride 1");
  Expect(MatchImageGapStride(20000U, 10000U, 128U) == 2U,
         "two fixed-period gaps must report one missing image");
  Expect(MatchImageGapStride(30000U, 10000U, 128U) == 3U,
         "three fixed-period gaps must report two missing images");
  Expect(MatchImageGapStride(5000U, 5000U, 128U) == 1U,
         "profile period must be consumed directly");
  Expect(MatchImageGapStride(15000U, 5000U, 128U) == 3U,
         "narrow profile must use its own fixed period");
}

void TestResidualTolerance()
{
  using CameraFrameSyncCore::MatchImageGapStride;
  Expect(MatchImageGapStride(31500U, 10000U, 128U) == 3U,
         "gap at the tolerance boundary must be accepted");
  Expect(MatchImageGapStride(32600U, 10000U, 128U) == 0U,
         "unexplained camera gap must be rejected");
  Expect(MatchImageGapStride(11500U, 5000U, 128U) == 2U,
         "minimum camera tolerance boundary must be accepted");
  Expect(MatchImageGapStride(11600U, 5000U, 128U) == 0U,
         "residual beyond the minimum tolerance must be rejected");
  Expect(MatchImageGapStride(0U, 10000U, 128U) == 0U,
         "duplicate camera timestamp must be rejected");
}

void TestStrideBound()
{
  using CameraFrameSyncCore::MatchImageGapStride;
  Expect(MatchImageGapStride(1280000U, 10000U, 128U) == 128U,
         "maximum supported drop stride must be accepted");
  Expect(MatchImageGapStride(1290000U, 10000U, 128U) == 0U,
         "gap beyond the retained trigger window must resynchronize");
  Expect(MatchImageGapStride(10000U, 0U, 128U) == 0U,
         "zero profile period must never match");
}

void TestTriggerSelectionUsesCameraGap()
{
  constexpr uint64_t period_us = 10000U;
  constexpr std::array<uint64_t, 5U> camera_timestamps{100U, 10100U, 30100U, 40100U,
                                                       70100U};
  constexpr std::array<uint32_t, 5U> expected_trigger_sequences{1U, 2U, 4U, 5U, 8U};

  uint64_t previous_camera_timestamp = camera_timestamps.front();
  uint32_t previous_trigger_sequence = expected_trigger_sequences.front();
  for (std::size_t index = 1U; index < camera_timestamps.size(); ++index)
  {
    const uint64_t gap = camera_timestamps[index] - previous_camera_timestamp;
    const uint32_t stride =
        CameraFrameSyncCore::MatchImageGapStride(gap, period_us, 128U);
    Expect(stride != 0U, "test camera gap must be explainable");
    const uint32_t selected_trigger = previous_trigger_sequence + stride;
    Expect(selected_trigger == expected_trigger_sequences[index],
           "camera gap must select the Nth real trigger event");
    previous_camera_timestamp = camera_timestamps[index];
    previous_trigger_sequence = selected_trigger;
  }
}

struct Sample
{
  uint64_t sensor_timestamp_us{};
  uint32_t value{};
};

void TestHistoryIsBoundedAndSearchable()
{
  CameraFrameSyncCore::SampleHistory<Sample, 3U> history;
  Expect(!history.PushBackDropOldest({1000U, 1U}),
         "first history insertion must not drop");
  Expect(!history.PushBackDropOldest({2000U, 2U}),
         "second history insertion must not drop");
  Expect(!history.PushBackDropOldest({3000U, 3U}),
         "third history insertion must not drop");
  Expect(history.PushBackDropOldest({4000U, 4U}),
         "full history must drop exactly its oldest sample");
  Expect(history.Size() == 3U && history.Front().sensor_timestamp_us == 2000U &&
             history.Back().sensor_timestamp_us == 4000U,
         "ring history must preserve logical order after wrap");

  const Sample* exact = CameraFrameSyncCore::FindBySensorTimestamp(history, 3000U, 0U);
  Expect(exact != nullptr && exact->value == 3U,
         "exact trigger timestamp must find the assembled IMU");
  const Sample* near = CameraFrameSyncCore::FindBySensorTimestamp(history, 3450U, 500U);
  Expect(near != nullptr && near->value == 3U,
         "offset lookup may select the nearest sample within tolerance");
  Expect(CameraFrameSyncCore::FindBySensorTimestamp(history, 1000U, 0U) == nullptr,
         "discarded IMU history must not match");
}
}  // namespace

int main()
{
  TestFixedPeriodStride();
  TestResidualTolerance();
  TestStrideBound();
  TestTriggerSelectionUsesCameraGap();
  TestHistoryIsBoundedAndSearchable();
  std::cout << "camera_frame_sync_sequence_test: PASS\n";
  return EXIT_SUCCESS;
}
