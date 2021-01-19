#include <Arduino.h>
#include <etl/circular_buffer.h>
#include <etl/map.h>
#include <etl/vector.h>

class Scales {
 public:
  static const size_t kMemoizedMeashrementsCount = 24;
  static const size_t kMaxScalesCount = 10;
  typedef etl::reference_wrapper<Scales> ScalesWrapper;
  typedef etl::vector<ScalesWrapper, kMaxScalesCount> ScalesList;
  typedef etl::map<uint8_t, ScalesList, kMaxScalesCount> ScalesMap;
  etl::circular_buffer<int32_t, kMemoizedMeashrementsCount> received_values_;
  Scales(uint8_t clock_pin, uint8_t data_pin, double scale = 1u, uint8_t gain = 128u, uint8_t tare = 0u);
  void SetGain(uint8_t gain);
  void SetTareWithAvg();
  bool IsReady();
  int32_t GetLastValue();
  double GetLastUnits();
  int32_t GetAvgValue();
  double GetAvgUnits();
  static void InitPins();
  static bool IsAllReady();
  static void SetTareForAll();
  static void ReadAll();

 private:
  uint8_t clock_pin_;  // Power Down and Serial Clock Input Pin
  uint8_t gain_;      // amplification factor
  uint32_t tare_;         // offset used for tare weight
  double scale_;       // used to return weight in grams, kg, ounces, whatever
  uint8_t data_pin_;   // Serial Data Output Pin
  void ReceiveByte();
  int32_t value_buffer_ = 0;
  int32_t last_value_ = 0;
  uint8_t received_bit_count_ = 0;
  void PullBit();
  static ScalesMap scales_by_clock_pin_;
  static void RegisterScales(Scales& scale);
  static void BlockInterrupts();
  static void UnblockInterrupts();
  static void PullData(uint8_t clock_pin, ScalesList& scalesOnClock);
  static void PushGain(uint8_t gain, uint8_t clock_pin);
  static uint32_t Extend24To32(uint32_t var);
};
