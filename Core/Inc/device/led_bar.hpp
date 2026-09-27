#pragma once
#include <cstdint>

enum class LedBarDotMode {
  normal,
  dot16,
  dot8,
  dot4,
  dot2,
};

enum class LedBarValMode {
  pmbit8,
  pmlinear8,
  pbit16,
  plinear16
};

class LedBar16 {
public:
  void set(const uint16_t pattern);
  void set(const uint16_t val, const LedBarDotMode mode);
  void set(const float val, const LedBarValMode mode, const float maxabs);
  void setBit(uint8_t index, bool on);
  void clear();
  void update();

private:
  void apply();
  uint16_t pattern_ = 0;
  bool dirty_ = true;
};
