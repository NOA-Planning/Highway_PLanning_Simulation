#pragma once

#include "common.h"
#include "config.h"
#include "environment.h"
#include "localization.h"
#include "reference_line.h"
namespace ahrs {
class Visualization {
  void ShowResult(const ReferenceLine& line, const Environment& env,
                  const Localization& loc, const Curve& trajectory,
                  const Config& config)
};
}  // namespace ahrs