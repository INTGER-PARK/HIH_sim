#pragma once

// CoP feedback names, stable published IDs and dense sample indices.
// MOB=0 reads minitrone/cop_hat; SOLVER=2 reads minitrone/cop_solver_real.
// Topics are relative to the role namespace. TOUCH is not selectable.
// The four CoP nodes validate sample freshness and reset input filters on source
// changes while retaining their accumulated offsets. Use cop_source to select.

#include <array>
#include <cstddef>
#include <string>

namespace hih_controller::cop_feedback
{
// Keep the published source IDs stable; source ID 1 is no longer selectable.
// Use dense sample indices rather than indexing arrays with the published ID.
// Published IDs stay MOB=0 and SOLVER=2; ID 2 is not an array index.
enum class CopSource : int {MOB = 0, SOLVER = 2};
// The two sample slots follow the same order as source and topic arrays.
inline constexpr std::size_t kCopSourceCount = 2;
inline constexpr std::array<CopSource, kCopSourceCount> kCopSources{
  CopSource::MOB, CopSource::SOLVER};
inline constexpr std::array<const char *, kCopSourceCount> kCopTopics{
  "minitrone/cop_hat", "minitrone/cop_solver_real"};

constexpr std::size_t sourceIndex(CopSource source)
{
  // Map the published IDs MOB(0)->0 and SOLVER(2)->1.
  // The caller must pass a supported enum value.
  return source == CopSource::SOLVER ? 1U : 0U;
}

constexpr CopSource nextSource(CopSource source)
{
  // Keyboard cycle: MOB -> SOLVER -> MOB, excluding TOUCH.
  return source == CopSource::MOB ? CopSource::SOLVER : CopSource::MOB;
}

constexpr const char * sourceName(CopSource source)
{
  // Uppercase terminal label, distinct from the lowercase parameter value.
  return source == CopSource::SOLVER ? "SOLVER" : "MOB";
}

constexpr const char * sourceParameterValue(CopSource source)
{
  // Canonical cop_source parameter spelling: "mob" or "solver".
  return source == CopSource::SOLVER ? "solver" : "mob";
}

inline bool parseSource(const std::string & value, CopSource & source)
{
  // Update source on successful parsing; leave it unchanged on failure.
  // Accept only the spellings below, without case/whitespace normalization.
  // gt/GT are compatibility aliases for SOLVER, not another feedback source.
  if (value == "mob" || value == "MOB") {
    source = CopSource::MOB;
    return true;
  }
  if (value == "solver" || value == "SOLVER" || value == "gt" || value == "GT") {
    source = CopSource::SOLVER;
    return true;
  }
  return false;
}
}  // namespace hih_controller::cop_feedback
