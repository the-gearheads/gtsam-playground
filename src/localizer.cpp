/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "localizer.h"

#include "TagModel.h"
#include "gtsam/nonlinear/Expression.h"

using namespace gtsam;
using symbol_shorthand::X;

constexpr int NUM_CORNERS = 4;

Localizer::Localizer() {
  ISAM2Params parameters;
  // parameters.relinearizeThreshold = 0.01;
  // parameters.relinearizeSkip = 1;
  // parameters.cacheLinearizedFactors = false;
  // parameters.enableDetailedResults = true;
  parameters.findUnusedFactorSlots = true;
  parameters.print();

  // TODO: make sure that timestamps in units of uS doesn't cause numerical
  // precision issues
  double lag = 5 * 1e6;
  // double lag = 2;
  smootherISAM2 = IncrementalFixedLagSmoother(lag, parameters);

  // // And make sure to call optimize first to get values
  // TODO i killed maybe needed, idk
  // Optimize();
}

void Localizer::Reset(Pose3 wTr, SharedNoiseModel noise, uint64_t timeUs) {
  // Anchor graph using initial pose. I subtract one to make sure that we dont
  // add this time to the estimate map twice
  timeUs -= 1;

  currStateIdx = X(timeUs);

  smootherISAM2 = IncrementalFixedLagSmoother(smootherISAM2.smootherLag(),
                                              smootherISAM2.params());

  graph.resize(0);
  currentEstimate.clear();
  newTimestamps.clear();
  factorsToRemove.clear();
  // twistsFromPreviousKey.clear();

  graph.addPrior(currStateIdx, wTr, noise);
  currentEstimate.insert(currStateIdx, wTr);
  newTimestamps[currStateIdx] = timeUs;

  wTb_latest = wTr;
}

void Localizer::AddOdometry(OdometryObservation odom) {
  const Pose3 &poseDelta = odom.poseDelta;
  const SharedNoiseModel &odometryNoise = odom.odometryNoise;
  uint64_t timeUs = odom.timeUs;

  Key newStateIdx = X(timeUs);

  // Add an odometry pose delta from our last state to our new one
  graph.emplace_shared<BetweenFactor<Pose3>>(currStateIdx, newStateIdx,
                                             poseDelta, odometryNoise);

  // And get initial guess just by composing previous pose
  wTb_latest = wTb_latest.transformPoseFrom(poseDelta);
  currentEstimate.insert(newStateIdx, wTb_latest);

  newTimestamps[newStateIdx] = timeUs;
  // twistsFromPreviousKey[newStateIdx] = poseDelta;
  latestOdomTime = timeUs;

  currStateIdx = newStateIdx;
}

using KeyTimeConstIt = FixedLagSmoother::KeyTimestampMap::const_iterator;
static KeyTimeConstIt FindCloser(KeyTimeConstIt left, KeyTimeConstIt right,
                                 double time) {
  double deltaLeft = time - left->second;
  double deltaRight = right->second - time;
  if (deltaLeft < deltaRight) {
    return left;
  } else {
    return right;
  }
}

// yeah this entire function was rewritten by gemini (in part) to fix the case where vision timestamp == odom timestamp. this seems to do the same thing reading it carefully (and appears to) and is more readable sooo
Key Localizer::GetOrInsertKey(Key newKey, double time) {
  using KeyTimeMap = FixedLagSmoother::KeyTimestampMap;
  const auto &isamTimestamps = smootherISAM2.timestamps();

  // --- 1. Pre-checks for immediate failure ---
  if (isamTimestamps.empty()) {
    fmt::println("GetOrInsertKey: Failed for time {}. Reason: iSAM history is empty.", time);
    return 0;
  }

  const auto &isamEntryAfter = isamTimestamps.upper_bound(newKey);
  if (isamEntryAfter == isamTimestamps.begin()) {
    fmt::println("GetOrInsertKey: Failed for time {}. Reason: Time is before oldest iSAM state ({}).", time, isamTimestamps.begin()->second);
    return 0;
  }
  const auto &isamEntryBefore = std::prev(isamEntryAfter);

  // --- 2. Search for the key in valid time regions ---

  // Case A: Time is bracketed by two states within the main iSAM graph.
  if (isamEntryAfter != isamTimestamps.end() && isamEntryBefore->second <= time) {
    return FindCloser(isamEntryBefore, isamEntryAfter, time)->first;
  }

  // Case B: Time is bracketed by two states in the `newTimestamps` map.
  if (!newTimestamps.empty()) {
    auto notAddedAfter = newTimestamps.upper_bound(newKey);
    if (notAddedAfter != newTimestamps.begin()) {
      auto notAddedBefore = std::prev(notAddedAfter);
      if (notAddedAfter != newTimestamps.end() && notAddedBefore->second <= time) {
        return FindCloser(notAddedBefore, notAddedAfter, time)->first;
      }
    }
  }

  // Case C: Time falls in the "gap" between the last iSAM state and the first new state.
  if (!newTimestamps.empty()) {
    if (time >= isamEntryBefore->second && time < newTimestamps.begin()->second) {
      return FindCloser(isamEntryBefore, newTimestamps.begin(), time)->first;
    }
  }

  // --- 3. If no key was found, diagnose and report the failure ---
  double latestKnownTime = newTimestamps.empty() ? isamEntryBefore->second : newTimestamps.rbegin()->second;
  if (time > latestKnownTime) {
    fmt::println("GetOrInsertKey: Failed for time {}. Reason: Time is after latest known state ({}).", time, latestKnownTime);
    return 0;
  }

  // If we reach here, it's a logic error. wtf is going on if this happens
  fmt::println("GetOrInsertKey: Unhandled case for time {}. sad", time);
  fmt::println("  - iSAM state before: {}", isamEntryBefore->second);
  if (isamEntryAfter != isamTimestamps.end()) {
    fmt::println("  - iSAM state after:  {}", isamEntryAfter->second);
  } else {
    fmt::println("  - iSAM state after:  (end of history)");
  }
  if (!newTimestamps.empty()) {
    fmt::println("  - New states range:  [{}, {}]", newTimestamps.begin()->second, newTimestamps.rbegin()->second);
  } else {
    fmt::println("  - New states:        (empty)");
  }
  throw std::logic_error("wtf");
}

bool Localizer::AddTagObservation(CameraVisionObservation obs) {
  const auto &isamTimestamps = smootherISAM2.timestamps();

  if (isamTimestamps.empty()) {
    // in practice, are timestamps that hit this point reusable in the future?
    // std::cerr << "No isam history yet - skipping" << std::endl;
    fmt::println("No isam history yet - skipping");
    return false;
  }

  if (obs.timeUs < isamTimestamps.begin()->second) {
    std::cerr << "Timestamp is before even isam history - skipping"
              << std::endl;
    return false;
  }

  int tagID = obs.tagID;
  const Cal3_S2_ &cameraCal = obs.cameraCal;
  const Pose3 &robotTcamera = obs.robotTcamera;
  const std::vector<Point2> &corners = obs.corners;
  const SharedNoiseModel cameraNoise = obs.cameraNoise;
  const uint64_t timeUs = obs.timeUs;

  auto worldPcorners_opt = TagModel::WorldToCorners(tagID);
  if (!worldPcorners_opt) {
    // todo return bad thing
    fmt::println("Could not find tag {} in our map!", tagID);
    return false;
  }
  auto worldPcorners = worldPcorners_opt.value();

  Key newKey = X(timeUs);

  // Find where we should attach our new factors to
  Key stateAtTime = GetOrInsertKey(newKey, timeUs);
  if (stateAtTime == 0) {
    fmt::println("Key not found - skipping");
    return false;
  }

  for (size_t i = 0; i < NUM_CORNERS; i++) {
    // corner in image space
    Point2 measurement = corners[i];

    // current world->body pose
    const Pose3_ worldTbody_fac(stateAtTime);
    const auto prediction = PredictLandmarkImageLocation(
        worldTbody_fac, robotTcamera, cameraCal, worldPcorners[i]);

    graph.addExpressionFactor(prediction, measurement, cameraNoise);
  }
  return true;
}

void Localizer::Optimize() {
  // fmt::println("Adding {} factors!", graph.size());
  // graph.print("New factors: ");
  // currentEstimate.print("New estimates: ");


  // {
  //   // Cull old vision measurements. Our times are ordered by key, which just so happens to be X(timestamp, us)
  //   auto MAX_AGE = 30 * 1'000'000;
  //   auto min_time = currStateIdx - MAX_AGE;
  //   auto min_time_it = keyToTimestamp.lower_bound(min_time);

  //   // Prepare to remove all our culled factors
  //   for (auto it = keyToTimestamp.begin(); *it < *min_time_it; it++) {
  //     factorsToRemove.push_back(it->first);
  //     fmt::println("removing {}, {}", it->first, it->second);
  //   }
  //   // And cull them from our map
  //   keyToTimestamp.erase(keyToTimestamp.begin(), min_time_it);
  // }

  smootherISAM2.update(graph, currentEstimate, newTimestamps, factorsToRemove);

  // reset the graph; isam wants to be fed factors to be -added-
  graph.resize(0);
  currentEstimate.clear();
  newTimestamps.clear();
  factorsToRemove.clear();

  // And grab the estimate of only the latest pose (maximize laziness)
  // Cache for use with FK prediction when adding odom factors
  wTb_latest = smootherISAM2.calculateEstimate<Pose3>(currStateIdx);
}

Matrix Localizer::GetLatestMarginals() const {
  return smootherISAM2.marginalCovariance(GetCurrStateIdx());
}

Vector6 Localizer::GetPoseComponentStdDevs() const {
  Matrix marginals = GetLatestMarginals();
  return marginals.diagonal().cwiseSqrt();
}

const std::vector<frc::Pose3d> Localizer::GetPoseHistory() const {
  // Plot all history, so grab the whole estimate
  Values result = smootherISAM2.calculateEstimate();

  // 5 seconds of history
  auto start = currStateIdx - (5 * 1e6);

  std::vector<frc::Pose3d> ret;
  ret.reserve(1000);

  for (const Values::ConstKeyValuePair &estPair : result) {
    if (estPair.key < start)
      continue;

    Pose3 est = estPair.value.cast<Pose3>();

    // auto rot = est.rotation().toQuaternion();
    // vector<double> poseEst{est.x(), est.y(), est.z(), rot.w(),
    //                             rot.x(), rot.y(), rot.z()};

    ret.emplace_back(frc::Translation3d{units::meter_t{est.x()},
                                        units::meter_t{est.y()},
                                        units::meter_t{est.z()}},
                     frc::Rotation3d{est.rotation().matrix()});
  }
  // fmt::println("i={}", i);

  return ret;
}
