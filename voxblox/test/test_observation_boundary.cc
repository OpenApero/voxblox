#include <algorithm>

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "voxblox/core/common.h"
#include "voxblox/core/layer.h"
#include "voxblox/core/voxel.h"
#include "voxblox/integrator/integrator_utils.h"
#include "voxblox/integrator/tsdf_integrator.h"

using namespace voxblox;  // NOLINT

constexpr FloatingPoint kMaxRayLength = 5.0f;
constexpr FloatingPoint kTruncationDistance = 0.2f;
constexpr FloatingPoint kVoxelSize = 0.05f;

// A 42 x 54 deg depth camera: the image corner sits 32.6 deg off axis, so a
// surface on the z = 5.0 observation plane has a norm of about 5.9.
constexpr FloatingPoint kCornerSurfaceZ = 4.9f;
constexpr FloatingPoint kCornerOffAxis = 3.13f;  // norm 5.82
constexpr FloatingPoint kSentinelZ = 5.2f;
/// The desensitized sentinel: far beyond the plane, so only its class matters.
constexpr FloatingPoint kFarSentinelZ = 10.0f;

/// A point on the image-corner ray (32.6 deg off axis) at the given depth.
Point cornerRayAt(FloatingPoint z) {
  return Point(kCornerOffAxis * z / kCornerSurfaceZ, 0.0f, z);
}

/// Exposes the protected range gate and clearing-ray cap for testing.
class GateProbe : public SimpleTsdfIntegrator {
 public:
  GateProbe(const Config& config, Layer<TsdfVoxel>* layer)
      : SimpleTsdfIntegrator(config, layer) {}

  using TsdfIntegratorBase::clearingRayLengthLimit;
  using TsdfIntegratorBase::isPointValid;
};

/// Distance from the origin to the farthest voxel a clearing ray actually
/// carves, i.e. what RayCaster makes of the cap.
FloatingPoint carvedRayLength(const Point& point_C, FloatingPoint cap) {
  const Point origin = Point::Zero();
  RayCaster ray_caster(origin, point_C, /*is_clearing_ray=*/true,
                       /*voxel_carving_enabled=*/true, cap, 1.0f / kVoxelSize,
                       kTruncationDistance);

  GlobalIndex ray_index;
  FloatingPoint farthest = 0.0f;
  while (ray_caster.nextRayIndex(&ray_index)) {
    farthest = std::max(
        farthest,
        (getCenterPointFromGridIndex(ray_index, kVoxelSize) - origin).norm());
  }
  return farthest;
}

class ObservationBoundaryTest : public ::testing::Test {
 public:
  ObservationBoundaryTest() : layer_(kVoxelSize, 16) {}

  TsdfIntegratorBase::Config makeConfig(ObservationBoundary boundary) const {
    TsdfIntegratorBase::Config config;
    config.min_ray_length_m = 0.1;
    config.max_ray_length_m = kMaxRayLength;
    config.default_truncation_distance = kTruncationDistance;
    config.allow_clear = true;
    config.observation_boundary = boundary;
    return config;
  }

 protected:
  Layer<TsdfVoxel> layer_;
};

TEST_F(ObservationBoundaryTest, PlanarKeepsCornerSurface) {
  GateProbe probe(makeConfig(ObservationBoundary::kPlanar), &layer_);

  const Point corner(kCornerOffAxis, 0.0f, kCornerSurfaceZ);
  ASSERT_GT(corner.norm(), 5.0f);

  bool is_clearing = true;
  EXPECT_TRUE(probe.isPointValid(corner, false, &is_clearing));
  EXPECT_FALSE(is_clearing);
}

TEST_F(ObservationBoundaryTest, PlanarClearsSentinel) {
  GateProbe probe(makeConfig(ObservationBoundary::kPlanar), &layer_);

  bool is_clearing = false;
  EXPECT_TRUE(
      probe.isPointValid(Point(0.0f, 0.0f, kSentinelZ), false, &is_clearing));
  EXPECT_TRUE(is_clearing);

  is_clearing = false;
  const Point corner_sentinel(kCornerOffAxis, 0.0f, kSentinelZ);
  EXPECT_TRUE(probe.isPointValid(corner_sentinel, false, &is_clearing));
  EXPECT_TRUE(is_clearing);
}

TEST_F(ObservationBoundaryTest, PlanarTreatsPointsBehindPlaneAsSurface) {
  GateProbe probe(makeConfig(ObservationBoundary::kPlanar), &layer_);

  // Synthetic clouds (e.g. an imagined ground disc) may sit behind the camera
  // plane; they stay ordinary surface points rather than clearing rays.
  bool is_clearing = true;
  EXPECT_TRUE(
      probe.isPointValid(Point(1.0f, 0.0f, -0.5f), false, &is_clearing));
  EXPECT_FALSE(is_clearing);

  // The min bound still applies, on the norm.
  EXPECT_FALSE(
      probe.isPointValid(Point(0.0f, 0.0f, -0.05f), false, &is_clearing));
}

TEST_F(ObservationBoundaryTest, SphericalIsUnchanged) {
  GateProbe probe(makeConfig(ObservationBoundary::kSpherical), &layer_);

  // The same corner surface is a clearing ray under the spherical boundary --
  // this is exactly what the planar boundary exists to fix.
  bool is_clearing = false;
  const Point corner(kCornerOffAxis, 0.0f, kCornerSurfaceZ);
  EXPECT_TRUE(probe.isPointValid(corner, false, &is_clearing));
  EXPECT_TRUE(is_clearing);

  is_clearing = true;
  EXPECT_TRUE(probe.isPointValid(Point(0.0f, 0.0f, 3.0f), false, &is_clearing));
  EXPECT_FALSE(is_clearing);

  EXPECT_FALSE(
      probe.isPointValid(Point(0.0f, 0.0f, 0.05f), false, &is_clearing));

  is_clearing = false;
  EXPECT_TRUE(
      probe.isPointValid(Point(0.0f, 0.0f, kSentinelZ), false, &is_clearing));
  EXPECT_TRUE(is_clearing);
}

TEST_F(ObservationBoundaryTest, SphericalClearingRayLengthLimitIsUnchanged) {
  GateProbe probe(makeConfig(ObservationBoundary::kSpherical), &layer_);

  // The cap is the configured max range whatever the ray, as before.
  EXPECT_FLOAT_EQ(probe.clearingRayLengthLimit(Point(0.0f, 0.0f, kSentinelZ)),
                  kMaxRayLength);
  EXPECT_FLOAT_EQ(probe.clearingRayLengthLimit(cornerRayAt(kFarSentinelZ)),
                  kMaxRayLength);
  EXPECT_FLOAT_EQ(probe.clearingRayLengthLimit(Point(0.0f, 0.0f, -1.0f)),
                  kMaxRayLength);
}

TEST_F(ObservationBoundaryTest, PlanarClampsFarSentinelToThePlane) {
  GateProbe probe(makeConfig(ObservationBoundary::kPlanar), &layer_);

  // An off-axis sentinel far beyond the plane: the carve must stop where the
  // ray crosses z = max_ray_length_m, not near the sentinel itself.
  const Point sentinel = cornerRayAt(kFarSentinelZ);
  const FloatingPoint plane_intersection =
      kMaxRayLength * sentinel.norm() / sentinel.z();
  const FloatingPoint cap = probe.clearingRayLengthLimit(sentinel);
  EXPECT_FLOAT_EQ(cap, plane_intersection);

  // The unclamped carve would have run nearly twice as far.
  const FloatingPoint unclamped = sentinel.norm() - kTruncationDistance;
  ASSERT_GT(unclamped, cap + 1.0f);

  const FloatingPoint carved = carvedRayLength(sentinel, cap);
  EXPECT_NEAR(carved, cap, 2.0f * kVoxelSize);
  EXPECT_LT(carved, unclamped - 1.0f);

  // Defensive branch: a malformed point never scales the cap.
  EXPECT_FLOAT_EQ(probe.clearingRayLengthLimit(Point(1.0f, 0.0f, 0.0f)),
                  kMaxRayLength);
}

TEST_F(ObservationBoundaryTest, PlanarSentinelAtPlanePlusTruncationIsAsBefore) {
  GateProbe probe(makeConfig(ObservationBoundary::kPlanar), &layer_);

  // Regression against the pre-clamp design, whose carve was norm - truncation.
  // On axis the two agree exactly, so RayCaster's min changes nothing.
  const Point on_axis(0.0f, 0.0f, kMaxRayLength + kTruncationDistance);
  EXPECT_FLOAT_EQ(probe.clearingRayLengthLimit(on_axis),
                  on_axis.norm() - kTruncationDistance);

  // Off axis the plane cap wins, and is smaller by exactly trunc * (1/cos - 1).
  const Point off_axis = cornerRayAt(kMaxRayLength + kTruncationDistance);
  const FloatingPoint inverse_cos = off_axis.norm() / off_axis.z();
  const FloatingPoint unclamped = off_axis.norm() - kTruncationDistance;
  EXPECT_NEAR(unclamped - probe.clearingRayLengthLimit(off_axis),
              kTruncationDistance * (inverse_cos - 1.0f), 1e-5f);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();

  return result;
}
