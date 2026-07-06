// viz/Visualizer — thin wrapper over a rerun.io recording (Task 1.8, spec §6).
//
// Built with WITH_RERUN=OFF (the default), EVERY method is a no-op and the rerun SDK is not
// linked, so call sites compile and run unchanged. PImpl keeps all rerun types out of this
// header, so consumers never need the rerun headers or the WITH_RERUN define.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "quevedomp/core/types.hpp"
#include "quevedomp/robot/robot_model.hpp"

namespace quevedomp {

class Visualizer {
public:
  explicit Visualizer(const std::string &app_id = "quevedomp");
  ~Visualizer();
  Visualizer(Visualizer &&) noexcept;
  Visualizer &operator=(Visualizer &&) noexcept;
  Visualizer(const Visualizer &) = delete;
  Visualizer &operator=(const Visualizer &) = delete;

  // True only in a WITH_RERUN=ON build.
  bool enabled() const noexcept;

  // Pick a sink (call at most one). save() writes an .rrd file for offline viewing; spawn()
  // launches the native viewer and streams to it.
  void save(const std::string &path);
  void spawn();

  // Animation cursor: subsequent logs are stamped at frame `index` on a "frame" timeline.
  void set_frame(int64_t index);

  // RGB in [0,255]; used to tint meshes/points/segments (e.g. red for a colliding config).
  using Color = std::array<std::uint8_t, 3>;

  // Entity paths are rerun's hierarchical names, e.g. "world/ur5". All are no-ops when disabled.
  void log_pose(const std::string &entity, const Transform &tf);
  // `color`, if given, tints the whole mesh (useful to flag a colliding link red). Set `is_static`
  // for scene elements that never move (e.g. the environment): they are logged on rerun's static
  // timeline so they show at every frame, not just the one that happened to be active when logged.
  void log_mesh(const std::string &entity, const Mesh &mesh, const Transform &tf = Transform{},
                const std::optional<Color> &color = std::nullopt, bool is_static = false);
  // Points at world positions, one shared `color` and `radius` (m). A sphere obstacle renders as a
  // single point with radius = its radius; collision witness points use this too.
  void log_points(const std::string &entity, const std::vector<Eigen::Vector3d> &points,
                  const Color &color, float radius = 0.01f);
  // Line segments (each an endpoint pair) in one `color` — skeletons, witness pairs, normals.
  void log_segments(const std::string &entity,
                    const std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &segments,
                    const Color &color);
  // Per-link coordinate frames + a skeleton (line segments between connected link origins) for
  // the robot at configuration q.
  void log_robot(const std::string &entity, const RobotModel &model, const JointPosition &q);
  // The 3D path traced by `tip` across the trajectory's waypoint configurations (a line strip).
  void log_trajectory(const std::string &entity, const RobotModel &model, const Trajectory &traj,
                      const std::string &tip);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace quevedomp
