// viz/Visualizer — thin wrapper over a rerun.io recording (Task 1.8, spec §6).
//
// Built with WITH_RERUN=OFF (the default), EVERY method is a no-op and the rerun SDK is not
// linked, so call sites compile and run unchanged. PImpl keeps all rerun types out of this
// header, so consumers never need the rerun headers or the WITH_RERUN define.
#pragma once

#include <cstdint>
#include <memory>
#include <string>

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

  // Entity paths are rerun's hierarchical names, e.g. "world/ur5". All are no-ops when disabled.
  void log_pose(const std::string &entity, const Transform &tf);
  void log_mesh(const std::string &entity, const Mesh &mesh, const Transform &tf = Transform{});
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
