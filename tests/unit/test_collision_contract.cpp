// Task 2a.1 verify — the collision interface (spec §4.2) compiles, is implementable against a
// stub backend, the base-class query() delegates to query_batch, and the value types behave.
// No CUDA types appear anywhere (this builds in the no-GPU dev-cpu preset).
#include <gtest/gtest.h>

#include <memory>
#include <span>

#include <Eigen/Core>

#include "quevedomp/collision/collision_scene.hpp"
#include "quevedomp/collision/edge_check.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/collision/types.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::collision;

namespace {

// Minimal backend: reports every config as colliding. Proves the pure-virtual contract is
// implementable and lets us exercise the base-class query() convenience.
class StubScene : public CollisionScene {
public:
  SceneHandle add_object(std::string, const Geometry &, const Transform &) override {
    return next_++;
  }
  void remove_object(SceneHandle) override {}
  void move_object(SceneHandle, const Transform &) override {}
  std::unique_ptr<Workspace> make_workspace() const override {
    return std::make_unique<StubWorkspace>();
  }
  BatchResult query_batch(const RobotInstance &, std::span<const JointPosition> qs,
                          const QueryOptions &opts, Workspace &) const override {
    BatchResult b;
    b.in_collision.assign(qs.size(), 1);
    if (opts.distance)
      b.min_distance.assign(qs.size(), -0.5f);
    return b;
  }

private:
  struct StubWorkspace : Workspace {};
  SceneHandle next_ = 0;
};

} // namespace

TEST(CollisionContract, QueryDelegatesToQueryBatch) {
  const auto model = RobotModel::from_urdf("<robot name=\"r\"><link name=\"base\"/></robot>");
  const RobotInstance robot(model);

  StubScene scene;
  const auto ws = scene.make_workspace();
  QueryOptions opts;
  opts.distance = true;

  const CollisionResult r = scene.query(robot, JointPosition::Zero(0), opts, *ws);
  EXPECT_TRUE(r.in_collision);
  EXPECT_FLOAT_EQ(r.min_distance, -0.5f);
}

TEST(CollisionContract, GeometryAndSceneDescription) {
  SceneDescription env;
  env.objects.push_back({"box", BoxShape{Eigen::Vector3d(0.1, 0.2, 0.3)}, Transform::Identity()});
  env.objects.push_back({"ball", SphereShape{0.25}, Transform::Identity()});
  env.objects.push_back({"can", CylinderShape{0.05, 0.2}, Transform::Identity()});
  env.objects.push_back({"mesh", Mesh{}, Transform::Identity()});
  ASSERT_EQ(env.objects.size(), 4u);
  EXPECT_TRUE(std::holds_alternative<BoxShape>(env.objects[0].geometry));
  EXPECT_TRUE(std::holds_alternative<Mesh>(env.objects[3].geometry));
}

TEST(CollisionContract, PaddingMapIsOrderIndependent) {
  PaddingMap pm;
  EXPECT_TRUE(pm.empty());
  pm.set("a", "b", 0.01f);
  EXPECT_FLOAT_EQ(pm.get("a", "b"), 0.01f);
  EXPECT_FLOAT_EQ(pm.get("b", "a"), 0.01f);        // order-independent
  EXPECT_FLOAT_EQ(pm.get("x", "y", 0.05f), 0.05f); // fallback
}

TEST(CollisionContract, AllowedCollisionMatrix) {
  AllowedCollisionMatrix acm;
  acm.allow("link1", "link2");
  EXPECT_TRUE(acm.is_allowed("link2", "link1")); // order-independent
  EXPECT_FALSE(acm.is_allowed("link1", "link3"));
  acm.disallow("link1", "link2");
  EXPECT_FALSE(acm.is_allowed("link1", "link2"));
}

TEST(CollisionContract, QueryOptionsDefaults) {
  const QueryOptions o;
  EXPECT_FALSE(o.distance);
  EXPECT_TRUE(o.check_self_collision);
  EXPECT_FLOAT_EQ(o.max_distance, 0.10f);
  EXPECT_FALSE(o.per_pair_padding.has_value());
}
