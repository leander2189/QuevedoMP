// Task 2a.5 verify — RobotModel / RobotInstance / SceneDescription serializers round-trip
// (serialize -> deserialize -> compare equality), and reject corrupt blobs (spec §5.3).
#include <gtest/gtest.h>

#include <string>

#include <Eigen/Core>

#include "quevedomp/capture/serialize.hpp"
#include "quevedomp/collision/geometry.hpp"
#include "quevedomp/robot/robot_instance.hpp"
#include "quevedomp/robot/robot_model.hpp"

using namespace quevedomp;
using namespace quevedomp::capture;
using namespace quevedomp::collision;

namespace {

const char *kArm = R"(<robot name="arm">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="link1"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-1.5" upper="1.5" effort="10" velocity="2"/>
  </joint>
  <joint name="j2" type="prismatic">
    <parent link="link1"/><child link="link2"/>
    <origin xyz="0.2 0 0"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="0.5" effort="5" velocity="1"/>
  </joint>
</robot>)";

const char *kYaml = R"(joint_limits:
  j1: { max_acceleration: 3.0, max_jerk: 30.0 }
)";

Transform at(double x, double y, double z) {
  return Transform::from_translation(Eigen::Vector3d(x, y, z));
}

Mesh tetra() {
  Mesh m;
  m.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  m.triangles = {{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
  return m;
}

} // namespace

TEST(CaptureRobotModel, RoundTripStructure) {
  const auto model = RobotModel::from_urdf(kArm);
  const auto back = deserialize_robot_model(serialize_robot_model(*model));

  EXPECT_EQ(back->name(), model->name());
  EXPECT_EQ(back->root_link(), model->root_link());
  EXPECT_EQ(back->num_links(), model->num_links());
  EXPECT_EQ(back->num_joints(), model->num_joints());
  EXPECT_EQ(back->dof(), model->dof());
  EXPECT_EQ(back->source_urdf(), model->source_urdf());

  // Re-serializing the round-tripped model reproduces the exact bytes.
  EXPECT_EQ(serialize_robot_model(*back), serialize_robot_model(*model));

  // Joint kinematics survive (checked via a representative joint).
  const Joint *j1 = back->find_joint("j1");
  ASSERT_NE(j1, nullptr);
  EXPECT_EQ(j1->type, JointType::Revolute);
  EXPECT_TRUE(j1->origin.is_approx(model->find_joint("j1")->origin));
}

TEST(CaptureRobotModel, RoundTripYamlExtension) {
  const auto model = RobotModel::from_urdf(kArm, std::string(kYaml));
  const auto back = deserialize_robot_model(serialize_robot_model(*model));

  ASSERT_TRUE(back->source_yaml().has_value());
  EXPECT_EQ(*back->source_yaml(), *model->source_yaml());
  // The YAML-supplied acceleration limit is present after the round trip.
  EXPECT_DOUBLE_EQ(back->find_joint("j1")->limits.acceleration, 3.0);
}

TEST(CaptureRobotModel, RejectsBadMagicAndTruncation) {
  const std::string blob = serialize_robot_model(*RobotModel::from_urdf(kArm));
  EXPECT_THROW(deserialize_robot_model("not a blob"), std::runtime_error);
  EXPECT_THROW(deserialize_robot_model(blob.substr(0, blob.size() - 4)), std::runtime_error);
}

TEST(CaptureRobotInstance, RoundTripModelAndAcm) {
  const auto model = RobotModel::from_urdf(kArm);
  RobotInstance robot(model);
  robot.acm().allow("base", "link1");
  robot.acm().allow("link2", "link1"); // stored normalized as (link1, link2)

  const auto back = deserialize_robot_instance(serialize_robot_instance(robot));

  EXPECT_EQ(back.model().name(), model->name());
  EXPECT_EQ(back.model().num_joints(), model->num_joints());
  EXPECT_EQ(back.acm().pairs(), robot.acm().pairs());
  EXPECT_TRUE(back.acm().is_allowed("link1", "base"));
  EXPECT_TRUE(back.acm().is_allowed("link1", "link2"));
  EXPECT_FALSE(back.acm().is_allowed("base", "link2"));
}

TEST(CaptureScene, RoundTripAllGeometryKinds) {
  SceneDescription scene;
  scene.objects.push_back({"box", BoxShape{Eigen::Vector3d(0.1, 0.2, 0.3)}, at(1, 0, 0)});
  scene.objects.push_back({"ball", SphereShape{0.25}, at(0, 2, 0)});
  scene.objects.push_back({"can", CylinderShape{0.05, 0.4}, at(0, 0, 3)});
  scene.objects.push_back({"mesh", tetra(), at(-1, -1, -1)});

  const auto back = deserialize_scene(serialize_scene(scene));
  ASSERT_EQ(back.objects.size(), scene.objects.size());

  for (std::size_t i = 0; i < scene.objects.size(); ++i) {
    const SceneObject &a = scene.objects[i];
    const SceneObject &b = back.objects[i];
    EXPECT_EQ(b.id, a.id);
    EXPECT_TRUE(b.pose.is_approx(a.pose)) << "object " << a.id;
    ASSERT_EQ(b.geometry.index(), a.geometry.index()) << "object " << a.id;
  }

  EXPECT_TRUE(std::get<BoxShape>(back.objects[0].geometry)
                  .half_extents.isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
  EXPECT_DOUBLE_EQ(std::get<SphereShape>(back.objects[1].geometry).radius, 0.25);
  EXPECT_DOUBLE_EQ(std::get<CylinderShape>(back.objects[2].geometry).length, 0.4);

  const Mesh &m = std::get<Mesh>(back.objects[3].geometry);
  EXPECT_EQ(m.vertices.size(), 4u);
  EXPECT_EQ(m.triangles.size(), 4u);
  EXPECT_TRUE(m.vertices[1].isApprox(Eigen::Vector3d(1, 0, 0)));
  EXPECT_EQ(m.triangles[3], Eigen::Vector3i(1, 2, 3));

  // Re-serialize reproduces the exact bytes.
  EXPECT_EQ(serialize_scene(back), serialize_scene(scene));
}

TEST(CaptureScene, RejectsBadMagic) {
  EXPECT_THROW(deserialize_scene("garbage"), std::runtime_error);
}
