#include <gtest/gtest.h>
#include "collision_checker/collision_checker.hpp"
#include <polygon/box2d.hpp>
#include <derived_object_msgs/Object.h>
#include <tf/transform_datatypes.h>

TEST(Box2dTest, box2d_collision) {
  const double len = 5.0;
  const double width = 3.0;
  const double theta1 = M_PI;
  const double theta2 = /*0.25 * */ M_PI;
  Eigen::Vector2d center1{0.0, 0.0};
  Eigen::Vector2d center2{5.2, 0.0};
  common::Box2d box1 = common::Box2d(center1, theta1, len, width);
  common::Box2d box2 = common::Box2d(center2, theta2, len, width);
  EXPECT_FALSE(box1.HasOverlapWithBox2d(box2));
}

class CollisionCheckTest : public ::testing::Test {
 public:
  planning::ReferenceLine reference_line_;
  std::shared_ptr<planning::Obstacle> obstacle_;
  derived_object_msgs::Object object_;
  double start_s_{2.0};
  double end_s_{100.0};
  double t_start_{0.0};
  double t_end_{8.0};
  std::array<double, 3> init_d_{0, 0, 0};
  double lookahead_time_{8.0};
  double delta_t_{0.1};
  vehicle_state::VehicleParams vehicle_params_;
  std::shared_ptr<planning::STGraph> st_graph_;
  std::shared_ptr<planning::CollisionChecker> collision_checker_;
 protected:
  void SetUp() override {
    // reference line
    SetUpReferenceLine();
    // setup vehicle params
    vehicle_params_.width = 3.0;
    vehicle_params_.length = 5.0;
    vehicle_params_.back_axle_to_center_length = 1.5;
    // setup obstacle
    SetUpObject();
    obstacle_ = std::make_shared<planning::Obstacle>(object_);
    obstacle_->PredictTrajectory(8.0, 0.1);
    // setup st graph
    SetUpSTGraph();
    std::unordered_map<int, std::shared_ptr<planning::Obstacle>> obstacle_map;
    obstacle_map.emplace(obstacle_->Id(), obstacle_);
    collision_checker_ = std::make_shared<planning::CollisionChecker>(obstacle_map,
                                                                      reference_line_,
                                                                      st_graph_,
                                                                      start_s_,
                                                                      init_d_[0],
                                                                      2.0,
                                                                      0.3,);

  }

  void TearDown() override {}
  void SetUpObject() {
    object_.object_classified = derived_object_msgs::Object::OBJECT_DETECTED;
    object_.classification = derived_object_msgs::Object::CLASSIFICATION_CAR;
    object_.shape.type = shape_msgs::SolidPrimitive::BOX;
    object_.shape.dimensions[0] = 5.0;
    object_.shape.dimensions[1] = 3.0;
    object_.shape.dimensions[2] = 1.5;
    object_.pose.position.x = 105.789;
    object_.pose.position.y = -193.267;
    object_.pose.position.z = 2.0;
    object_.id = 1;
    object_.twist.linear.x = object_.twist.linear.y = object_.twist.linear.z = 0.0;
    object_.twist.angular.x = object_.twist.angular.y = object_.twist.angular.z = 0.0;
    object_.pose.orientation.x = object_.pose.orientation.y = object_.pose.orientation.z = 0.0;
    object_.pose.orientation.w = 1.0;
    object_.accel.linear.x = object_.accel.linear.y = object_.accel.linear.z = object_.accel.angular.x =
    object_.accel.angular.y = object_.accel.angular.z = 0.0;
  }

  void SetUpSTGraph() {
    std::vector<std::shared_ptr<planning::Obstacle>> obstacles{obstacle_};
    st_graph_ = std::make_shared<planning::STGraph>(obstacles,
                                                    reference_line_,
                                                    start_s_,
                                                    end_s_,
                                                    t_start_,
                                                    t_end_,
                                                    init_d_,
                                                    8.0,
                                                    0.0);
  }

  void SetUpReferenceLine() {
    Eigen::MatrixXd poses(190, 3);
    poses << 127.413, -196.713, -3.1391,
        122.789, -193.225, -3.1391,
        121.789, -193.227, -3.1391,
        120.789, -193.23, -3.1391,
        119.789, -193.232, -3.1391,
        118.789, -193.235, -3.1391,
        117.789, -193.237, -3.1391,
        116.789, -193.24, -3.1391,
        115.789, -193.242, -3.1391,
        114.789, -193.245, -3.1391,
        113.789, -193.247, -3.1391,
        112.789, -193.25, -3.1391,
        111.789, -193.252, -3.1391,
        110.789, -193.255, -3.1391,
        109.789, -193.257, -3.1391,
        108.789, -193.26, -3.1391,
        107.789, -193.262, -3.1391,
        106.789, -193.265, -3.1391,
        105.789, -193.267, -3.1391,
        104.789, -193.27, -3.1391,
        103.789, -193.272, -3.1391,
        102.789, -193.275, -3.1391,
        101.789, -193.277, -3.1391,
        100.789, -193.28, -3.1391,
        99.7886, -193.282, -3.1391,
        98.7886, -193.285, -3.1391,
        97.7886, -193.287, -3.1391,
        96.7886, -193.29, -3.1391,
        95.7886, -193.292, -3.1391,
        94.7886, -193.295, -3.1391,
        93.7887, -193.297, -3.1391,
        92.7887, -193.3, -3.1391,
        91.7887, -193.302, -3.1391,
        90.7887, -193.305, -3.1391,
        89.7887, -193.307, -3.1391,
        88.7887, -193.31, -3.1391,
        87.7887, -193.312, -3.1391,
        86.7887, -193.314, -3.1391,
        85.7887, -193.317, -3.1391,
        84.7887, -193.319, -3.1391,
        83.7887, -193.322, -3.1391,
        82.7887, -193.324, -3.1391,
        81.7887, -193.327, -3.1391,
        80.7887, -193.329, -3.1391,
        79.7887, -193.332, -3.1391,
        78.7887, -193.334, -3.1391,
        77.7887, -193.337, -3.1391,
        76.7887, -193.339, -3.1391,
        75.7887, -193.342, -3.1391,
        74.7887, -193.344, -3.1391,
        73.7887, -193.347, -3.1391,
        72.7887, -193.349, -3.1391,
        71.7887, -193.352, -3.1391,
        70.7887, -193.354, -3.1391,
        69.7887, -193.357, -3.1391,
        68.7887, -193.359, -3.1391,
        67.7887, -193.362, -3.1391,
        66.7887, -193.364, -3.1391,
        65.7887, -193.367, -3.1391,
        64.7887, -193.369, -3.1391,
        63.7887, -193.372, -3.1391,
        62.7887, -193.374, -3.1391,
        61.7888, -193.377, -3.1391,
        60.7888, -193.379, -3.1391,
        59.7888, -193.382, -3.1391,
        58.7888, -193.384, -3.1391,
        57.7888, -193.387, -3.1391,
        56.7888, -193.389, -3.1391,
        55.7888, -193.392, -3.1391,
        54.7888, -193.394, -3.1391,
        53.7888, -193.397, -3.1391,
        52.7888, -193.399, -3.1391,
        51.7888, -193.402, -3.1391,
        50.7888, -193.404, -3.1391,
        49.7888, -193.407, -3.1391,
        48.7888, -193.409, -3.1391,
        47.7888, -193.412, -3.1391,
        46.7888, -193.414, -3.1391,
        45.7888, -193.417, -3.1391,
        44.7888, -193.419, -3.1391,
        43.7888, -193.422, -3.1391,
        42.7888, -193.424, -3.1391,
        41.7888, -193.427, -3.1391,
        40.7888, -193.429, -3.1391,
        39.7888, -193.432, -3.1391,
        38.7888, -193.434, -3.1391,
        37.7888, -193.437, -3.1391,
        36.7888, -193.439, -3.1391,
        35.7888, -193.442, -3.1391,
        34.7888, -193.444, -3.1391,
        33.7888, -193.447, -3.1391,
        32.7888, -193.449, -3.1391,
        31.7888, -193.452, -3.1391,
        30.7889, -193.454, -3.1391,
        29.7889, -193.457, -3.1391,
        28.7889, -193.459, -3.1391,
        27.7889, -193.462, -3.1391,
        26.7889, -193.464, -3.1391,
        25.7889, -193.467, -3.1391,
        24.7889, -193.469, -3.1391,
        23.7889, -193.472, -3.1391,
        22.7889, -193.474, -3.1391,
        21.7889, -193.477, -3.1391,
        20.7889, -193.479, -3.1391,
        19.7889, -193.482, -3.1391,
        18.7889, -193.484, -3.1391,
        17.7889, -193.487, -3.1391,
        16.7889, -193.489, -3.1391,
        15.7889, -193.492, -3.1391,
        14.7889, -193.494, -3.1391,
        13.7889, -193.497, -3.1391,
        12.7889, -193.499, -3.1391,
        11.7889, -193.502, -3.1391,
        10.1547, -193.506, -3.1391,
        9.42194, -193.46, 2.9927,
        8.73865, -193.297, 2.82311,
        8.09262, -193.022, 2.65352,
        7.50238, -192.641, 2.48393,
        6.98385, -192.165, 2.31514,
        6.54878, -191.606, 2.149,
        6.21215, -190.983, 1.98287,
        5.98325, -190.312, 1.81673,
        5.8542, -189.193, 1.57712,
        5.84787, -188.193, 1.57712,
        5.84154, -187.193, 1.57712,
        5.83522, -186.193, 1.57712,
        5.82889, -185.193, 1.57712,
        5.82256, -184.193, 1.57712,
        5.81623, -183.193, 1.57712,
        5.80991, -182.193, 1.57712,
        5.80358, -181.193, 1.57712,
        5.79449, -179.756, 1.57712,
        5.78816, -178.756, 1.57712,
        5.78183, -177.756, 1.57712,
        5.77551, -176.756, 1.57712,
        5.76918, -175.756, 1.57712,
        5.76285, -174.756, 1.57712,
        5.75652, -173.756, 1.57712,
        5.7502, -172.756, 1.57712,
        5.73793, -170.817, 1.57712,
        5.7316, -169.817, 1.57712,
        5.72527, -168.817, 1.57712,
        5.71894, -167.817, 1.57712,
        5.71136, -166.618, 1.57712,
        5.70503, -165.618, 1.57712,
        5.6987, -164.618, 1.57712,
        5.69238, -163.618, 1.57712,
        5.68479, -162.419, 1.57712,
        5.67846, -161.419, 1.57712,
        5.67213, -160.419, 1.57712,
        5.66581, -159.419, 1.57712,
        5.65948, -158.419, 1.57712,
        5.65315, -157.419, 1.57712,
        5.64682, -156.419, 1.57712,
        5.6405, -155.419, 1.57712,
        5.63417, -154.419, 1.57712,
        5.62784, -153.419, 1.57712,
        5.62151, -152.419, 1.57712,
        5.61519, -151.419, 1.57712,
        5.60886, -150.419, 1.57712,
        5.60253, -149.419, 1.57712,
        5.5962, -148.419, 1.57712,
        5.58988, -147.419, 1.57712,
        5.58355, -146.419, 1.57712,
        5.57722, -145.419, 1.57712,
        5.5709, -144.419, 1.57712,
        5.56201, -143.014, 1.57712,
        5.56081, -142.825, 1.57712,
        5.55448, -141.825, 1.57712,
        5.54815, -140.825, 1.57712,
        5.54182, -139.825, 1.57712,
        5.5355, -138.825, 1.57712,
        5.52917, -137.825, 1.57712,
        5.52284, -136.825, 1.57712,
        5.51652, -135.825, 1.57712,
        5.51019, -134.825, 1.57712,
        5.50386, -133.825, 1.57712,
        5.49401, -132.269, 1.57712,
        5.48769, -131.269, 1.57712,
        5.48136, -130.269, 1.57712,
        5.47503, -129.269, 1.57712,
        5.4687, -128.269, 1.57712,
        5.46238, -127.269, 1.57712,
        5.45605, -126.269, 1.57712,
        5.44972, -125.269, 1.57712,
        5.44339, -124.269, 1.57712,
        5.43274, -122.585, 1.57712,
        5.42596, -121.513, 1.57712,
        5.43176, -122.431, 1.57712,
        1.89289, -116.299, 1.57712;
    std::vector<planning_msgs::WayPoint> way_points;
    for (int i = 0; i < poses.rows(); ++i) {
      planning_msgs::WayPoint way_point;
      way_point.pose.position.x = poses(i, 0);
      way_point.pose.position.y = poses(i, 1);
      way_point.pose.orientation = tf::createQuaternionMsgFromYaw(poses(i, 2));
      way_point.lane_width = 4.0;
      way_points.push_back(way_point);
    }
    reference_line_ = planning::ReferenceLine(way_points);
    reference_line_.Smooth(13, 100.0, 1.0, 5, 5);

  }
};

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}