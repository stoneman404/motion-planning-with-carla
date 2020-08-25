#include "reference_line/reference_line_smoother.hpp"
#include <gtest/gtest.h>
#include <tf/transform_datatypes.h>

namespace planning {
class ReferenceLineSmootherTest : public testing::Test {
protected:
    void SetUp() override {
      smoother_ = std::unique_ptr<ReferenceLineSmoother>(new ReferenceLineSmoother());
    }
    void TearDown() override {}
    std::unique_ptr<ReferenceLineSmoother> smoother_;

};

TEST_F(ReferenceLineSmootherTest, line_ref_test) {
  size_t number_of_waypoints = 100;
  double heading = 0.0;
  double ds = 1.0;
  std::vector<planning_msgs::WayPoint> way_points;
  way_points.reserve(number_of_waypoints);
  planning_msgs::WayPoint way_point;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  for (size_t i = 0; i < number_of_waypoints; ++i) {
    way_point.s = s;
    way_point.pose.position.x = x;
    way_point.pose.position.y = y;
    way_point.pose.position.z = 0.0;
    way_point.pose.orientation = tf::createQuaternionMsgFromYaw(heading);
    way_point.lane_width = 5.0;
    way_point.lane_id = 1;
    way_point.section_id = 1;
    way_point.road_id = 1;
    way_point.id = i;
    way_point.has_left_lane = false;
    way_point.has_right_lane = false;
    way_point.has_value = true;
    way_point.is_junction = false;
    way_points.push_back(way_point);
    s += ds;
    x += ds * std::cos(heading);
    y += ds * std::sin(heading);
  }
  std::cout << "waypoints.size() " << way_points.size() << std::endl;
  auto smoothed_ref_line = ReferenceLine(way_points);
  EXPECT_EQ(smoother_->GetSmoothReferenceLine(way_points, &smoothed_ref_line), true);
  for (size_t i = 0; i < way_points.size(); ++i) {
    std::cout << "x: " << way_points[i].pose.position.x << ", y: " << way_points[i].pose.position.y
              << " heading: " << tf::getYaw(way_points[i].pose.orientation) << std::endl;
  }
  std::cout << " ========================= " << std::endl;
  for (size_t i = 0; i < smoothed_ref_line.reference_points().size(); ++i) {
    const auto ref_point = smoothed_ref_line.reference_points()[i];
    std::cout << "x: " << ref_point.x() << ", y: " << ref_point.y()
              << " heading: " << ref_point.heading() << std::endl;
  }

}

TEST_F(ReferenceLineSmootherTest, sin_ref_test){
  size_t number_of_waypoints = 100;
  double ds = 1.0;
  std::vector<planning_msgs::WayPoint> way_points;
  way_points.reserve(number_of_waypoints);
  planning_msgs::WayPoint way_point;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  Eigen::MatrixXd xy(101, 2);
  xy << 0,	0,
  1,	0.0157053795390641,
  2,	0.0313952597646567,
  3,	0.0470541566592572,
  4,	0.0626666167821521,
  5,	0.0782172325201154,
  6,	0.0936906572928623,
  7,	0.109071620698271,
  8,	0.124344943582427,
  9,	0.139495553019615,
  10,	0.154508497187474,
  11,	0.169368960122646,
  12,	0.184062276342339,
  13,	0.198573945317390,
  14,	0.212889645782536,
  15,	0.226995249869773,
  16,	0.240876837050858,
  17,	0.254520707875186,
  18,	0.267913397489498,
  19,	0.281041688926065,
  20,	0.293892626146237,
  21,	0.306453526826488,
  22,	0.318711994874345,
  23,	0.330655932661826,
  24,	0.342273552964344,
  25,	0.353553390593274,
  26,	0.364484313710706,
  27,	0.375055534815230,
  28,	0.385256621387895,
  29,	0.395077506187845,
  30,	0.404508497187474,
  31,	0.413540287137281,
  32,	0.422163962751008,
  33,	0.430371013501972,
  34,	0.438153340021932,
  35,	0.445503262094184,
  36,	0.452413526233010,
  37,	0.458877312841991,
  38,	0.464888242944126,
  39,	0.470440384477113,
  40,	0.475528258147577,
  41,	0.480146842838472,
  42,	0.484291580564316,
  43,	0.487958380969374,
  44,	0.491143625364344,
  45,	0.493844170297569,
  46,	0.496057350657239,
  47,	0.497780982301540,
  48,	0.499013364214136,
  49,	0.499753280182866,
  50,	0.500000000000000,
  51,	0.499753280182866,
  52,	0.499013364214136,
  53,	0.497780982301540,
  54,	0.496057350657239,
  55,	0.493844170297569,
  56,	0.491143625364344,
  57,	0.487958380969374,
  58,	0.484291580564316,
  59,	0.480146842838472,
  60,	0.475528258147577,
  61,	0.470440384477113,
  62,	0.464888242944126,
  63,	0.458877312841991,
  64,	0.452413526233010,
  65,	0.445503262094184,
  66,	0.438153340021932,
  67,	0.430371013501972,
  68,	0.422163962751008,
  69,	0.413540287137281,
  70,	0.404508497187474,
  71,	0.395077506187845,
  72,	0.385256621387895,
  73,	0.375055534815230,
  74,	0.364484313710706,
  75,	0.353553390593274,
  76,	0.342273552964344,
  77,	0.330655932661826,
  78,	0.318711994874345,
  79,	0.306453526826488,
  80,	0.293892626146237,
  81,	0.281041688926065,
  82,	0.267913397489499,
  83,	0.254520707875186,
  84,	0.240876837050858,
  85,	0.226995249869773,
  86,	0.212889645782536,
  87,	0.198573945317391,
  88,	0.184062276342339,
  89,	0.169368960122646,
  90,	0.154508497187474,
  91,	0.139495553019615,
  92,	0.124344943582428,
  93,	0.109071620698271,
  94,	0.0936906572928623,
  95,	0.0782172325201155,
  96,	0.0626666167821523,
  97,	0.0470541566592572,
  98,	0.0313952597646568,
  99,	0.0157053795390641,
  100,	6.12323399573677e-17;
  for (size_t i = 0; i < xy.rows(); ++i){
    way_point.pose.position.x = 0.4 * xy(i, 0);
    way_point.pose.position.y = 1.5 * xy(i, 1);
    way_point.pose.position.z = 0.0;
    way_point.lane_width = 4.0;
    way_point.lane_id = 1;
    way_point.section_id = 1;
    way_point.road_id = 1;
    way_point.id = i;
    way_point.has_left_lane = false;
    way_point.has_right_lane = false;
    way_point.has_value = true;
    way_point.is_junction = false;
    way_points.push_back(way_point);

//    way_point.pose.orientation = tf::createQuaternionMsgFromYaw(heading);

  }
  std::cout << "waypoints.size() " << way_points.size() << std::endl;
  auto smoothed_ref_line = ReferenceLine(way_points);
  EXPECT_EQ(smoother_->GetSmoothReferenceLine(way_points, &smoothed_ref_line), true);
  for (size_t i = 0; i < way_points.size(); ++i) {
    std::cout << "x: " << way_points[i].pose.position.x << ", y: " << way_points[i].pose.position.y
              << " heading: " << tf::getYaw(way_points[i].pose.orientation) << std::endl;
  }
  std::cout << " ========================= " << std::endl;
  for (size_t i = 0; i < smoothed_ref_line.reference_points().size(); ++i) {
    const auto ref_point = smoothed_ref_line.reference_points()[i];
    std::cout << "x: " << ref_point.x() << ", y: " << ref_point.y()
              << " heading: " << ref_point.heading() << std::endl;
  }
}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "reference_line_smoother_test");
  ros::NodeHandle nh;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}