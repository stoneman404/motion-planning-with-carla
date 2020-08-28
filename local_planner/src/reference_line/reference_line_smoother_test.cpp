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

TEST_F(ReferenceLineSmootherTest, sin_ref_test) {
  size_t number_of_waypoints = 100;
  double ds = 1.0;
  std::vector<planning_msgs::WayPoint> way_points;
  way_points.reserve(number_of_waypoints);
  planning_msgs::WayPoint way_point;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  Eigen::MatrixXd xy(101, 2);
  xy << 0, 0,
      1, 0.0157053795390641,
      2, 0.0313952597646567,
      3, 0.0470541566592572,
      4, 0.0626666167821521,
      5, 0.0782172325201154,
      6, 0.0936906572928623,
      7, 0.109071620698271,
      8, 0.124344943582427,
      9, 0.139495553019615,
      10, 0.154508497187474,
      11, 0.169368960122646,
      12, 0.184062276342339,
      13, 0.198573945317390,
      14, 0.212889645782536,
      15, 0.226995249869773,
      16, 0.240876837050858,
      17, 0.254520707875186,
      18, 0.267913397489498,
      19, 0.281041688926065,
      20, 0.293892626146237,
      21, 0.306453526826488,
      22, 0.318711994874345,
      23, 0.330655932661826,
      24, 0.342273552964344,
      25, 0.353553390593274,
      26, 0.364484313710706,
      27, 0.375055534815230,
      28, 0.385256621387895,
      29, 0.395077506187845,
      30, 0.404508497187474,
      31, 0.413540287137281,
      32, 0.422163962751008,
      33, 0.430371013501972,
      34, 0.438153340021932,
      35, 0.445503262094184,
      36, 0.452413526233010,
      37, 0.458877312841991,
      38, 0.464888242944126,
      39, 0.470440384477113,
      40, 0.475528258147577,
      41, 0.480146842838472,
      42, 0.484291580564316,
      43, 0.487958380969374,
      44, 0.491143625364344,
      45, 0.493844170297569,
      46, 0.496057350657239,
      47, 0.497780982301540,
      48, 0.499013364214136,
      49, 0.499753280182866,
      50, 0.500000000000000,
      51, 0.499753280182866,
      52, 0.499013364214136,
      53, 0.497780982301540,
      54, 0.496057350657239,
      55, 0.493844170297569,
      56, 0.491143625364344,
      57, 0.487958380969374,
      58, 0.484291580564316,
      59, 0.480146842838472,
      60, 0.475528258147577,
      61, 0.470440384477113,
      62, 0.464888242944126,
      63, 0.458877312841991,
      64, 0.452413526233010,
      65, 0.445503262094184,
      66, 0.438153340021932,
      67, 0.430371013501972,
      68, 0.422163962751008,
      69, 0.413540287137281,
      70, 0.404508497187474,
      71, 0.395077506187845,
      72, 0.385256621387895,
      73, 0.375055534815230,
      74, 0.364484313710706,
      75, 0.353553390593274,
      76, 0.342273552964344,
      77, 0.330655932661826,
      78, 0.318711994874345,
      79, 0.306453526826488,
      80, 0.293892626146237,
      81, 0.281041688926065,
      82, 0.267913397489499,
      83, 0.254520707875186,
      84, 0.240876837050858,
      85, 0.226995249869773,
      86, 0.212889645782536,
      87, 0.198573945317391,
      88, 0.184062276342339,
      89, 0.169368960122646,
      90, 0.154508497187474,
      91, 0.139495553019615,
      92, 0.124344943582428,
      93, 0.109071620698271,
      94, 0.0936906572928623,
      95, 0.0782172325201155,
      96, 0.0626666167821523,
      97, 0.0470541566592572,
      98, 0.0313952597646568,
      99, 0.0157053795390641,
      100, 6.12323399573677e-17;
  for (size_t i = 0; i < xy.rows(); ++i) {
    way_point.pose.position.x =  xy(i, 0);
    way_point.pose.position.y =  xy(i, 1);
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
//  for (size_t i = 0; i < way_points.size(); ++i) {
//    std::cout << "x: " << way_points[i].pose.position.x << ", y: " << way_points[i].pose.position.y
//              << " heading: " << tf::getYaw(way_points[i].pose.orientation) << std::endl;
//  }
//std::cout << xy << std::endl;
//  std::cout << " ========================= " << std::endl;
//  for (size_t i = 0; i < smoothed_ref_line.reference_points().size(); ++i) {
//    const auto ref_point = smoothed_ref_line.reference_points()[i];
//    std::cout << "x: " << ref_point.x() << ", y: " << ref_point.y()
//              << " heading: " << ref_point.heading() << std::endl;
//  }
}

TEST_F(ReferenceLineSmootherTest, spline_ref_test) {
  double ds = 1.0;
  std::vector<planning_msgs::WayPoint> way_points;
  planning_msgs::WayPoint way_point;
  double s = 0.0;
  double x = 0.0;
  double y = 0.0;
  Eigen::MatrixXd xy(94, 2);
  xy <<20.4 ,  19.0 ,
      21.46000796015203 ,  18.991095809833954 ,
      22.51800605619075 ,  18.97662731813894 ,
      23.57198442400282 ,  18.951030223386013 ,
      24.619933199474943 ,  18.9087402240462 ,
      25.659842518493786 ,  18.844193018590545 ,
      26.68970251694604 ,  18.751824305490082 ,
      27.707503330718378 ,  18.626069783215854 ,
      28.711235095697486 ,  18.461365150238905 ,
      29.698887947770043 ,  18.25214610503027 ,
      30.668452022822734 ,  17.99284834606099 ,
      31.617917456742237 ,  17.677907571802102 ,
      32.54527438541524 ,  17.301759480724648 ,
      33.448512944728414 ,  16.858839771299667 ,
      34.32562327056845 ,  16.3435841419982 ,
      35.174595498822015 ,  15.750428291291282 ,
      35.993419765375805 ,  15.073807917649958 ,
      36.78008620611651 ,  14.308158719545261 ,
      37.53258495693079 ,  13.447916395448237 ,
      38.24889523175912 ,  12.4875667891199 ,
      38.925004792983074 ,  11.430738978872101 ,
      39.552607673297345 ,  10.300775591884333 ,
      40.12283941529374 ,  9.12358341813603 ,
      40.626835561564064 ,  7.92506924760664 ,
      41.05573165470011 ,  6.731139870275596 ,
      41.40066323729367 ,  5.567702076122343 ,
      41.65276585193657 ,  4.460662655126315 ,
      41.80317504122058 ,  3.435928397266957 ,
      41.84302634773751 ,  2.5194060925237074 ,
      41.76345531407918 ,  1.7370025308760022 ,
      41.555597482837356 ,  1.1146245023032861 ,
      41.21058839660385 ,  0.678178796784997  ,
      40.721323238378865 ,  0.45008183226866827 ,
      40.096224749994924,  0.42195000823577616 ,
      39.35180154989063 ,  0.5693608122260765 ,
      38.50463184073933 ,  0.8677537064955164  ,
      37.57129382521436 ,  1.2925681533000422  ,
      36.56836570598908 ,  1.8192436148956008 ,
      35.5124256857368 ,  2.423219553538138 ,
      34.420051967130895 ,  3.0799354314836025 ,
      33.307822752844686 ,  3.7648307109879404 ,
      32.192316245551524 ,  4.453344854307096 ,
      31.090110647924746 ,  5.120917323697018 ,
      30.017784162637714 ,  5.742987581413652 ,
      28.991914992363743 ,  6.294995089712943 ,
      28.02908133977618 ,  6.752379310850843 ,
      27.145861407548388 ,  7.090579707083297 ,
      26.35883339835368 ,  7.285035740666249 ,
      25.684575514865422 ,  7.311186873855647 ,
      25.136278287320504 ,  7.1487428888035875 ,
      24.706397283416536 ,  6.803550959850665 ,
      24.37954610022202 ,  6.2913434328913125 ,
      24.14032364506504 ,  5.627871170926333 ,
      23.973328825273658 ,  4.828885036956532 ,
      23.863160548175962 ,  3.9101358939827144 ,
      23.794417721100018 ,  2.887374605005682 ,
      23.751699251373907 ,  1.7763520330262423 ,
      23.719604046325692 ,  0.5928190410451968 ,
      23.682731013283455 ,  -0.6474735079366467 ,
      23.625679059575262 ,  -1.9287747509184872 ,
      23.533047092529202 ,  -3.2353338248995183 ,
      23.389434019473327 ,  -4.551399866878936 ,
      23.179438747735734 ,  -5.861222013855937 ,
      22.88766018464449 ,  -7.149049402829716 ,
      22.499175677124718 ,  -8.39933095008315 ,
      22.01094899081816 ,  -9.601478915717914 ,
      21.432342180735272 ,  -10.750082642552446 ,
      20.773298172666003 ,  -11.839974024291337 ,
      20.043759892400267 ,  -12.865984954639176 ,
      19.25367026572802 ,  -13.822947327300541 ,
      18.412972218439176 ,  -14.705693035980033 ,
      17.531608676323692 ,  -15.509053974382232 ,
      16.61952256517149 ,  -16.227862036211718 ,
      15.686656810772504 ,  -16.856949115173094 ,
      14.742954338916672 ,  -17.391147104970933 ,
      13.798358075393933 ,  -17.825287899309835 ,
      12.862810945994214 ,  -18.154203391894377 ,
      11.946255876507458 ,  -18.372725476429153 ,
      11.058635792723592 ,  -18.475686046618744 ,
      10.209893620432554 ,  -18.457916996167747 ,
      9.409972285424285 ,  -18.314250218780735 ,
      8.66879915152992 ,  -18.03953368412417 ,
      7.99219167572623 ,  -17.632861016715207 ,
      7.376363246045228 ,  -17.103247127208807 ,
      6.816157665116366 ,  -16.461121748326214 ,
      6.306418735569092 ,  -15.716914612788639 ,
      5.84199026003286 ,  -14.881055453317318 ,
      5.417716041137119 ,  -13.963974002633483 ,
      5.028439881511321 ,  -12.976099993458357 ,
      4.669005583784918 ,  -11.927863158513171 ,
      4.334256950587357 ,  -10.829693230519156 ,
      4.019037784548095 ,  -9.692019942197533 ,
      3.7181918882965777 ,  -8.525273026269538 ,
      3.4265630644622593 ,  -7.339882215456397 ;
  for (size_t i = 0; i < xy.rows(); ++i) {
    way_point.pose.position.x =  xy(i, 0);
    way_point.pose.position.y =  xy(i, 1);
    way_point.pose.position.z = 0.0;
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

//    way_point.pose.orientation = tf::createQuaternionMsgFromYaw(heading);

  }
  std::cout << "waypoints.size() " << way_points.size() << std::endl;
  auto smoothed_ref_line = ReferenceLine(way_points);
  EXPECT_EQ(smoother_->GetSmoothReferenceLine(way_points, &smoothed_ref_line), true);
//  for (size_t i = 0; i < way_points.size(); ++i) {
//    std::cout << "x: " << way_points[i].pose.position.x << ", y: " << way_points[i].pose.position.y
//              << " heading: " << tf::getYaw(way_points[i].pose.orientation) << std::endl;
//  }
//  std::cout << " ========================= " << std::endl;
//  for (size_t i = 0; i < smoothed_ref_line.reference_points().size(); ++i) {
//    const auto ref_point = smoothed_ref_line.reference_points()[i];
//    std::cout << "x: " << ref_point.x() << ", y: " << ref_point.y()
//              << " heading: " << ref_point.heading() << std::endl;
//  }

}

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "reference_line_smoother_test");
  ros::NodeHandle nh;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}