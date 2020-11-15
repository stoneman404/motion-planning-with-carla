#include "curves/spline2d.hpp"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "math/math_utils.hpp"

namespace common {
class Spline2dTest : public testing::Test {
 public:
  size_t order_;
  std::unique_ptr<Spline2d> spline2d_;
 protected:
  void SetUp() override;
};

void Spline2dTest::SetUp() {
  order_ = 5;
  Eigen::MatrixXd xy(94, 2);
  xy << 20.4, 19.0,
      21.46000796015203, 18.991095809833954,
      22.51800605619075, 18.97662731813894,
      23.57198442400282, 18.951030223386013,
      24.619933199474943, 18.9087402240462,
      25.659842518493786, 18.844193018590545,
      26.68970251694604, 18.751824305490082,
      27.707503330718378, 18.626069783215854,
      28.711235095697486, 18.461365150238905,
      29.698887947770043, 18.25214610503027,
      30.668452022822734, 17.99284834606099,
      31.617917456742237, 17.677907571802102,
      32.54527438541524, 17.301759480724648,
      33.448512944728414, 16.858839771299667,
      34.32562327056845, 16.3435841419982,
      35.174595498822015, 15.750428291291282,
      35.993419765375805, 15.073807917649958,
      36.78008620611651, 14.308158719545261,
      37.53258495693079, 13.447916395448237,
      38.24889523175912, 12.4875667891199,
      38.925004792983074, 11.430738978872101,
      39.552607673297345, 10.300775591884333,
      40.12283941529374, 9.12358341813603,
      40.626835561564064, 7.92506924760664,
      41.05573165470011, 6.731139870275596,
      41.40066323729367, 5.567702076122343,
      41.65276585193657, 4.460662655126315,
      41.80317504122058, 3.435928397266957,
      41.84302634773751, 2.5194060925237074,
      41.76345531407918, 1.7370025308760022,
      41.555597482837356, 1.1146245023032861,
      41.21058839660385, 0.678178796784997,
      40.721323238378865, 0.45008183226866827,
      40.096224749994924, 0.42195000823577616,
      39.35180154989063, 0.5693608122260765,
      38.50463184073933, 0.8677537064955164,
      37.57129382521436, 1.2925681533000422,
      36.56836570598908, 1.8192436148956008,
      35.5124256857368, 2.423219553538138,
      34.420051967130895, 3.0799354314836025,
      33.307822752844686, 3.7648307109879404,
      32.192316245551524, 4.453344854307096,
      31.090110647924746, 5.120917323697018,
      30.017784162637714, 5.742987581413652,
      28.991914992363743, 6.294995089712943,
      28.02908133977618, 6.752379310850843,
      27.145861407548388, 7.090579707083297,
      26.35883339835368, 7.285035740666249,
      25.684575514865422, 7.311186873855647,
      25.136278287320504, 7.1487428888035875,
      24.706397283416536, 6.803550959850665,
      24.37954610022202, 6.2913434328913125,
      24.14032364506504, 5.627871170926333,
      23.973328825273658, 4.828885036956532,
      23.863160548175962, 3.9101358939827144,
      23.794417721100018, 2.887374605005682,
      23.751699251373907, 1.7763520330262423,
      23.719604046325692, 0.5928190410451968,
      23.682731013283455, -0.6474735079366467,
      23.625679059575262, -1.9287747509184872,
      23.533047092529202, -3.2353338248995183,
      23.389434019473327, -4.551399866878936,
      23.179438747735734, -5.861222013855937,
      22.88766018464449, -7.149049402829716,
      22.499175677124718, -8.39933095008315,
      22.01094899081816, -9.601478915717914,
      21.432342180735272, -10.750082642552446,
      20.773298172666003, -11.839974024291337,
      20.043759892400267, -12.865984954639176,
      19.25367026572802, -13.822947327300541,
      18.412972218439176, -14.705693035980033,
      17.531608676323692, -15.509053974382232,
      16.61952256517149, -16.227862036211718,
      15.686656810772504, -16.856949115173094,
      14.742954338916672, -17.391147104970933,
      13.798358075393933, -17.825287899309835,
      12.862810945994214, -18.154203391894377,
      11.946255876507458, -18.372725476429153,
      11.058635792723592, -18.475686046618744,
      10.209893620432554, -18.457916996167747,
      9.409972285424285, -18.314250218780735,
      8.66879915152992, -18.03953368412417,
      7.99219167572623, -17.632861016715207,
      7.376363246045228, -17.103247127208807,
      6.816157665116366, -16.461121748326214,
      6.306418735569092, -15.716914612788639,
      5.84199026003286, -14.881055453317318,
      5.417716041137119, -13.963974002633483,
      5.028439881511321, -12.976099993458357,
      4.669005583784918, -11.927863158513171,
      4.334256950587357, -10.829693230519156,
      4.019037784548095, -9.692019942197533,
      3.7181918882965777, -8.525273026269538,
      3.4265630644622593, -7.339882215456397;
  std::vector<double> xs(xy.rows(), 0.0);
  std::vector<double> ys(xy.rows(), 0.0);
  for (size_t i = 0; i < xy.rows(); ++i) {
    xs[i] = xy(i, 0);
    ys[i] = xy(i, 1);
  }
  spline2d_ = std::make_unique<Spline2d>(xs, ys, order_);
}

TEST_F(Spline2dTest, arc_length) {

  EXPECT_EQ(spline2d_->Order(), order_);
//  std::cout << spline2d_->ArcLength() << std::endl;
  EXPECT_NEAR(spline2d_->ArcLength(), spline2d_->ChordLength().back(), 0.2);
//  std::cout << spline2d_->ArcLength() << ", chord length "
//            << spline2d_->ChordLength().back() << std::endl;
}

TEST_F(Spline2dTest, evaluate) {
  double x, y;
  bool result = spline2d_->Evaluate(0, &x, &y);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(x, 20.4);
  EXPECT_DOUBLE_EQ(y, 19.0);
  double s = spline2d_->ArcLength();
  result = spline2d_->Evaluate(s, &x, &y);
  EXPECT_TRUE(result);
  EXPECT_DOUBLE_EQ(x, 3.4265630644622593);
  EXPECT_DOUBLE_EQ(y, -7.339882215456397);

  s = 2.5;
  result = spline2d_->Evaluate(s, &x, &y);
  EXPECT_NEAR(x, 23.045623322632196, 0.2);
  EXPECT_NEAR(y, 18.965567614990277, 0.2);
}

TEST_F(Spline2dTest, closed_point) {
  double x = 3.5265630644622593 - 0.1;
  double y = -7.339882215456397;
  double nearest_x, nearest_y, nearest_s;
  bool result = spline2d_->GetNearestPointOnSpline(x, y, &nearest_x, &nearest_y, &nearest_s);
  EXPECT_TRUE(result);
  EXPECT_NEAR(nearest_s, spline2d_->ArcLength(), 0.1);
  EXPECT_NEAR(nearest_x, 3.4265630644622593, 0.1);
  EXPECT_NEAR(nearest_y, -7.339882215456397, 0.1);
  x = 20;
  y = 0;
  result = spline2d_->GetNearestPointOnSpline(x, y, &nearest_x, &nearest_y, &nearest_s);
//  std::cout << nearest_s << ", x: " << nearest_x << ", y: " << nearest_y << std::endl;
  EXPECT_TRUE(result);
//  EXPECT_NEAR(nearest_x, 23.719604046325692, 0.1);
//  EXPECT_NEAR(nearest_y, 0.5928190410451968, 0.1);
  x = 40.47;
  y = 2.4;

  result = spline2d_->GetNearestPointOnSpline(x, y, &nearest_x, &nearest_y, &nearest_s);
  EXPECT_TRUE(result);
//  EXPECT_NEAR(nearest_x, x, 1.0);
//  std::cout << "nearest_x : " << nearest_x << " , neareset_y: " << nearest_y << std::endl;
//  std::cout << "nearest_s : " << nearest_s << std::endl;

}

TEST_F(Spline2dTest, derivatives) {
  double dx, dy, ddx, ddy, dddx, dddy;
  double s = spline2d_->ArcLength() - 40;
  bool result = spline2d_->EvaluateFirstDerivative(s, &dx, &dy);
  EXPECT_TRUE(result);
//  std::cout << "dx: " << dx << ", dy: " << dy << std::endl;
//  std::cout << "yaw : " << std::atan2(dy, dx) << std::endl;
  result = spline2d_->EvaluateSecondDerivative(s, &dx, &dy);
  EXPECT_TRUE(result);

//  std::cout << "ddx: " << ddx << ", ddy: " << ddy << std::endl;
//  std::cout << "yaw : " << std::atan2(dy, dx) << std::endl;
  result = spline2d_->EvaluateThirdDerivative(s, &ddx, &ddy);
  EXPECT_TRUE(result);
  double kappa = MathUtils::CalcKappa(dx, dy, ddx, ddy);
//  std::cout << "kappa : " << kappa << std::endl;
  result = spline2d_->EvaluateThirdDerivative(s, &dddx, &dddy);
  EXPECT_TRUE(result);
  double dkappa = MathUtils::CalcDKappa(dx, dy, ddx, ddy, dddx, dddy);
//  std::cout << "dkappa: " << dkappa << std::endl;
}

TEST_F(Spline2dTest, spline) {
  Eigen::MatrixXd xy(18, 2);
  xy << 127.413, -196.713,
      122.789, -193.225,
      122.789, -193.225,
      121.789, -193.227,
      120.789, -193.23,
      119.789, -193.232,
      118.789, -193.235,
      117.789, -193.237,
      116.789, -193.24,
      115.789, -193.242,
      114.789, -193.245,
      113.789, -193.247,
      112.789, -193.25,
      111.789, -193.252,
      110.789, -193.255,
      109.789, -193.257,
      108.789, -193.26,
      107.789, -193.262;
  std::vector<double> xs(xy.rows(), 0.0);
  std::vector<double> ys(xy.rows(), 0.0);
  for (size_t i = 0; i < xy.rows(); ++i) {
    xs[i] = xy(i, 0);
    ys[i] = xy(i, 1);
  }
  auto spline2d = std::make_unique<Spline2d>(xs, ys, order_);
  double x, y, s;
  std::cout << "arc length: " << spline2d->ArcLength() << std::endl;
  spline2d->GetNearestPointOnSpline(109, -193, &x, &y, &s);
  std::cout << "x: " << x << " y: " << y << " s: " << s << std::endl;
}
}

int main(int argc, char **argv) {
//  ros::init(argc, argv, "spline2d_test");
//  ros::NodeHandle nh;
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}