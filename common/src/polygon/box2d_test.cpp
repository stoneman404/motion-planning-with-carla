#include <gtest/gtest.h>
#include <polygon/box2d.hpp>
using namespace common;
TEST(BoxTest, corner_test) {
  Eigen::Vector2d center{0, 0};
  double len = 5.0;
  double wid = 3.0;
  double theta = 0.0;
  common::Box2d box = common::Box2d(center, theta, len, wid);
  std::vector<Eigen::Vector2d> corners = box.GetAllCorners();
  for (size_t i = 0; i < corners.size(); ++i) {
    std::cout << i << "th corner is " << corners[i].x() << ", " << corners[i].y() << std::endl;
  }
}

TEST(BoxTest, is_point_in) {
  Eigen::Vector2d center{0, 0};
  double len = 5.0;
  double wid = 3.0;
  double theta = 0.0;
  common::Box2d box = common::Box2d(center, theta, len, wid);
  Eigen::Vector2d xy{1.0, 1.0};
  EXPECT_TRUE(box.IsPointIn(xy));
}

TEST(BoxTest, is_point_on_boudary) {
  Eigen::Vector2d center{0, 0};
  double len = 5.0;
  double wid = 3.0;
  double theta = 0.0;
  common::Box2d box = common::Box2d(center, theta, len, wid);
  Eigen::Vector2d xy{-2.5, 1.0};
  EXPECT_TRUE(box.IsPointOnBoundary(xy));
  EXPECT_FALSE(box.IsPointOnBoundary({0.0, 0.0}));
}

TEST(BoxTest, overlap_test) {
  const double len = 5.0;
  const double width = 3.0;
  const double theta1 = 1.57669;
  const double theta2 = /*0.25 * */ 0.0;
  Eigen::Vector2d center1{-43.872, 69.0341};
  Eigen::Vector2d center2{-43.872, 69.0341};
  common::Box2d box1 = common::Box2d(center1, 0.0, 9, 1.6);
  common::Box2d box2 = common::Box2d(center2, 0.0, 9, 1.6);
  EXPECT_TRUE(box1.HasOverlapWithBox2d(box2));
}

namespace {
Box2d box1({0, 0}, 0, 4, 2);

Box2d box2({5, 2}, 0, 4, 2);
Box2d box3({1, 2}, 0, 4, 2);
Box2d box4({7, 8}, M_PI_4, 5.0, 3.0);
Box2d box5({7, 9}, 0.0, 5.0, 3.0);
Box2d box6({-43.872, 69.0341}, 0.0, 5, 3);
Box2d box7({-43.8, 69.03}, M_PI_2, 5, 3);
}

TEST(Box2dTest, GetAllCorners) {
  std::vector<Eigen::Vector2d> corners1 = box1.GetAllCorners();
  EXPECT_NEAR(corners1[0].x(), 2.0, 1e-5);
  EXPECT_NEAR(corners1[0].y(), -1.0, 1e-5);
  EXPECT_NEAR(corners1[1].x(), 2.0, 1e-5);
  EXPECT_NEAR(corners1[1].y(), 1.0, 1e-5);
  EXPECT_NEAR(corners1[2].x(), -2.0, 1e-5);
  EXPECT_NEAR(corners1[2].y(), 1.0, 1e-5);
  EXPECT_NEAR(corners1[3].x(), -2.0, 1e-5);
  EXPECT_NEAR(corners1[3].y(), -1.0, 1e-5);
}

TEST(Box2dTest, HasOverlap) {
  EXPECT_FALSE(box1.HasOverlapWithBox2d(box2));
  EXPECT_FALSE(box1.HasOverlapWithBox2d(box4));
  EXPECT_FALSE(box2.HasOverlapWithBox2d(box4));
  EXPECT_TRUE(box1.HasOverlapWithBox2d(box3));
  EXPECT_TRUE(box1.HasOverlapWithBox2d(box1));
  EXPECT_TRUE(box4.HasOverlapWithBox2d(box5));
  EXPECT_TRUE(box7.HasOverlapWithBox2d(box6));
  EXPECT_TRUE(box6.HasOverlapWithBox2d(box7));

}

TEST(Box2dTest, DistanceTo) {
  EXPECT_NEAR(box1.DistanceToPoint({3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceToPoint({-3, 0}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceToPoint({0, 2}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceToPoint({0, -2}), 1.0, 1e-5);
  EXPECT_NEAR(box1.DistanceToPoint({0, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceToPoint({0, 1}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceToPoint({1, 0}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceToPoint({0, -1}), 0.0, 1e-5);
  EXPECT_NEAR(box1.DistanceToPoint({-1, 0}), 0.0, 1e-5);

}

TEST(Box2dTest, IsPointIn) {
  EXPECT_TRUE(box1.IsPointIn({0, 0}));
  EXPECT_TRUE(box1.IsPointIn({1, 0.5}));
  EXPECT_TRUE(box1.IsPointIn({-0.5, -1}));
  EXPECT_TRUE(box1.IsPointIn({2, 1}));
  EXPECT_FALSE(box1.IsPointIn({-3, 0}));
  EXPECT_FALSE(box1.IsPointIn({0, 2}));
  EXPECT_FALSE(box1.IsPointIn({-4, -2}));
}

TEST(Box2dTest, IsPointOnBoundary) {
  EXPECT_FALSE(box1.IsPointOnBoundary({0, 0}));
  EXPECT_FALSE(box1.IsPointOnBoundary({1, 0.5}));
  EXPECT_TRUE(box1.IsPointOnBoundary({-0.5, -1}));
  EXPECT_TRUE(box1.IsPointOnBoundary({2, 0.5}));
  EXPECT_TRUE(box1.IsPointOnBoundary({-2, 1}));
  EXPECT_FALSE(box1.IsPointOnBoundary({-3, 0}));
  EXPECT_FALSE(box1.IsPointOnBoundary({0, 2}));
  EXPECT_FALSE(box1.IsPointOnBoundary({-4, -2}));
}

TEST(Box2dTest, RotateFromCenterAndShift) {
  Box2d box({0, 0}, 0, 4, 2);
  std::vector<Eigen::Vector2d> corners = box.GetAllCorners();
  EXPECT_NEAR(box.heading(), 0.0, 1e-5);
  box.RotateFromCenter(M_PI_2);
  EXPECT_NEAR(box.heading(), M_PI_2, 1e-5);
  corners = box.GetAllCorners();

  EXPECT_NEAR(corners[0].x(), 1.0, 1e-5);
  EXPECT_NEAR(corners[0].y(), 2.0, 1e-5);
  EXPECT_NEAR(corners[1].x(), -1.0, 1e-5);
  EXPECT_NEAR(corners[1].y(), 2.0, 1e-5);
  EXPECT_NEAR(corners[2].x(), -1.0, 1e-5);
  EXPECT_NEAR(corners[2].y(), -2.0, 1e-5);
  EXPECT_NEAR(corners[3].x(), 1.0, 1e-5);
  EXPECT_NEAR(corners[3].y(), -2.0, 1e-5);

  box.Shift({30, 40});
  corners = box.GetAllCorners();
  EXPECT_NEAR(corners[0].x(), 31.0, 1e-5);
  EXPECT_NEAR(corners[0].y(), 42.0, 1e-5);
  EXPECT_NEAR(corners[1].x(), 29.0, 1e-5);
  EXPECT_NEAR(corners[1].y(), 42.0, 1e-5);
  EXPECT_NEAR(corners[2].x(), 29.0, 1e-5);
  EXPECT_NEAR(corners[2].y(), 38.0, 1e-5);
  EXPECT_NEAR(corners[3].x(), 31.0, 1e-5);
  EXPECT_NEAR(corners[3].y(), 38.0, 1e-5);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}