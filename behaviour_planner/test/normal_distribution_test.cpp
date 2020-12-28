#include <boost/math/distributions/normal.hpp>
#include <iostream>
#include <cmath>

double norm_distribution(double x, double mean, double square_error) {
  return 1.0 / (square_error * std::sqrt(2 * M_PI))
      * exp(-(x - mean) * (x - mean) / (2.0 * square_error * square_error));
}
int main() {

  std::cout << norm_distribution(6, 0, 4) << std::endl;
  boost::math::normal_distribution<> norm(0, 4);//期望，方差
  std::cout << boost::math::pdf(norm, 6) << std::endl;
  size_t i = 10;
  for (; i > 0; i--) {
    std::cout << i << ", ";
  }
  std::cout << std::endl;
  std::vector<int> vec{0, 2, -3, 2, 5, 3, 5, 65, 254, 6, 10};
  for (size_t j = 0; j < vec.size(); j++) {
    std::cout << vec[j] << ", ";
  }
  std::cout << std::endl;
  std::cout << "now the i is :" << i << std::endl;
//  double PI = acos(-1);
//  std::cout << boost::math::cdf(norm, 0) << std::endl; //0.5  概率密度值，小于0的概率
//  std::cout << boost::math::pdf(norm, 0) - 2 / (sqrt(2 * PI)) << std::endl; //0   正态分布函数值
//  std::cout << boost::math::cdf(boost::math::complement(norm, 1)) << std::endl; // 概率密度大于1的概率
//  std::cout << boost::math::quantile(norm, 0.5) << std::endl;        //当概率密度小于0.5时对应的x值
  return 0;
}
