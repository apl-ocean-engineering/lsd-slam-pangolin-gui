//
// g++ -ggdb -I/usr/include/eigen3 -I./thirdparty/Sophus -o Sophus_test Sophus_test.cpp
//

#include <stdio.h>
#include "sophus/se3.hpp"

using namespace std;

int main( int argc, char **argv )
{
  Eigen::Matrix<float,6,1> inc;
  inc <<  -0.000324532f, 0.00119352f, -1.57896e-05f, -0.000128844f, 0.00168068f, 0.00248538f;
	Sophus::SE3f exp_increment = Sophus::SE3f::exp((inc));

  printf("     log exp_increment =%.4f %.4f %.4f %.4f %.4f %.4f\n",
      exp_increment.log()[0],exp_increment.log()[1],exp_increment.log()[2],
      exp_increment.log()[3],exp_increment.log()[4],exp_increment.log()[5]);

  // Eigen::Matrix3f omega;
  //   omega << 0, 0.00248538004, -0.00168067997, -0.00248538004, 0, -0.000128843996, 0.00168067997, 0.000128843996, 0;
  //   cout << omega << endl;
  //
  // Eigen::Matrix3f omega_sq = omega*omega;
  // cout << omega_sq << endl;
}
