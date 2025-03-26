#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(4, 0, " ", " ", "", "")
#include <iostream>
#include <sophus/so3.hpp>

#include "sophus/se3.hpp"

// # link1 pos = [0, 0, 0.333] quat = [1, 0, 0, 0] joint_axis = [0, 0, 1]
// # link2 pos = [0, 0, 0] quat = [1, -1, 0, 0] joint_axis = [0, 0, 1]
// # link3 pos = [0, -0.316, 0] quat = [1, 1, 0, 0] joint_axis = [0, 0, 1]
// # link4 pos = [0.0825, 0, 0] quat = [1, 1, 0, 0] joint_axis = [0, 0, 1]
// # link5 pos = [-0.0825, 0.384, 0] quat = [1, -1, 0, 0] joint_axis = [0, 0, 1]
// # link6 pos = [0, 0, 0] quat = [1, 1, 0, 0] joint_axis = [0, 0, 1]
// # link7 pos = [0.088, 0, 0] quat = [1, 1, 0, 0] joint_axis = [0, 0, 1]
// # attachment pos = [0, 0, 0.107] quat = [0.3826834, 0, 0, 0.9238795]

int test1() {
  // 输入数据
  Eigen::Vector3d pos1 = Eigen::Vector3d(0, 0, 0.333);
  Eigen::Quaterniond quat1 = Eigen::Quaterniond(1, 0, 0, 0);
  Eigen::Vector3d joint_axis1 = Eigen::Vector3d(0, 0, 1);
  Sophus::SE3d tf_w1(quat1, pos1);

  Eigen::Vector3d pos2 = Eigen::Vector3d(0, 0, 0);
  Eigen::Quaterniond quat2 = Eigen::Quaterniond(1, -1, 0, 0);
  Eigen::Vector3d joint_axis2 = Eigen::Vector3d(0, 0, 1);
  Sophus::SE3d tf_12(quat2, pos2);

  Eigen::Vector3d pos3 = Eigen::Vector3d(0, -0.316, 0);
  Eigen::Quaterniond quat3 = Eigen::Quaterniond(1, 1, 0, 0);
  Eigen::Vector3d joint_axis3 = Eigen::Vector3d(0, 0, 1);
  Sophus::SE3d tf_23(quat3, pos3);

  Eigen::Vector3d pos4 = Eigen::Vector3d(0.0825, 0, 0);
  Eigen::Quaterniond quat4 = Eigen::Quaterniond(1, 1, 0, 0);
  Eigen::Vector3d joint_axis4 = Eigen::Vector3d(0, 0, 1);
  Sophus::SE3d tf_34(quat4, pos4);

  Eigen::Vector3d pos5 = Eigen::Vector3d(-0.0825, 0.384, 0);
  Eigen::Quaterniond quat5 = Eigen::Quaterniond(1, -1, 0, 0);
  Eigen::Vector3d joint_axis5 = Eigen::Vector3d(0, 0, 1);
  Sophus::SE3d tf_45(quat5, pos5);

  Eigen::Vector3d pos6 = Eigen::Vector3d(0, 0, 0);
  Eigen::Quaterniond quat6 = Eigen::Quaterniond(1, 1, 0, 0);
  Eigen::Vector3d joint_axis6 = Eigen::Vector3d(0, 0, 1);
  Sophus::SE3d tf_56(quat6, pos6);

  Eigen::Vector3d pos7 = Eigen::Vector3d(0.088, 0, 0);
  Eigen::Quaterniond quat7 = Eigen::Quaterniond(1, 1, 0, 0);
  Eigen::Vector3d joint_axis7 = Eigen::Vector3d(0, 0, 1);
  Sophus::SE3d tf_67(quat7, pos7);

  Eigen::Vector3d pos_attachment = Eigen::Vector3d(0, 0, 0.107);
  Eigen::Quaterniond quat_attachment =
      Eigen::Quaterniond(0.3826834, 0, 0, 0.9238795);
  Sophus::SE3d tf_7attachment(quat_attachment, pos_attachment);

  auto tf_w2 = tf_w1 * tf_12;
  auto tf_w3 = tf_w1 * tf_12 * tf_23;
  auto tf_w4 = tf_w1 * tf_12 * tf_23 * tf_34;
  auto tf_w5 = tf_w1 * tf_12 * tf_23 * tf_34 * tf_45;
  auto tf_w6 = tf_w1 * tf_12 * tf_23 * tf_34 * tf_45 * tf_56;
  auto tf_w7 = tf_w1 * tf_12 * tf_23 * tf_34 * tf_45 * tf_56 * tf_67;

  // 计算twist
  Eigen::Vector3d w1 = tf_w1.rotationMatrix() * joint_axis1;
  Eigen::Vector3d w2 = tf_w2.rotationMatrix() * joint_axis2;
  Eigen::Vector3d w3 = tf_w3.rotationMatrix() * joint_axis3;
  Eigen::Vector3d w4 = tf_w4.rotationMatrix() * joint_axis4;
  Eigen::Vector3d w5 = tf_w5.rotationMatrix() * joint_axis5;
  Eigen::Vector3d w6 = tf_w6.rotationMatrix() * joint_axis6;
  Eigen::Vector3d w7 = tf_w7.rotationMatrix() * joint_axis7;

  Eigen::Vector3d p1 = tf_w1.translation();
  Eigen::Vector3d p2 = tf_w2.translation();
  Eigen::Vector3d p3 = tf_w3.translation();
  Eigen::Vector3d p4 = tf_w4.translation();
  Eigen::Vector3d p5 = tf_w5.translation();
  Eigen::Vector3d p6 = tf_w6.translation();
  Eigen::Vector3d p7 = tf_w7.translation();

  Eigen::Vector3d v1 = -w1.cross(p1);
  Eigen::Vector3d v2 = -w2.cross(p2);
  Eigen::Vector3d v3 = -w3.cross(p3);
  Eigen::Vector3d v4 = -w4.cross(p4);
  Eigen::Vector3d v5 = -w5.cross(p5);
  Eigen::Vector3d v6 = -w6.cross(p6);
  Eigen::Vector3d v7 = -w7.cross(p7);

  std::vector<double> control_input = {0.5,         0.50694025, 0.49901801,
                                       -0.50329994, 0.49931873, 0.49942658};
  Sophus::SE3d M = tf_w7 * tf_7attachment;
  Eigen::Matrix<double, 6, 1> S1;
  S1 << v1, w1;
  Eigen::Matrix<double, 6, 1> S2;
  S2 << v2, w2;
  Eigen::Matrix<double, 6, 1> S3;
  S3 << v3, w3;
  Eigen::Matrix<double, 6, 1> S4;
  S4 << v4, w4;
  Eigen::Matrix<double, 6, 1> S5;
  S5 << v5, w5;
  Eigen::Matrix<double, 6, 1> S6;
  S6 << v6, w6;
  Eigen::Matrix<double, 6, 1> S7;
  S7 << v7, w7;

  auto S_to_string = [](const Eigen::Matrix<double, 6, 1> &S) {
    return "    [" + std::to_string(S[3]) + ", " + std::to_string(S[4]) + ", " +
           std::to_string(S[5]) + ", " + std::to_string(S[0]) + ", " +
           std::to_string(S[1]) + ", " + std::to_string(S[2]) + "],\n";
  };

  auto tf_to_string = [](const Sophus::SE3d &tf) {
    auto matrix = tf.matrix();
    std::string result = "    [";
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        result += std::to_string(matrix(i, j)) + ", ";
      }
    }
    result.pop_back();
    result.pop_back();
    result += "],\n";
    return result;
  };

  std::cout << "joint_twist = [\n"
            << S_to_string(S1) << S_to_string(S2) << S_to_string(S3)
            << S_to_string(S4) << S_to_string(S5) << S_to_string(S6)
            << S_to_string(S7) << "]" << std::endl;

  std::cout << "joint_placement = [\n"
            << tf_to_string(tf_w1) << tf_to_string(tf_12) << tf_to_string(tf_23)
            << tf_to_string(tf_34) << tf_to_string(tf_45) << tf_to_string(tf_56)
            << tf_to_string(tf_67) << tf_to_string(tf_7attachment) << "]"
            << std::endl;

  Sophus::SE3d T = Sophus::SE3d::exp(S1 * control_input[0]) *
                   Sophus::SE3d::exp(S2 * control_input[1]) *
                   Sophus::SE3d::exp(S3 * control_input[2]) *
                   Sophus::SE3d::exp(S4 * control_input[3]) *
                   Sophus::SE3d::exp(S5 * control_input[4]) *
                   Sophus::SE3d::exp(S6 * control_input[5]) *
                   Sophus::SE3d::exp(S7 * control_input[6]) * M;
  std::cout << "translation: \n" << T.translation() << std::endl;

  return 0;
}

int test2() {
  Eigen::Vector3d w1 = Eigen::Vector3d(0, 0, 1);
  Eigen::Vector3d w2 = Eigen::Vector3d(0, 0, 1);

  Eigen::Vector3d p1 = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector3d p2 = Eigen::Vector3d(2, 0, 0);

  Eigen::Vector3d v1 = -w1.cross(p1);
  Eigen::Vector3d v2 = -w2.cross(p2);

  Eigen::Matrix<double, 6, 1> S1;
  S1 << v1, w1;
  Sophus::SE3d::Transformation bracket_S1 = Sophus::SE3d::hat(S1);

  Eigen::Matrix<double, 6, 1> S2;
  S2 << v2, w2;

  Sophus::SE3d M =
      Sophus::SE3d(Eigen::Matrix3d::Identity(), Eigen::Vector3d(4, 0, 0));

  const double kPi = Sophus::Constants<double>::pi();
  std::vector<double> control_input = {kPi / 2, -kPi / 2};

  Sophus::SE3d T = Sophus::SE3d::exp(S1 * control_input[0]) *
                   Sophus::SE3d::exp(S2 * control_input[1]) * M;
  std::cout << "translation: \n" << T.translation() << std::endl;
  return 0;
}

int main() {
  test1();
  test2();
  return 0;
}