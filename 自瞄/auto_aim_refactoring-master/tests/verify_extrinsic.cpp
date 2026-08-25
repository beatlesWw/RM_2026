#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

// 验证外参标定质量的测试程序
// 使用方法：在不同角度拍摄装甲板，检查重投影误差

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <config_path>" << std::endl;
        return -1;
    }

    // 读取配置
    auto yaml = YAML::LoadFile(argv[1]);
    auto R_camera2gimbal_data = yaml["R_camera2gimbal"].as<std::vector<double>>();
    auto t_camera2gimbal_data = yaml["t_camera2gimbal"].as<std::vector<double>>();
    auto camera_matrix_data = yaml["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

    Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
    Eigen::Vector3d t_camera2gimbal = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());
    
    cv::Mat camera_matrix, distort_coeffs;
    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> K(camera_matrix_data.data());
    Eigen::Matrix<double, 1, 5> D(distort_coeffs_data.data());
    cv::eigen2cv(K, camera_matrix);
    cv::eigen2cv(D, distort_coeffs);

    // 打印外参信息
    std::cout << "\n========== 外参验证 ==========" << std::endl;
    std::cout << "\nR_camera2gimbal (旋转矩阵):" << std::endl;
    std::cout << R_camera2gimbal << std::endl;
    
    std::cout << "\nt_camera2gimbal (平移向量, 单位:米):" << std::endl;
    std::cout << t_camera2gimbal.transpose() << std::endl;

    // 计算相机相对于云台的欧拉角
    Eigen::Vector3d euler = R_camera2gimbal.eulerAngles(2, 1, 0);  // ZYX顺序
    std::cout << "\n相机相对云台的姿态 (yaw, pitch, roll) [度]:" << std::endl;
    std::cout << "  yaw:   " << euler[0] * 57.3 << std::endl;
    std::cout << "  pitch: " << euler[1] * 57.3 << std::endl;
    std::cout << "  roll:  " << euler[2] * 57.3 << std::endl;

    // 检查旋转矩阵是否正交
    Eigen::Matrix3d I = R_camera2gimbal * R_camera2gimbal.transpose();
    double orthogonality_error = (I - Eigen::Matrix3d::Identity()).norm();
    std::cout << "\n旋转矩阵正交性误差: " << orthogonality_error << std::endl;
    if (orthogonality_error > 1e-6) {
        std::cout << "  ⚠️  警告：旋转矩阵不正交！外参可能有问题。" << std::endl;
    } else {
        std::cout << "  ✓ 旋转矩阵正交性良好" << std::endl;
    }

    // 检查行列式（应该为1）
    double det = R_camera2gimbal.determinant();
    std::cout << "\n旋转矩阵行列式: " << det << std::endl;
    if (std::abs(det - 1.0) > 1e-6) {
        std::cout << "  ⚠️  警告：行列式不为1！这不是有效的旋转矩阵。" << std::endl;
    } else {
        std::cout << "  ✓ 行列式检查通过" << std::endl;
    }

    std::cout << "\n========== 建议 ==========" << std::endl;
    std::cout << "1. 如果正交性误差 > 1e-6，需要重新标定" << std::endl;
    std::cout << "2. 如果装甲板倾斜时重投影误差明显增大，说明旋转矩阵不准确" << std::endl;
    std::cout << "3. 建议使用多组不同姿态的标定板数据重新进行手眼标定" << std::endl;
    std::cout << "4. 标定时确保：" << std::endl;
    std::cout << "   - 标定板姿态多样化（不同yaw/pitch/roll角度）" << std::endl;
    std::cout << "   - 标定板距离覆盖实际使用范围（1-8米）" << std::endl;
    std::cout << "   - 采集至少20-30组数据" << std::endl;
    std::cout << "==============================\n" << std::endl;

    return 0;
}
