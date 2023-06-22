// Eigen 核心部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
// 几何模块
#include <Eigen/Geometry>
#include <ctime>
#include <iostream>
#include <opencv2/opencv.hpp>

using std::cout;
using std::endl;
using namespace Eigen;


//练习EIGEN库的
int main(int argc,char **argv)
{
    cout <<__LINE__<<"  :" << "start" << endl;
    //3D 旋转矩阵
    Matrix3d rotation_matrix = Matrix3d::Identity();
    cout <<__LINE__<<"  :" << rotation_matrix<<endl;
    AngleAxisd rotation_vector(M_PI / 4,Vector3d(0,0,1));
    cout.precision(3); //浮点数的精度
    cout <<__LINE__<<"  :" << rotation_vector.matrix()<<endl;

    //给矩阵赋值
    rotation_matrix = rotation_vector.matrix();
    cout <<__LINE__<<":rotation_matrix\n" << rotation_matrix<<endl;
 
    Vector3d v(1,0,0);
    Vector3d v_rotated = rotation_vector *v;
    cout <<__LINE__<<": "<< v_rotated.transpose()<<endl;

    // 欧拉角: 可以将旋转矩阵直接转换成欧拉角
    Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0); // ZYX顺序，即yaw-pitch-roll顺序
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

    //欧式变换
    Isometry3d T = Isometry3d::Identity();
    T.rotate(rotation_vector);
    T.pretranslate(Vector3d(1,3,4));
    cout << "Transform = \n" << T.matrix() << endl;
    
    Vector3d v_transformed  = T * v;
    cout << "v transformed = " << v_transformed.transpose() << endl;

    //四元数
    Quaterniond q = Quaterniond(rotation_vector);
    cout << "quaternion from rotation matrix = " << q.coeffs().transpose();//coeffs代表系数

    return 0;
}