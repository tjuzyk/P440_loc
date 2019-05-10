#ifndef TRIANGULATION_ERROR_TERM_HPP
#define TRIANGULATION_ERROR_TERM_HPP

#include "Eigen/Core"
#include "ceres/ceres.h"
#include <iostream>

namespace uavos{


class TriangulationErrorTerm{
public:
    TriangulationErrorTerm(const double r_ab, const double sqrt_information):  // 中位数 标准差
        m_r_ab(r_ab), m_sqrt_information(sqrt_information){}         //令 m_r_ab为中位数， m_sqrt_information为标准差

    template <typename T>
    bool operator()(const T* const x_a, const T* const y_a, const T* const z_a,
                    const T* const x_b, const T* const y_b, const T* const z_b,
                    T* residuals_ptr) const{
        const Eigen::Matrix<T, 3, 1> p_a(*x_a, *y_a, *z_a); // *是取值
        const Eigen::Matrix<T, 3, 1> p_b(*x_b, *y_b, *z_b);
        const double sqrt_1 = 1;
        Eigen::Matrix<T, 3, 1> diff = p_a - p_b;  //根据参数服务器上的坐标  anchor到tag的矢量 norm()为矢量的模即距离
        if(diff(0,0)==T(0) && diff(1,0)==T(0) && diff(2,0)==T(0)){
            diff(0,0) = T(1e-6);
        }

        residuals_ptr[0] = ceres::abs( diff.norm() - T(m_r_ab) );  // 两点的距离（参数服务器上的数据） - 测量的中位数 后取绝对值   即残差
                                                                   // 故在校准时飞机应保持静止
        if(m_sqrt_information != 0){
           double m_sqrt_information_reciprocal = 1.0/m_sqrt_information;
        // consider the sqrt_information, i.e. 1/sqrt(covariance)
           residuals_ptr[0] = residuals_ptr[0] * T(m_sqrt_information_reciprocal);  //问题在这，分母（标准差）不能为0
        }
        // avoid zero derivate for sqrt()
        const T threshold = T(1e-6);
        if(residuals_ptr[0]<threshold){
            residuals_ptr[0] = threshold;
        }

    /*  std::cout<<*x_a<<", "<<*y_a<<", "<<*z_a<<std::endl;
        std::cout<<*x_b<<", "<<*y_b<<", "<<*z_b<<std::endl;

        T distance = ceres::sqrt( (*x_a-*x_b)*(*x_a-*x_b) +
                                  (*y_a-*y_b)*(*y_a-*y_b) +
                                  (*z_a-*z_b)*(*z_a-*z_b) );    //两点之间的距离
        residuals_ptr[0] = ceres::abs( T(m_r_ab) - distance );

        std::cout<<"distance: "<<distance<<", r_ab: "<<m_r_ab<<std::endl;
        std::cout<<residuals_ptr[0]<<std::endl;
        std::cout<<"-------"<<std::endl;   */

        return true;

    }

    static ceres::CostFunction* Create(const double r_ab, const double sqrt_information){ //  r_ab为距离的中位数  sqrt_information为距离的标准差
        return (new ceres::AutoDiffCostFunction<TriangulationErrorTerm, 1, 1, 1, 1, 1, 1, 1>( new TriangulationErrorTerm(r_ab, sqrt_information) ) );
//        return (new ceres::NumericDiffCostFunction<TriangulationErrorTerm, ceres::CENTRAL, 1, 1, 1, 1, 1, 1, 1>( new TriangulationErrorTerm(r_ab, sqrt_information) ) );
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const double m_r_ab;
    const double m_sqrt_information;

};


class TriangulationErrorTerm2D{
public:
    TriangulationErrorTerm2D(const double r_ab, const double sqrt_information):
        m_r_ab(r_ab), m_sqrt_information(sqrt_information){}

    template <typename T>
    bool operator()(const T* const x_a, const T* const y_a,
                    const T* const x_b, const T* const y_b,
                    T* residuals_ptr) const{
        const Eigen::Matrix<T, 2, 1> p_a(*x_a, *y_a);
        const Eigen::Matrix<T, 2, 1> p_b(*x_b, *y_b);
        const double sqrt_1 = 1;
        Eigen::Matrix<T, 2, 1> diff = p_a - p_b;
        if(diff(0,0)==T(0) && diff(1,0)==T(0)){
            diff(0,0) = T(1e-6);
        }

        residuals_ptr[0] = ceres::abs( diff.norm() - T(m_r_ab) );
        if(m_sqrt_information != 0){
           double m_sqrt_information_reciprocal = 1.0/m_sqrt_information;
        // consider the sqrt_information, i.e. 1/sqrt(covariance)
           residuals_ptr[0] = residuals_ptr[0] * T(m_sqrt_information_reciprocal);
        }
        // avoid zero derivate for sqrt()
        const T threshold = T(1e-6);
        if(residuals_ptr[0]<threshold){
            residuals_ptr[0] = threshold;
        }

    /*  std::cout<<*x_a<<", "<<*y_a<<", "<<*z_a<<std::endl;
        std::cout<<*x_b<<", "<<*y_b<<", "<<*z_b<<std::endl;

        T distance = ceres::sqrt( (*x_a-*x_b)*(*x_a-*x_b) +
                                  (*y_a-*y_b)*(*y_a-*y_b) +
                                  (*z_a-*z_b)*(*z_a-*z_b) );
        residuals_ptr[0] = ceres::abs( T(m_r_ab) - distance );

        std::cout<<"distance: "<<distance<<", r_ab: "<<m_r_ab<<std::endl;
        std::cout<<residuals_ptr[0]<<std::endl;
        std::cout<<"-------"<<std::endl;   */

        return true;

    }

    static ceres::CostFunction* Create2D(const double r_ab,const double sqrt_information){
        return (new ceres::AutoDiffCostFunction<TriangulationErrorTerm2D, 1, 1, 1, 1, 1>( new TriangulationErrorTerm2D(r_ab, sqrt_information) ) );
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    const double m_r_ab;
    const double m_sqrt_information;

};


}


#endif // TRIANGULATION_ERROR_TERM_HPP
