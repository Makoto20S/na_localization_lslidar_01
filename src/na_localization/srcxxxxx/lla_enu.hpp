#ifndef LLA_ENU_HPP
#define LLA_ENU_HPP

// cpp
#include <stdio.h>
#include <iostream>
#include <cmath>
//eigen
#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>

class LLAENU {
  public:
	/** 
	 * @brief 默认构造函数
	*/
    LLAENU();
    /**
     * @brief 构造函数,设置原点经纬高
     * @param longitude[in] 经度(rad)
     * @param latitude[in] 纬度(rad)
     * @param altitude[in] 高度(m)
     * @return void
    */
    LLAENU(double longitude, double latitude, double altitude = 0.0);
    /**
     * @brief 参数初始化
    */
    void Initialize();
    /**
     * @brief 设置原点经纬高
     * @param longitude[in] 经度(rad)
     * @param latitude[in] 纬度(rad)
     * @param altitude[in] 高度(m)
     * @return void
    */
    void SetOriginLLA(double longitude, double latitude, double altitude = 0.0);
    inline double RadToDeg(double rad) {
        return rad / M_PI * 180.0;
    }
    /**
     * @brief 经纬高转换为局部东北天系坐标
     * @param longitude[in] 经度(rad)
     * @param latitude[in] 纬度(rad)
     * @param altitude[in] 高度(m)
     * @return 局部东北天系坐标
    */
    inline Eigen::Vector3d LLAToENU(double longitude, double latitude, double altitude = 0.0);
    /**
     * @brief 局部东北天系坐标转换为经纬度
     * @param local_e[in] 局部东北天系x坐标
     * @param local_n[in] 局部东北天系y坐标
     * @param local_u[in] 局部东北天系z坐标
     * @return 经纬高
    */
    inline Eigen::Vector3d ENUToLLA(double local_e, double local_n, double local_u = 0.0);

    // 地球参数
    double Re_ = 6378137.0; // 地球半径
    double f_ = 1/298.257; // 偏心率
    double local_Rn_, local_Rm_; // 原点处子午、卯酉圈半径

    // 经度(rad), 纬度(rad), 高度(m)
    double longitude_, latitude_, altitude_;
    bool origin_lla_initialized_ = false;
    double ratio_x_, ratio_y_, inv_ratio_x_, inv_ratio_y_;
};

LLAENU::LLAENU() {
    std::cout << "  -- LLAENU Construct --  \n";
}

void LLAENU::Initialize() {
    local_Rn_ = Re_ * (1. + f_ * std::sin(latitude_) * std::sin(latitude_));
    local_Rm_ = Re_ * (1. - 2. * f_ + 3. * f_ * std::sin(latitude_) * std::sin(latitude_));
    ratio_x_ = (local_Rn_ + altitude_) * std::cos(latitude_);
    ratio_y_ = local_Rm_ + altitude_;
	inv_ratio_x_ = 1. / ratio_x_;
	inv_ratio_y_ = 1. / ratio_y_;
    origin_lla_initialized_ = true;
}

LLAENU::LLAENU(
    double longitude, double latitude, double altitude): 
    longitude_(longitude), latitude_(latitude), altitude_(altitude) {
    Initialize();
    std::cout << "  -- LLAENU Construct --  \n";
    printf("     longitude_: %3.7lf; longitude_: %3.7lf; altitude_: %3.3lf; \n", 
        RadToDeg(longitude_), RadToDeg(latitude_), altitude_);
}

void LLAENU::SetOriginLLA(double longitude, double latitude, double altitude) {
    longitude_ = longitude;
    latitude_ = latitude;
    altitude_ = altitude;
    Initialize();
}

Eigen::Vector3d LLAENU::LLAToENU(double longitude, double latitude, double altitude) {
    return Eigen::Vector3d{
        (longitude - longitude_) * ratio_x_,
        (latitude - latitude_) * ratio_y_,
        altitude - altitude_
    };
}

Eigen::Vector3d LLAENU::ENUToLLA(double local_e, double local_n, double local_u) {
	return Eigen::Vector3d{
        longitude_ + local_e * inv_ratio_x_,
        latitude_ + local_n * inv_ratio_y_,
        altitude_ + local_u
    };
}

#endif