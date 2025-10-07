
#include "ib2_ctl/att_profiler.h"

#include "ib2_ctl_common/Constants.h"
#include "ib2_ctl_common/RangeChecker.h"

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::AttProfiler::AttProfiler() :
rclcpp::Node("att_profile"),
Tmax_(0.), wmax_(0.), qthr_(ib2_mss::DEG),
epsQm_(1.e-9), aia_(Eigen::Quaterniond::Identity()), 
rda_(Eigen::Quaterniond::Identity()), scanAxes_(2, Eigen::Vector3d::UnitX())
{
}

//------------------------------------------------------------------------------
// rosparamによるコンストラクタ
ib2::AttProfiler::AttProfiler(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) :
rclcpp::Node("att_profile", options),
Tmax_(0.), wmax_(0.), qthr_(ib2_mss::DEG),
epsQm_(1.e-9), aia_(Eigen::Quaterniond::Identity()), 
rda_(Eigen::Quaterniond::Identity()), scanAxes_(3, Eigen::Vector3d::UnitX())
{ 
    using namespace ib2_mss;
    static const RangeCheckerD T_MAX_RANGE
    (RangeCheckerD::TYPE::GT_LE, 0., 0.02, true);
    static const RangeCheckerD W_MAX_RANGE
    (RangeCheckerD::TYPE::GT_LE, 0., 0.5236, true);
    static const RangeCheckerD EPS_RANGE
    (RangeCheckerD::TYPE::GT_LE, 0., 0.0002, true);

    static const std::string ROSPARAM_T_MAX("att_profile.t_max");
    static const std::string ROSPARAM_W_MAX("att_profile.w_max");
    static const std::string ROSPARAM_Q_THR("att_profile.theta_threshold");
    static const std::string ROSPARAM_EPS  ("att_profile.eps_qm");

    double Tmax(-1.);
    double wmax(-1.);
    double qthr(-1.);
    double eps_qm(-1.);
    Eigen::Quaterniond aia(0., 0., 0., 0.);
    Eigen::Quaterniond rda(0., 0., 0., 0.);
    std::vector<Eigen::Vector3d> scan(3, Eigen::Vector3d::Zero());

    // パラメータの宣言
    this->declare_parameter(ROSPARAM_T_MAX, Tmax);
    this->declare_parameter(ROSPARAM_W_MAX, wmax);
    this->declare_parameter(ROSPARAM_Q_THR, qthr);
    this->declare_parameter(ROSPARAM_EPS, eps_qm);
    this->declare_parameter("att_profile.aia.x", aia.x());
    this->declare_parameter("att_profile.aia.y", aia.y());
    this->declare_parameter("att_profile.aia.z", aia.z());
    this->declare_parameter("att_profile.aia.w", aia.w());
    this->declare_parameter("att_profile.rda.x", rda.x());
    this->declare_parameter("att_profile.rda.y", rda.y());
    this->declare_parameter("att_profile.rda.z", rda.z());
    this->declare_parameter("att_profile.rda.w", rda.w());
    this->declare_parameter("att_profile.scan.axis1.x", scan.at(0).x());
    this->declare_parameter("att_profile.scan.axis1.y", scan.at(0).y());
    this->declare_parameter("att_profile.scan.axis1.z", scan.at(0).z());
    this->declare_parameter("att_profile.scan.axis2.x", scan.at(1).x());
    this->declare_parameter("att_profile.scan.axis2.y", scan.at(1).y());
    this->declare_parameter("att_profile.scan.axis2.z", scan.at(1).z());
    this->declare_parameter("att_profile.scan.axis3.x", scan.at(2).x());
    this->declare_parameter("att_profile.scan.axis3.y", scan.at(2).y());
    this->declare_parameter("att_profile.scan.axis3.z", scan.at(2).z());

    // パラメータの取得
    Tmax    = this->get_parameter(ROSPARAM_T_MAX).as_double();
    wmax    = this->get_parameter(ROSPARAM_W_MAX).as_double();
    qthr    = this->get_parameter(ROSPARAM_Q_THR).as_double();
    eps_qm  = this->get_parameter(ROSPARAM_EPS).as_double();
    aia.x() = this->get_parameter("att_profile.aia.x").as_double();
    aia.y() = this->get_parameter("att_profile.aia.y").as_double();
    aia.z() = this->get_parameter("att_profile.aia.z").as_double();
    aia.w() = this->get_parameter("att_profile.aia.w").as_double();
    rda.x() = this->get_parameter("att_profile.rda.x").as_double();
    rda.y() = this->get_parameter("att_profile.rda.y").as_double();
    rda.z() = this->get_parameter("att_profile.rda.z").as_double();
    rda.w() = this->get_parameter("att_profile.rda.w").as_double();
    scan.at(0).x() = this->get_parameter("att_profile.scan.axis1.x").as_double();
    scan.at(0).y() = this->get_parameter("att_profile.scan.axis1.y").as_double();
    scan.at(0).z() = this->get_parameter("att_profile.scan.axis1.z").as_double();
    scan.at(1).x() = this->get_parameter("att_profile.scan.axis2.x").as_double();
    scan.at(1).y() = this->get_parameter("att_profile.scan.axis2.y").as_double();
    scan.at(1).z() = this->get_parameter("att_profile.scan.axis2.z").as_double();
    scan.at(2).x() = this->get_parameter("att_profile.scan.axis3.x").as_double();
    scan.at(2).y() = this->get_parameter("att_profile.scan.axis3.y").as_double();
    scan.at(2).z() = this->get_parameter("att_profile.scan.axis3.z").as_double();

    // パラメータのチェック
    T_MAX_RANGE.valid(Tmax, ROSPARAM_T_MAX);
    W_MAX_RANGE.valid(wmax, ROSPARAM_W_MAX);
    RangeCheckerD::notNegative(qthr, true, ROSPARAM_Q_THR);
    EPS_RANGE.valid(eps_qm , ROSPARAM_EPS);
    RangeCheckerD::positive(aia.norm(), true, "AIA quaternion magnitude");
    RangeCheckerD::positive(rda.norm(), true, "RDA quaternion magnitude");
    for (size_t i = 0; i < scan.size(); ++i)
    {
        std::ostringstream ss;
        ss << "scan axis " << i+1 << " magnitude";
        RangeCheckerD::positive(scan.at(i).norm(), true, ss.str());
        scan.at(i).normalize();
    }
    
    Tmax_ = Tmax;
    wmax_ = wmax;
    qthr_ = qthr;
    epsQm_ = eps_qm;
    aia_ = aia.normalized();
    rda_ = rda.normalized();
    scanAxes_ = scan;

    // パラメータの表示
    RCLCPP_INFO(this->get_logger(),
            "******** Set Parameters in att_profiler.cpp");
    RCLCPP_INFO(this->get_logger(), "att_profile.t_max           : %f", Tmax_);
    RCLCPP_INFO(this->get_logger(), "att_profile.w_max           : %f", wmax_);
    RCLCPP_INFO(this->get_logger(), "att_profile.theta_threshold : %f", qthr_);
    RCLCPP_INFO(this->get_logger(), "att_profile.eps_qm          : %f", epsQm_);
    RCLCPP_INFO(this->get_logger(), "att_profile.aia_x           : %f", aia_.x());
    RCLCPP_INFO(this->get_logger(), "att_profile.aia_y           : %f", aia_.y());
    RCLCPP_INFO(this->get_logger(), "att_profile.aia_z           : %f", aia_.z());
    RCLCPP_INFO(this->get_logger(), "att_profile.aia_w           : %f", aia_.w());
    RCLCPP_INFO(this->get_logger(), "att_profile.rda_x           : %f", rda_.x());
    RCLCPP_INFO(this->get_logger(), "att_profile.rda_y           : %f", rda_.y());
    RCLCPP_INFO(this->get_logger(), "att_profile.rda_z           : %f", rda_.z());
    RCLCPP_INFO(this->get_logger(), "att_profile.rda_w           : %f", rda_.w());
    RCLCPP_INFO(this->get_logger(), "att_profile.scan.axis1.x    : %f", scanAxes_.at(0).x());
    RCLCPP_INFO(this->get_logger(), "att_profile.scan.axis1.y    : %f", scanAxes_.at(0).y());
    RCLCPP_INFO(this->get_logger(), "att_profile.scan.axis1.z    : %f", scanAxes_.at(0).z());
    RCLCPP_INFO(this->get_logger(), "att_profile.scan.axis2.x    : %f", scanAxes_.at(1).x());
    RCLCPP_INFO(this->get_logger(), "att_profile.scan.axis2.y    : %f", scanAxes_.at(1).y());
    RCLCPP_INFO(this->get_logger(), "att_profile.scan.axis2.z    : %f", scanAxes_.at(1).z());
    RCLCPP_INFO(this->get_logger(), "att_profile.scan.axis3.x    : %f", scanAxes_.at(2).x());
    RCLCPP_INFO(this->get_logger(), "att_profile.scan.axis3.y    : %f", scanAxes_.at(2).y());
    RCLCPP_INFO(this->get_logger(), "att_profile.scan.axis3.z    : %f", scanAxes_.at(2).z());
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::AttProfiler::~AttProfiler() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
// ib2::AttProfiler::AttProfiler
// (const AttProfiler&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
// ib2::AttProfiler&
// ib2::AttProfiler::operator=(const AttProfiler&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
// ib2::AttProfiler::AttProfiler(AttProfiler&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
// ib2::AttProfiler&
// ib2::AttProfiler::operator=(AttProfiler&&) = default;

//------------------------------------------------------------------------------
//  姿勢プロファイル最大トルクの取得
double ib2::AttProfiler::Tmax() const
{
    return Tmax_;
}
//------------------------------------------------------------------------------
//  姿勢プロファイル最大角速度の取得
double ib2::AttProfiler::wmax() const
{
    return wmax_;
}

//------------------------------------------------------------------------------
//  角速度制御を切る閾値の取得
double ib2::AttProfiler::qthr() const
{
    return qthr_;
}

//------------------------------------------------------------------------------
// 回転角微小数の取得
double ib2::AttProfiler::epsQm() const
{
    return epsQm_;
}

//------------------------------------------------------------------------------
// AIAの参照
const Eigen::Quaterniond& ib2::AttProfiler::aia() const
{
    return aia_;
}

//------------------------------------------------------------------------------
// RDAの参照
const Eigen::Quaterniond& ib2::AttProfiler::rda() const
{
    return rda_;
}

//------------------------------------------------------------------------------
// スキャン軸の参照
const std::vector<Eigen::Vector3d>& ib2::AttProfiler::scanAxes() const
{
    return scanAxes_;
}

// End Of File -----------------------------------------------------------------
