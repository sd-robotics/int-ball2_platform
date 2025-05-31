
#include "ib2_ctl/ctl_body.h"
#include "guidance_control_common/RangeChecker.h"

#include <string>

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
ib2::CtlBody::CtlBody() :
rclcpp::Node("ctl_body"),
m_(1.), Is_(Eigen::Matrix3d::Identity())
{
}

//------------------------------------------------------------------------------
// rosparamによるコンストラクタ
ib2::CtlBody::CtlBody(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) :
rclcpp::Node("ctl_body", options),
m_(1.), Is_(Eigen::Matrix3d::Identity())
{
    using namespace ib2_mss;
    
    static const std::string ROSPARAM_MASS ("mass");
    static const std::string ROSPARAM_IS_XX("Is/xx");
    static const std::string ROSPARAM_IS_YY("Is/yy");
    static const std::string ROSPARAM_IS_ZZ("Is/zz");
    static const std::string ROSPARAM_IS_XY("Is/xy");
    static const std::string ROSPARAM_IS_YZ("Is/yz");
    static const std::string ROSPARAM_IS_ZX("Is/zx");
    
    double mass(-1.);
    double Is_xx(-1.);
    double Is_yy(-1.);
    double Is_zz(-1.);
    double Is_xy(0.);
    double Is_yz(0.);
    double Is_zx(0.);
    
    // パラメータの宣言
    this->declare_parameter(ROSPARAM_MASS, mass);
    this->declare_parameter(ROSPARAM_IS_XX, Is_xx);
    this->declare_parameter(ROSPARAM_IS_YY, Is_yy);
    this->declare_parameter(ROSPARAM_IS_ZZ, Is_zz);
    this->declare_parameter(ROSPARAM_IS_XY, Is_xy);
    this->declare_parameter(ROSPARAM_IS_YZ, Is_yz);
    this->declare_parameter(ROSPARAM_IS_ZX, Is_zx);

    // パラメータの取得
    mass  = this->get_parameter(ROSPARAM_MASS).as_double();
    Is_xx = this->get_parameter(ROSPARAM_MASS).as_double();
    Is_yy = this->get_parameter(ROSPARAM_MASS).as_double();
    Is_zz = this->get_parameter(ROSPARAM_MASS).as_double();
    Is_xy = this->get_parameter(ROSPARAM_MASS).as_double();
    Is_yz = this->get_parameter(ROSPARAM_MASS).as_double();
    Is_zx = this->get_parameter(ROSPARAM_MASS).as_double();
    
    // パラメータのチェック
    RangeCheckerD::positive(mass , true, ROSPARAM_MASS);
    RangeCheckerD::positive(Is_xx, true, ROSPARAM_IS_XX);
    RangeCheckerD::positive(Is_yy, true, ROSPARAM_IS_YY);
    RangeCheckerD::positive(Is_zz, true, ROSPARAM_IS_ZZ);
    
    m_ = mass;
    Is_(0,0) = Is_xx;
    Is_(1,1) = Is_yy;
    Is_(2,2) = Is_zz;
    Is_(0,1) = Is_xy;
    Is_(1,2) = Is_yz;
    Is_(2,0) = Is_zx;
    Is_(1,0) = Is_(0,1);
    Is_(2,1) = Is_(1,2);
    Is_(0,2) = Is_(2,0);

    // パラメータの表示
    RCLCPP_INFO(this->get_logger(), "******** Set Parameters in ctl_body.cpp");
    RCLCPP_INFO(this->get_logger(), "%s   : %f", ROSPARAM_MASS.c_str(),  m_);
    RCLCPP_INFO(this->get_logger(), "%s   : %f", ROSPARAM_IS_XX.c_str(), Is_(0, 0));
    RCLCPP_INFO(this->get_logger(), "%s   : %f", ROSPARAM_IS_YY.c_str(), Is_(1, 1));
    RCLCPP_INFO(this->get_logger(), "%s   : %f", ROSPARAM_IS_ZZ.c_str(), Is_(2, 2));
    RCLCPP_INFO(this->get_logger(), "%s   : %f", ROSPARAM_IS_XY.c_str(), Is_(0, 1));
    RCLCPP_INFO(this->get_logger(), "%s   : %f", ROSPARAM_IS_YZ.c_str(), Is_(1, 2));
    RCLCPP_INFO(this->get_logger(), "%s   : %f", ROSPARAM_IS_ZX.c_str(), Is_(2, 0));
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::CtlBody::~CtlBody() = default;

//------------------------------------------------------------------------------
// コピーコンストラクタ
ib2::CtlBody::CtlBody(const CtlBody&) = default;

//------------------------------------------------------------------------------
// コピー代入演算子
ib2::CtlBody& ib2::CtlBody::operator=(const CtlBody&) = default;

//------------------------------------------------------------------------------
// ムーブコンストラクタ
ib2::CtlBody::CtlBody(CtlBody&&) = default;

//------------------------------------------------------------------------------
// ムーブ代入演算子
ib2::CtlBody& ib2::CtlBody::operator=(CtlBody&&) = default;

//------------------------------------------------------------------------------
//  機体質量の取得
double ib2::CtlBody::m() const
{
    return m_;
}

//------------------------------------------------------------------------------
//  質量特性行列の参照
const Eigen::Matrix3d& ib2::CtlBody::Is() const
{
    return Is_;
}

// End Of File -----------------------------------------------------------------
