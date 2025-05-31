

#include "ib2_fsm/fsm.h"
#include "guidance_control_common/RangeChecker.h"

#include <Eigen/Core>

#include <cassert>

#include <numeric>

// マクロ宣言、後でどこか共通の*.hに移動したほうがいいかも
#define TOPIC_CTL_DUTY    "/ctl/duty"
#define TOPIC_CTL_WRENCH  "/ctl/wrench"    // Modification for platform packages


//------------------------------------------------------------------------------
// ファイルスコープ
namespace
{
}

//------------------------------------------------------------------------------
// デフォルトコンストラクタ
Fsm::Fsm(const rclcpp::NodeOptions& options) :
    rclcpp::Node("fsm", options)
{
    // 初期化　
    ib2::ThrustAllocator thr(options);
    setMember(thr);

    // TODO: configure QoS

    // Subscriber
    wrench_sub_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        TOPIC_CTL_WRENCH, 5,
        std::bind(&Fsm::wrenchCallback, this, std::placeholders::_1)); // Modification for platform packages

    // Advertised messages　（デューティ）
    pub_duty_   = this->create_publisher<example_interfaces::msg::Float64MultiArray>(
        TOPIC_CTL_DUTY, 5);
}

//------------------------------------------------------------------------------
// デストラクタ
Fsm::~Fsm() = default;

//------------------------------------------------------------------------------
// メンバ設定
bool Fsm::setMember(const ib2::ThrustAllocator& thr)
{
    using namespace ib2_mss;
    static const RangeCheckerD PWM_RANGE
    (RangeCheckerD::TYPE::GT_LE, 0., 1., true);
    static const RangeCheckerI32 N_FAN_RANGE
    (RangeCheckerI32::TYPE::GE_LE, 0, 8, true);
    RangeCheckerD fj0_range
    (RangeCheckerD::TYPE::GE_LE, 0., thr.Fmax(), true);

    static const std::string ROSPARAM_PWM_MAX     ("/fan/PWMmax");
    static const std::string ROSPARAM_N_SATURATION("/fan/n_saturation");

    bool ret = true;
    int nfan(thr.nfan());
    double PWMmax(-1.);
    int nsaturation(-1);
    Eigen::VectorXd fan_kj(Eigen::VectorXd::Ones(nfan) * -1.);
    Eigen::VectorXd fj0(fan_kj);
    
    this->declare_parameter(ROSPARAM_PWM_MAX, PWMmax);
    this->declare_parameter(ROSPARAM_N_SATURATION, nsaturation);
    this->declare_parameter("/fan/kj/fan01", fan_kj(0));
    this->declare_parameter("/fan/kj/fan02", fan_kj(1));
    this->declare_parameter("/fan/kj/fan03", fan_kj(2));
    this->declare_parameter("/fan/kj/fan04", fan_kj(3));
    this->declare_parameter("/fan/kj/fan05", fan_kj(4));
    this->declare_parameter("/fan/kj/fan06", fan_kj(5));
    this->declare_parameter("/fan/kj/fan07", fan_kj(6));
    this->declare_parameter("/fan/kj/fan08", fan_kj(7));
    this->declare_parameter("/fan/fj0/fan01", fj0(0));
    this->declare_parameter("/fan/fj0/fan02", fj0(1));
    this->declare_parameter("/fan/fj0/fan03", fj0(2));
    this->declare_parameter("/fan/fj0/fan04", fj0(3));
    this->declare_parameter("/fan/fj0/fan05", fj0(4));
    this->declare_parameter("/fan/fj0/fan06", fj0(5));
    this->declare_parameter("/fan/fj0/fan07", fj0(6));
    this->declare_parameter("/fan/fj0/fan08", fj0(7));

    PWMmax = this->get_parameter(ROSPARAM_PWM_MAX).as_double();
    nsaturation = this->get_parameter(ROSPARAM_N_SATURATION).as_int();
    fan_kj(0) = this->get_parameter("/fan/kj/fan01").as_double();
    fan_kj(1) = this->get_parameter("/fan/kj/fan02").as_double();
    fan_kj(2) = this->get_parameter("/fan/kj/fan03").as_double();
    fan_kj(3) = this->get_parameter("/fan/kj/fan04").as_double();
    fan_kj(4) = this->get_parameter("/fan/kj/fan05").as_double();
    fan_kj(5) = this->get_parameter("/fan/kj/fan06").as_double();
    fan_kj(6) = this->get_parameter("/fan/kj/fan07").as_double();
    fan_kj(7) = this->get_parameter("/fan/kj/fan08").as_double();
    fj0(0) = this->get_parameter("/fan/fj0/fan01").as_double();
    fj0(1) = this->get_parameter("/fan/fj0/fan02").as_double();
    fj0(2) = this->get_parameter("/fan/fj0/fan03").as_double();
    fj0(3) = this->get_parameter("/fan/fj0/fan04").as_double();
    fj0(4) = this->get_parameter("/fan/fj0/fan05").as_double();
    fj0(5) = this->get_parameter("/fan/fj0/fan06").as_double();
    fj0(6) = this->get_parameter("/fan/fj0/fan07").as_double();
    fj0(7) = this->get_parameter("/fan/fj0/fan08").as_double();

    // TODO: パラメータの確認
    ret = ret && PWMmax;
    ret = ret && nsaturation;
    ret = ret && fan_kj(0);
    ret = ret && fan_kj(1);
    ret = ret && fan_kj(2);
    ret = ret && fan_kj(3);
    ret = ret && fan_kj(4);
    ret = ret && fan_kj(5);
    ret = ret && fan_kj(6);
    ret = ret && fan_kj(7);
    ret = ret && fj0(0);
    ret = ret && fj0(1);
    ret = ret && fj0(2);
    ret = ret && fj0(3);
    ret = ret && fj0(4);
    ret = ret && fj0(5);
    ret = ret && fj0(6);
    ret = ret && fj0(7);

    ret = ret && PWM_RANGE.valid(PWMmax, ROSPARAM_PWM_MAX);
    ret = ret && N_FAN_RANGE.valid(nsaturation, ROSPARAM_N_SATURATION);
    ret = ret && fj0_range.valid(fj0.minCoeff(), "minimum fj0");
    ret = ret && fj0_range.valid(fj0.maxCoeff(), "maximum fj0");
    ret = ret && RangeCheckerD::notNegative(fan_kj.minCoeff(), false, "minimum fan kj");
    
    thr_  = thr;
    nfan_ = nfan;
    pwm_max_     = Eigen::VectorXd::Ones(nfan_) * PWMmax;
    nsaturation_ = nsaturation;
    fan_kj_ = fan_kj;
    fj0_    = fj0;

    RCLCPP_INFO(this->get_logger(), "******** Set Parameters in fsm.cpp");
    RCLCPP_INFO(this->get_logger(), "%s   : %f", ROSPARAM_PWM_MAX.c_str(), pwm_max_(0));
    RCLCPP_INFO(this->get_logger(), "%s   : %d", ROSPARAM_N_SATURATION.c_str(), nsaturation_);
    RCLCPP_INFO(this->get_logger(), "/fan/kj/fan01   : %f", fan_kj_(0));
    RCLCPP_INFO(this->get_logger(), "/fan/kj/fan02   : %f", fan_kj_(1));
    RCLCPP_INFO(this->get_logger(), "/fan/kj/fan03   : %f", fan_kj_(2));
    RCLCPP_INFO(this->get_logger(), "/fan/kj/fan04   : %f", fan_kj_(3));
    RCLCPP_INFO(this->get_logger(), "/fan/kj/fan05   : %f", fan_kj_(4));
    RCLCPP_INFO(this->get_logger(), "/fan/kj/fan06   : %f", fan_kj_(5));
    RCLCPP_INFO(this->get_logger(), "/fan/kj/fan07   : %f", fan_kj_(6));
    RCLCPP_INFO(this->get_logger(), "/fan/kj/fan08   : %f", fan_kj_(7));
    RCLCPP_INFO(this->get_logger(), "/fan/fj0/fan01  : %f", fj0_(0));
    RCLCPP_INFO(this->get_logger(), "/fan/fj0/fan02  : %f", fj0_(1));
    RCLCPP_INFO(this->get_logger(), "/fan/fj0/fan03  : %f", fj0_(2));
    RCLCPP_INFO(this->get_logger(), "/fan/fj0/fan04  : %f", fj0_(3));
    RCLCPP_INFO(this->get_logger(), "/fan/fj0/fan05  : %f", fj0_(4));
    RCLCPP_INFO(this->get_logger(), "/fan/fj0/fan06  : %f", fj0_(5));
    RCLCPP_INFO(this->get_logger(), "/fan/fj0/fan07  : %f", fj0_(6));
    RCLCPP_INFO(this->get_logger(), "/fan/fj0/fan08  : %f", fj0_(7));

    if(!ret)
        RCLCPP_ERROR(this->get_logger(), "Parameter Setting Error in fsm.cpp");

    return ret;
}

//------------------------------------------------------------------------------
// 制御推力トルクのサブスクライバのコールバック関数
//void Fsm::subscribeCommand    // Modification for platform packages
void Fsm::wrenchCallback        // Modification for platform packages
(const geometry_msgs::msg::WrenchStamped& wrench_command) const
{
    // 力トルクコマンド格納
    auto& f(wrench_command.wrench.force);
    auto& t(wrench_command.wrench.torque);
    Eigen::Vector3d Fcmd(f.x, f.y, f.z);
    Eigen::Vector3d Tcmd(t.x, t.y, t.z);

    // ファン推進制御力配分計算
    Eigen::VectorXd fj(thr_.allocate(Fcmd, Tcmd) + fj0_);
    Eigen::VectorXd pwm(fan_kj_.array() * fj.array().abs().sqrt());
    saturation(pwm);

    // 各ファンの駆動デューティ比のPublish
    publishDuty(pwm);
}

//------------------------------------------------------------------------------
// 各ファンの駆動デューティ比のPublish
void Fsm::publishDuty(const Eigen::VectorXd& pwm) const
{
    assert(pwm.size() == nfan_);

    example_interfaces::msg::Float64MultiArray msg;
    msg.layout.dim.push_back(example_interfaces::msg::MultiArrayDimension());
    msg.layout.dim[0].size = nfan_;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "fan_duty";
    msg.layout.data_offset = 0;
    msg.data.resize(nfan_, 0.);
    Eigen::Map<Eigen::VectorXd>(msg.data.data(), nfan_) = pwm;

    pub_duty_->publish(msg);
}

//------------------------------------------------------------------------------
// ファン推進力飽和時処理
void Fsm::saturation(Eigen::VectorXd& pwm) const
{
    // 上位 nsaturation_個のファンに最大値を設定
    if ((pwm.array() > pwm_max_.array()).count() >= nsaturation_)
    {
        // 降順にソート
        std::vector<int> index(pwm.size());
        std::iota(index.begin(), index.end(), 0);
        std::sort(index.begin(), index.end(),
              [&](int x, int y){return pwm(x) > pwm(y); });

         // 上位 nsaturation_個のファンにPWM最大値を設定＆それ以外はOFF
        for (auto itr = index.begin(); itr != index.end(); ++itr)
            pwm(*itr) = itr-index.begin() < nsaturation_ ? pwm_max_(*itr) : 0.;
    }
    else
    {
        for (int i = 0; i < pwm.size(); i++)
        {
            if (pwm(i) > pwm_max_(i))
                pwm(i) = pwm_max_(i);
        }
    }
}

// // Modification for platform packages
// メイン関数
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Fsm>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// End Of File -----------------------------------------------------------------
