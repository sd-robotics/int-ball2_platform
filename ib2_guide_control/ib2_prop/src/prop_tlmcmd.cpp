
#include "ib2_prop/prop_tlmcmd.h"

//------------------------------------------------------------------------------
// コンストラクタ
ib2::PropTlmCmd::PropTlmCmd(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) :
	rclcpp::Node("prop_tlmcmd", options)
{
	this->declare_parameter<int32_t>("/prop/fan_number", 0);
	fan_num_ = this->get_parameter("/prop/fan_number").as_int();

	// Duty初期化
	fan_duty_.layout.dim.push_back(example_interfaces::msg::MultiArrayDimension());
	fan_duty_.layout.dim[0].size   = fan_num_;
	fan_duty_.layout.dim[0].stride = 1;
	fan_duty_.layout.dim[0].label  = "";
	fan_duty_.layout.data_offset   = 0;
	fan_duty_.data.resize(fan_num_, 0.0);

	// ファン駆動状態メッセージ初期化
	fan_status_.duty.layout.dim.push_back(example_interfaces::msg::MultiArrayDimension());
	fan_status_.duty.data.resize(fan_num_, 0.0);
	initFanStatus(ib2_interfaces::msg::PowerStatus::ON);
	
	// ファン駆動デューティ比サブスクライバ
	sub_fan_duty_ = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
		TOPIC_CTL_DUTY, 1,
		std::bind(&ib2::PropTlmCmd::subFanDuty, this, std::placeholders::_1));

	// ファン駆動状態パブリッシャ
	pub_fan_status_ = this->create_publisher<ib2_interfaces::msg::FanStatus>(TOPIC_PROP_STATUS, 1);

	// ファン駆動モード設定サービスサーバ
	switch_power_server_ = this->create_service<ib2_interfaces::srv::SwitchPower>(
		SERVICE_SWITCH_POWER,
		std::bind(&ib2::PropTlmCmd::switchPower, this, std::placeholders::_1, std::placeholders::_2));
}

//------------------------------------------------------------------------------
// デストラクタ
ib2::PropTlmCmd::~PropTlmCmd() = default;


//------------------------------------------------------------------------------
// ファン駆動デューティ比をサブスクライブ
void ib2::PropTlmCmd::subFanDuty(const example_interfaces::msg::Float64MultiArray& msg)
{
	// データサイズチェック
	int size = msg.data.size();
	if(size != fan_num_)
	{
		RCLCPP_ERROR(this->get_logger(), "Prop Node Subscribed Invalid Size(%d) of Fan Duty", size);
		initFanStatus(ib2_interfaces::msg::PowerStatus::UNKNOWN);
		return;
	}

	fan_duty_ = msg;

	generateFanStatus();
}

//------------------------------------------------------------------------------
// ファン駆動状態メッセージ生成
void ib2::PropTlmCmd::generateFanStatus()
{
	initFanStatus(fan_status_.current_power.status);

	// 推進機能が停止の場合は、デューティ比 = 0とする
	if(fan_status_.current_power.status != ib2_interfaces::msg::PowerStatus::ON)
	{
		return;
	}

	// 推進機能が起動の場合、サブスクライブしたデューティ比を設定する
	for(int i = 0; i < fan_num_; i++)
	{
		fan_status_.duty.data[i] = limitter(fan_duty_.data[i], DUTY_MIN, DUTY_MAX);
	}
}

//------------------------------------------------------------------------------
// ファン駆動状態をパブリッシュ
void ib2::PropTlmCmd::pubFanStatus()
{
	pub_fan_status_->publish(fan_status_);
}

//------------------------------------------------------------------------------
// ファン駆動状態(異常停止中)をパブリッシュ
void ib2::PropTlmCmd::pubErrorFanStatus()
{
	initFanStatus(ib2_interfaces::msg::PowerStatus::UNKNOWN);
	pubFanStatus();
}

//------------------------------------------------------------------------------
// 推進機能起動/停止
bool ib2::PropTlmCmd::switchPower(
        const std::shared_ptr<ib2_interfaces::srv::SwitchPower::Request> req,
        std::shared_ptr<ib2_interfaces::srv::SwitchPower::Response> res)
{
	fan_status_.current_power.status = req->power.status;
	res->current_power.status  = fan_status_.current_power.status;

	if(req->power.status != ib2_interfaces::msg::PowerStatus::ON)
	{
		for(int i = 0; i < fan_num_; i++)
		{
			fan_duty_.data[i] = 0.0;
		}
	}
	
	return true;
}

//------------------------------------------------------------------------------
// ファン駆動デューティ比のgetter
std::vector<float> ib2::PropTlmCmd::getFanDuty()
{
	std::vector<float> duty(fan_num_, 0.0);

	for(int i = 0; i < fan_num_; i++)
	{
		duty[i] = fan_status_.duty.data[i];
	}

	// PWM制御ボードに送信するデューティで、ファン駆動状態メッセージを更新
	generateFanStatus();

	return duty;
}

//------------------------------------------------------------------------------
// ファン駆動状態初期化
void ib2::PropTlmCmd::initFanStatus(const uint8_t& status)
{
	fan_status_.header.stamp               = this->get_clock()->now();
	fan_status_.header.frame_id            = "";

	fan_status_.duty.layout.dim[0].size    = fan_num_;
	fan_status_.duty.layout.dim[0].stride  = 1;
	fan_status_.duty.layout.dim[0].label   = "";
	fan_status_.duty.layout.data_offset    = 0;

	for(int i = 0; i < fan_num_; i++)
	{
		fan_status_.duty.data[i]   = 0.0;
	}

	fan_status_.current_power.status = status;
}

//------------------------------------------------------------------------------
// テレメトリ・コマンド機能停止
void ib2::PropTlmCmd::shutdown()
{
	if(sub_fan_duty_){
		std::cout << " -> Shutdown Fan Duty Subscriber"           << std::endl;
		// sub_fan_duty_.reset();
	}

	if(pub_fan_status_){
		std::cout << " -> Shutdown Fan Status Publisher"          << std::endl;
		// pub_fan_status_.reset();
	}

	if(switch_power_server_)
	{
		std::cout << " -> Shutdown Switch Power Service Server" << std::endl;
		// switch_power_server_.reset();
	}

	initFanStatus(ib2_interfaces::msg::PowerStatus::UNKNOWN);
}

// End Of File -----------------------------------------------------------------
