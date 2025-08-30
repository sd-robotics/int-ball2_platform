
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "ib2_prop/prop_common.h"
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/multi_array_dimension.hpp>
#include "ib2_interfaces/msg/fan_status.hpp"
#include "ib2_interfaces/msg/power_status.hpp"
#include "ib2_interfaces/srv/switch_power.hpp"

#define TOPIC_CTL_DUTY               "/ctl/duty"
#define TOPIC_PROP_STATUS            "/prop/status"
#define SERVICE_SWITCH_POWER         "/prop/switch_power"
#define DUTY_MIN                     0.F			// 最小ファン駆動デューティ比[-]
#define DUTY_MAX                     1.F			// 最大ファン駆動デューティ比[-]

namespace ib2
{

/**
* @brief 推進機能ノード　テレメトリ・コマンドクラス
*/
class PropTlmCmd : public rclcpp::Node
{
	//----------------------------------------------------------------------
	// コンストラクタ/デストラクタ
public:
	/** コンストラクタ */
	PropTlmCmd(const rclcpp::NodeOptions& options);

	/** デストラクタ */
	~PropTlmCmd();

	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	PropTlmCmd(const PropTlmCmd&)            = delete;

	/** コピー代入演算子. */
	PropTlmCmd& operator=(const PropTlmCmd&) = delete;

	/** ムーブコンストラクタ. */
	PropTlmCmd(PropTlmCmd&&)                 = delete;

	/** ムーブ代入演算子. */
	PropTlmCmd& operator=(PropTlmCmd&&)      = delete;
    
	//----------------------------------------------------------------------
	// 操作(Setter)
private:

	//----------------------------------------------------------------------
	// 実装
public:
	/** ファン駆動状態をパブリッシュ */
	void pubFanStatus();

	/** ファン駆動状態をパブリッシュ(異常時) */
	void pubErrorFanStatus();

	/** ファン駆動デューティ比のgetter
	 * @return                       ファン駆動デューティ比
	 */
	std::vector<float> getFanDuty();

	/** テレメトリ・コマンド機能停止 */
	void shutdown();

	//--------------------------------------------------------------------------
	// 実装
private:
	/** ファン駆動デューティ比をサブスクライブ
	 * @param [in]      msg           ファン駆動デューティ比
	 */
	void subFanDuty(const example_interfaces::msg::Float64MultiArray& msg);

	/** ファン駆動状態初期化
	 * @param [in]      status        推進機能起動/停止ステータス
	 */
	void initFanStatus(const uint8_t& status);

	/** ファン駆動状態メッセージ生成 */
	void generateFanStatus();

	/** 推進機能起動/停止
	 * @param [in]                 req              推進機能起動/停止サービスリクエスト
	 * @param [in]                 res              現在の推進機能ステータス
	 * @retval                     true             設定成功
	 * @retval                     false            設定失敗
	 */
	bool switchPower(
        const std::shared_ptr<ib2_interfaces::srv::SwitchPower::Request> req,
        std::shared_ptr<ib2_interfaces::srv::SwitchPower::Response> res
	);

	//----------------------------------------------------------------------
	// メンバ変数
private:
	/** ファンデューティ比(誘導制御ノード)　サブスクライバ */
	rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr sub_fan_duty_;

	/** 推進機能ノードのファン駆動状態パブリッシャ */
	rclcpp::Publisher<ib2_interfaces::msg::FanStatus>::SharedPtr pub_fan_status_;

	/** 推進機能起動/停止サービスサーバ */
    rclcpp::Service<ib2_interfaces::srv::SwitchPower>::SharedPtr switch_power_server_;

	/** ファン数 */
	int32_t                          fan_num_;

	/** ファン駆動デューティ比メッセージ */
	example_interfaces::msg::Float64MultiArray fan_duty_;

	/** ファン駆動状態メッセージ */
	ib2_interfaces::msg::FanStatus	 fan_status_;
};

}  // namespace ib2

// End Of File -----------------------------------------------------------------
