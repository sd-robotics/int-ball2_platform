
#pragma once

#include <iostream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "ib2_prop/prop_tlmcmd.h"
#include "ib2_prop/prop_pca9685.h"
#include "ib2_interfaces/srv/update_parameter.hpp"

#define   SERVICE_UPDATE_PARAMS       "/prop/update_params"
#define   FAN_NUM     	8
#define   PUB_DURATION  1.0
#define   MON_DURATION  0.01
#define   FREQ      	1000

namespace ib2
{

class PropManager : public rclcpp::Node
{
    //----------------------------------------------------------------------
    // コンストラクタ/デストラクタ
public:
	/** コンストラクタ */
	PropManager(const rclcpp::NodeOptions& options);

	/** デストラクタ */
	~PropManager();

	//----------------------------------------------------------------------
	// コピー/ムーブ
private:
	/** コピーコンストラクタ. */
	PropManager(const PropManager&)            = delete;

	/** コピー代入演算子. */
	PropManager& operator=(const PropManager&) = delete;

	/** ムーブコンストラクタ. */
	PropManager(PropManager&&)                 = delete;

	/** ムーブ代入演算子. */
	PropManager& operator=(PropManager&&)      = delete;
    
	//----------------------------------------------------------------------
	// 実装
public:
	/** 管理機能実行 */
	void start();

	//--------------------------------------------------------------------------
	// 実装
private:
	/** rosparamからのパラメータ取得
	 * @return                     エラー識別子
	 *                                  0 : 取得成功
	 *                                  1 : ファン数取得エラー
	 *                                  2 : デバイスファイル名取得エラー
	 *                                  4 : デバイスアドレス取得エラー
	 *                                  8 : PWM周波数取得エラー
	 *                                  16: ファンステータスパブリッシュ周期取得エラー
	 *                                  32: I2C通信周期取得エラー
	 */
    int getParameter();

	/** PWM制御ボード(PCA9685)にPWM信号を送信 */
	void sendPWM();

	/** 推進機能ノード停止 */
	void shutdown();

	/** パラメータ更新サービス受信時の処理
	 * @param [in]                                       パラメータ更新サービスリクエスト
	 * @param [in]                 res                   パラメータ更新サービス実行結果
	 * @retval                     true                  更新成功
	 * @retval                     false                 更新失敗
	 */
	bool updateParams(
        const std::shared_ptr<ib2_interfaces::srv::UpdateParameter::Request> req,
        std::shared_ptr<ib2_interfaces::srv::UpdateParameter::Response> res);

	//----------------------------------------------------------------------
	// メンバ変数
private:

	/** ファン駆動状態パブリッシュ用　ROS Timer */
	rclcpp::TimerBase::SharedPtr  	 fan_status_timer_;

	/** PWM制御信号送信用　ROS Timer */
	rclcpp::TimerBase::SharedPtr  	 pwm_control_timer_;

	/** パラメータ更新サービスサーバ */
	rclcpp::Service<ib2_interfaces::srv::UpdateParameter>::SharedPtr update_params_server_;

    /* ファン数 */
    int32_t                          fan_num_;

    /** テレメトリ・コマンド　オブジェクト */
    ib2::PropTlmCmd                  prop_tlm_cmd_;

    /** PWM制御信号送信　オブジェクト */
	ib2::PropPCA9685                 prop_pca9685_;

    /** デバイスファイル名(PCA9685) */
    std::string                      device_file_name_;

    /** デバイスのアドレス */
    int                              device_address_;

    /** PWM周期 */
    unsigned short                   pwm_frequency_;

    /** ファンステータスのパブリッシュ周期[s] */
    rclcpp::Duration                 pub_fan_status_duration_;

	/** I2C通信周期[s] */
    rclcpp::Duration           		 i2c_comm_duration_;

    /** 初期化エラー識別子 */
    int                              init_error_id_;

	/** ファンデューティ */
	std::vector<float>               duty_;
};

} // namespace ib2

// Register the node with the rclcpp components system
RCLCPP_COMPONENTS_REGISTER_NODE(ib2::PropManager)

// End Of File -----------------------------------------------------------------
