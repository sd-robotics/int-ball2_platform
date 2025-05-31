#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
//fsm.h"    // Modification for platform packages
#include "ib2_ctl/dtc.h"

// Standard messages
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp> 
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "ib2_interfaces/msg/navigation.hpp"
#include "ib2_interfaces/msg/ctl_status_type.hpp"
#include "ib2_interfaces/msg/ctl_status.hpp"
#include "ib2_interfaces/msg/ctl_profile.hpp"
#include "ib2_interfaces/action/ctl_command.hpp"
#include "ib2_interfaces/srv/update_parameter.hpp"

#include <memory>

// マクロ宣言、後でどこか共通の*.hに移動したほうがいいかも
#define TOPIC_CTL_WRENCH  "/ctl/wrench"
#define TOPIC_CTL_PROFILE "/ctl/profile"
#define TOPIC_CTL_STATUS  "/ctl/status"
#define TOPIC_NAV_POSE    "/sensor_fusion/navigation"

namespace ib2
{
    // classの前方宣言
    class CtlBody;
    class PosAttController;
    class PosAttProfiler;
}
    
/**
* @brief 制御ノードクラス
*/
class Ctl : public rclcpp::Node
{
    //----------------------------------------------------------------------
    // コンストラクタ/デストラクタ
private:
    /** デフォルトコンストラクタ */
    Ctl() = delete;

public:
    using CtlCommand = ib2_interfaces::action::CtlCommand;
    using GoalHandleCtlCommand = rclcpp_action::ServerGoalHandle<CtlCommand>;


    /** コンストラクタ */
    explicit Ctl(const rclcpp::NodeOptions& options);

    /** デストラクタ */
    ~Ctl();

    //----------------------------------------------------------------------
    // コピー/ムーブ
private:
    /** コピーコンストラクタ. */
    Ctl(const Ctl&) = delete;

    /** コピー代入演算子. */
    Ctl& operator=(const Ctl&) = delete;

    /** ムーブコンストラクタ. */
    Ctl(Ctl&&) = delete;

    /** ムーブ代入演算子. */
    Ctl& operator=(Ctl&&) = delete;
    
    //----------------------------------------------------------------------
    // 操作(Setter)
private:
    /** メンバ設定
     * @retval true 設定成功
     * @retval false 設定失敗
     */
    bool setMember();
    
    /** 位置姿勢保持設定 */
    void setKeepPose();

    //----------------------------------------------------------------------
    // 実装
private:
    /** 制御目標への誘導
     * @param [in] goalType 制御目標種別
     * @param [in] tolp 位置誤差許容値[m]
     * @param [in] tola 姿勢誤差許容値[rad]
     * @retval true 目標到達
     * @retval false 中断
     */
    // bool guidance(int32_t goal_type, double tolp, double tola);
    bool guidance(int32_t goal_type, double tolp, double tola);
    
    /** ターゲットモードの処理
     * @param [in] goal 制御目標
     */
    void target();

    /** リリースモードの処理 */
    void release();
    
    /** ドッキングモードの処理
     * @param [in] correction マーカー補正フラグ(true更新あり/false補正なし)
     */
    void docking(bool correction);
    
    /** ドッキングスタンバイモードの処理 */
    void dockingStandBy();
    
    /** スキャンモードの処理 */
    void scan();
    
    /** 停止誘導モードの処理 */
    void stopping();
    
    /** アクション中止
     * @param [in] result_type アクション結果種別
     */
    void abortAction(uint8_t result_type);
    
    /** 制御目標キャンセル時の処理
     * @param [in] docking ドッキングモードフラグ
     */
    void cancelTarget(bool docking = false);

    /** 制御目標到達時の処理 */
    void goalTarget();

    /** 航法メッセージタイムアウト処理 */
    void timeoutNavigation();

    /** 制御目標到達判定
     * @param [in, out] stay 制御目標周辺継続判定結果
     * @param [in, out] tin 制御目標周辺到達時刻
     * @param [in] tnav 最新の航法時刻
     * @param [in] fb アクションフィードバック
     * @param [in] tolp 位置誤差許容値[m]
     * @param [in] tola 姿勢誤差許容値[rad]
     * @return 制御目標到達判定結果
     */
    bool reachGoal
    (bool& stay, rclcpp::Time& tin, const rclcpp::Time& tnav,
     const ib2_interfaces::action::CtlCommand::Feedback& fb, double tolp, double tola);

    /** 制御目標到達判定(SCAN)
     * @param [in, out] stay 制御目標周辺継続判定結果
     * @param [in, out] tin 制御目標周辺到達時刻
     * @param [in] tnav 最新の航法時刻
     * @param [in] fb アクションフィードバック
     * @param [in] tola 姿勢誤差許容値[rad]
     * @return 制御目標到達判定結果
     */
    bool reachGoalScan
    (bool& stay, rclcpp::Time& tin, const rclcpp::Time& tnav,
     const CtlCommand::Feedback& fb, double tola);

    /** 制御目標到達判定(DOCK)
     * @return 制御目標到達判定結果
     */
    bool reachGoalDock();

    /** 制御目標妥当性確認
     * @param [in] goal 制御目標
     * @retval true 妥当
     * @retval false 不正
     */
    bool validCommand(const std::shared_ptr<const CtlCommand::Goal>& goal) const;

    /** 航法メッセージ妥当性確認
     * @param [in] nav 判定対象航法メッセージ
     * @param [in] first 初期フラグ(true:初めて / false : 初めてでない)
     * @retval true 妥当
     * @retval false 不正
     */
    bool validNavigation(const ib2_interfaces::msg::Navigation& nav, bool first) const;
    
    //--------------------------------------------------------------------------
    // 実装（コールバック関数）
public:
    /** 制御目標アクション受信時の処理
     * @param [in] goal 制御目標値メッセージ
     */
    void commandCallback(const std::shared_ptr<GoalHandleCtlCommand>& goal);

    /** パラメータ更新サービス受信時の処理
     * @param [in] パラメータ更新サービスリクエスト
     * @param [in] res パラメータ更新サービス実行結果
     * @retval true 更新成功
     * @retval false 更新失敗
     */
    bool updateCallback(
        const std::shared_ptr<ib2_interfaces::srv::UpdateParameter::Request> req,
        std::shared_ptr<ib2_interfaces::srv::UpdateParameter::Response> res);

    /** 航法値のサブスクライバのコールバック関数
     * @param [in] nav_stamp 航法値
     */
    void navinfoCallback(const ib2_interfaces::msg::Navigation& nav_stamp);

    /** 定期的な処理
     * @param [in] ev タイマーイベント
     */
    void timerCallback();

    //----------------------------------------------------------------------
    // メンバ変数
private:
    /** 制御目標アクションサーバ */
    rclcpp_action::Server<CtlCommand>::SharedPtr command_as_;
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const CtlCommand::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleCtlCommand> goal_handle);
    void handle_accepted(
        const std::shared_ptr<GoalHandleCtlCommand> goal_handle);
    std::shared_ptr<GoalHandleCtlCommand> goal_handle_;

    /** パラメータ更新サービスサーバ */
    rclcpp::Service<ib2_interfaces::srv::UpdateParameter>::SharedPtr update_ss_;

    /** マーカー補正サービスクライアント */
    rclcpp::Client<ib2_interfaces::srv::MarkerCorrection>::SharedPtr marker_sc_;
    
    /** 誘導制御ステータス出力間隔 */
    rclcpp::Duration interval_status_;
    
    /** フィードバック間隔 */
    rclcpp::Duration interval_feedback_;

    /** 目標到達継続時間 */
    rclcpp::Duration duration_goal_;
    
    /** 制御目標位置到達判定値[m] */
    double tolerance_pos_;

    /** 制御目標姿勢到達判定値[rad] */
    double tolerance_att_;
    
    /** 位置停止判定値[m] */
    double tolerance_pos_stop_;
    
    /** 姿勢停止判定値[rad] */
    double tolerance_att_stop_;
    
    /** 航法異常連続上限 */
    size_t nav_counter_;
    
    /** 航法位置変動量上限[m/s] */
    double nav_dr_;
    
    /** 航法速度変動量上限[m/s2] */
    double nav_dv_;
    
    /** 航法加速度変動量上限[m/s3] */
    double nav_da_;

    /** 航法姿勢変動量上限[rad/s] */
    double nav_dq_;
    
    /** 航法角速度変動量上限[rad/s2] */
    double nav_dw_;
    
    /** ターゲットキャンセルの待ち時間 */
    rclcpp::Duration waitCancel_;
    
    /** リリース開始からAIP移動開始までの待ち時間 */
    rclcpp::Duration waitRelease_;
    
    /** ホーミング時VisualSLAM較正の待ち時間 */
    rclcpp::Duration waitCalibration_;
    
    /** ドッキング開始からスタンバイまでの待ち時間 */
    rclcpp::Duration waitDocking_;
    
    // Subscriber
    /** 航法値のサブスクライバ */
    rclcpp::Subscription<ib2_interfaces::msg::Navigation>::SharedPtr navinfo_sub_;

    /** TODO: 目標値のサブスクライバ */
    rclcpp::Subscription<ib2_interfaces::msg::Navigation>::SharedPtr target_sub_;

    // Publisher
    /** 誘導制御モードパブリッシャ */
    rclcpp::Publisher<ib2_interfaces::msg::CtlStatus>::SharedPtr status_pub_;

    /** 力トルクのパブリッシャ */
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;

    /** 制御プロファイルのパブリッシャ */
    rclcpp::Publisher<ib2_interfaces::msg::CtlProfile>::SharedPtr profile_pub_;

    /** 航法メッセージの前回値 */
    ib2_interfaces::msg::Navigation last_nav_stamp_;

    /** 機体パラメータ */
    std::unique_ptr<ib2::CtlBody> body_;

    /** 誘導制御則 */
    std::unique_ptr<ib2::PosAttController> controller_;

    /** 位置姿勢誘導プロファイル */
    std::unique_ptr<ib2::PosAttProfiler> profiler_;

    /** ファン選択 */
    //std::unique_ptr<Fsm> fsm_;    // Modification for platform packages

    /** 検知 */
    Dtc dtc_;

    /**  誘導制御モード */
    int32_t status_;

    /** ステータス出力タイマー */
    rclcpp::TimerBase::SharedPtr timer_;

    /** CtlStatus sequcens id  */
    mutable uint32_t seq_status_;
    
    /** 航法えメッセージ取得フラグ */
    bool valid_navigation_;
};


rclcpp_action::GoalResponse Ctl::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const CtlCommand::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request with target position: [%f, %f, %f]",
                goal->target.pose.position.x,
                goal->target.pose.position.y,
                goal->target.pose.position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Ctl::handle_cancel(
    const std::shared_ptr<GoalHandleCtlCommand> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void Ctl::handle_accepted(const std::shared_ptr<GoalHandleCtlCommand> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Ctl::commandCallback, this, std::placeholders::_1), goal_handle}.detach();
}


// End Of File -----------------------------------------------------------------
