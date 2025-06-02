#pragma once

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>

namespace ib2
{
    /**
     * @brief 位置制御プロファイルパラメータ.
     */
    class CtlBody final : public rclcpp::Node
    {
        //----------------------------------------------------------------------
        // コンストラクタ/デストラクタ
    public:
        /** デフォルトコンストラクタ */
        CtlBody();

        /** @brief コンストラクタ
         * @param options ノードオプション
         */
        explicit CtlBody(const rclcpp::NodeOptions& options);

        /** デストラクタ. */
        ~CtlBody();

        //----------------------------------------------------------------------
        // コピー/ムーブ
    public:
        /** コピーコンストラクタ. */
        CtlBody(const CtlBody&) = delete;

        /** コピー代入演算子. */
        CtlBody& operator=(const CtlBody&) = delete;

        /** ムーブコンストラクタ. */
        CtlBody(CtlBody&&) = delete;

        /** ムーブ代入演算子. */
        CtlBody& operator=(CtlBody&&) = delete;

        //----------------------------------------------------------------------
        // 属性(Getter)
    public:
        /** 機体質量の取得
         * @return 機体質量[kg]
         */
        double m() const;
        
        /** 質量特性行列の参照
         * @return 質量特性行列[kgm2]の参照
         */
        const Eigen::Matrix3d& Is() const;
        
        //----------------------------------------------------------------------
        // メンバー変数
    private:
        /** 機体質量[kg] */
        double m_;
        
        /** 質量特性行列[kgm2] */
        Eigen::Matrix3d Is_;
    };
}

// End Of File -----------------------------------------------------------------
