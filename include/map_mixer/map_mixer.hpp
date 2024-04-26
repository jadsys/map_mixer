/**
* @file     map_mixer.hpp
* @brief    地図生成クラスMapMixerの定義ヘッダファイル
* @author   S.Kumada
* @date     2021/09/28
* @note     map_updaterにて更新したレイヤ地図から、自己位置推定用地図と局所経路探索用地図を生成するためのクラスの定義ヘッダファイル
*/

#ifndef MAP_MIXER_H_
#define MAP_MIXER_H_

// ヘッダインクルード
#include <stdio.h> // 標準入出力
#include <string> // frame_id指定用
#include <boost/bind.hpp> // callback関数の引数設定用

#include <ros/ros.h> // 基本ライブラリ
#include <nav_msgs/OccupancyGrid.h> // グリットマップ
#include <message_filters/subscriber.h> // timestampを意識したサブスクライバーが作れるすごいやつ

// define定義
#define DEFAULT_ENTITY_ID                    ""
#define DEF_STATIC_LAYER_TOPIC_NAME         "static_layer_map"
#define DEF_QUASI_STATIC_LAYER_TOPIC_NAME   "q_static_layer_map"
#define DEF_DYNAMIC_LAYER_TOPIC_NAME        "dynamic_layer_map"
#define DEF_NO_ENTRY_LAYER_TOPIC_NAME       "no_entry_layer_map"
#define DEF_SOCIO_MAP_TOPIC_NAME            "map_movebase"
#define DEF_EGO_MAP_TOPIC_NAME              "map"
#define DEF_EGO_MAP_MIX_INTERVAL            5

#define ROS_QUEUE_SIZE_1                    1
#define ROS_RATE_30HZ                       30

// GridMapコスト
#define     FREESPACE_COST_GRIDMAP		    0
#define     UNKNOWN_COST_GRIDMAP		    -1
#define     OBSTACLE_COST_GRIDMAP		    100


// レイヤ型地図の構造体
typedef struct st_layerd_map_
{
    private:
        bool is_active_static_layer;
        bool is_active_quasi_static_layer;
        bool is_active_dynamic_layer;
        bool is_active_no_entry_layer;

    public:
        std::string             map_name_;  //!< 地図の名前

        nav_msgs::OccupancyGrid *static_layer_map_;         //!< 静的レイヤ地図のポインタ
        nav_msgs::OccupancyGrid *quasi_static_layer_map_;   //!< 準静的レイヤ地図のポインタ
        nav_msgs::OccupancyGrid *dynamic_layer_map_;        //!< 動的レイヤ地図のポインタ
        nav_msgs::OccupancyGrid *no_entry_layer_map_;       //!< 進入禁止レイヤ地図のポインタ

        nav_msgs::OccupancyGrid mixed_maps_;   //!< 統合した地図のポインタ

        /**
         * @brief        初期化関数
         * @param[in]    nav_msgs::OccupancyGrid *static_layer          静的レイヤ地図のポインタ
         * @param[in]    nav_msgs::OccupancyGrid *quasi_static_layer    準静的レイヤ地図のポインタ
         * @param[in]    nav_msgs::OccupancyGrid *dyanmic_layer         動的レイヤ地図のポインタ
         * @param[in]    nav_msgs::OccupancyGrid *no_entry_layer        進入禁止レイヤ地図のポインタ
         * @return       void
         * @details      統合する地図のポインタを設定する。
         */
        void active(nav_msgs::OccupancyGrid *static_layer = nullptr,nav_msgs::OccupancyGrid *quasi_static_layer = nullptr, nav_msgs::OccupancyGrid *dyanmic_layer = nullptr, nav_msgs::OccupancyGrid *no_entry_layer = nullptr)
        {

            static_layer_map_       = static_layer;
            quasi_static_layer_map_ = quasi_static_layer;
            dynamic_layer_map_      = dyanmic_layer;
            no_entry_layer_map_     = no_entry_layer;

            if(static_layer_map_ != nullptr)
            {
                is_active_static_layer = true;
            }
            if(quasi_static_layer_map_ != nullptr)
            {
                is_active_quasi_static_layer = true;
            }
            if(dynamic_layer_map_ != nullptr)
            {
                is_active_dynamic_layer = true;
            }
            if(no_entry_layer_map_ != nullptr)
            {
                is_active_no_entry_layer = true;
            }

            return;

        }

        /**
         * @brief        アクティブな地図の総数の取得
         * @param[in]    void
         * @return       unsigned int アクティブな地図の数
         * @details      統合するための地図の数を返す。
         */
        unsigned int numOfActiveMap(void)
        {
            unsigned int result = 0;
            
            if(is_active_static_layer) result++;
            if(is_active_quasi_static_layer) result++;
            if(is_active_dynamic_layer) result++;
            if(is_active_no_entry_layer) result++;

            return (result);
        }

        /**
         * @brief        静的レイヤのアクティブ判定関数
         * @param[in]    void
         * @return       bool アクティブ/非アクティブ
         * @details      統合する地図のアクティブ/非アクティブを判定し結果を返す
         */
        bool getStaticLayerValid(void)
        {
            return (is_active_static_layer);
        }

        /**
         * @brief        準静的レイヤのアクティブ判定関数
         * @param[in]    void
         * @return       bool アクティブ/非アクティブ
         * @details      統合する地図のアクティブ/非アクティブかを判定し結果を返す
         */
        bool getQStaticLayerValid(void)
        {
            return (is_active_quasi_static_layer);
        }

        /**
         * @brief        動的レイヤのアクティブ判定関数
         * @param[in]    void
         * @return       bool アクティブ/非アクティブ
         * @details      統合する地図のアクティブ/非アクティブかを判定し結果を返す
         */
        bool getDynamicLayerValid(void)
        {
            return (is_active_dynamic_layer);
        }

        /**
         * @brief        進入禁止レイヤのアクティブ判定関数
         * @param[in]    void
         * @return       bool アクティブ/非アクティブ
         * @details      統合する地図のアクティブ/非アクティブかを判定し結果を返す
         */
        bool getNoEntryLayerValid(void)
        {
            return (is_active_no_entry_layer);
        }

        /**
         * @brief        地図のサイズ取得関数
         * @param[in]    void
         * @return       unsigned int サイズ
         * @details      地図のデータサイズの大きさを返す
         */
        unsigned int getMapSize(void)
        {
            if( is_active_static_layer &&
                static_layer_map_ != nullptr )
            {
                return (static_layer_map_->data.size());
            }
            else
            {
                return 0;
            }
        }

} LayerdMap;

/**
 * @brief   地図の統合クラス
 * @details 受信したレイヤ地図を統合し、ソシオ地図、エゴ地図を生成する
 */
class MapMixer
{
    public:

        /**
        * @brief   MapMixerクラスのコンストラクタ
        * @details 初期化を行う
        */
        MapMixer(ros::NodeHandle paramNode, ros::NodeHandle node);


        /**
        * @brief   MapMixerクラスのデストラクタ
        * @details オブジェクトの破棄を行う
        */
        ~MapMixer();

        /**
        * @brief        メインループ関数
        * @param[in]    void
        * @return       void
        * @details      各レイヤ地図の更新を管理し、地図を更新・配信処理を呼び出す
        */
        void mixMapLoop(void);

        /**
        * @brief        地図の統合関数
        * @param[in]    LayerdMap *map　レイヤ地図の構造体
        * @return       void
        * @details      エゴ・ソシオ地図への統合を行う
        */
        void mixing(LayerdMap *map);        

        ros::Subscriber sub_static_layer_;
        ros::Subscriber sub_q_static_layer_;
        ros::Subscriber sub_dynamic_layer_;
        ros::Subscriber sub_no_entry_layer_;
        ros::Publisher  pub_socio_map_;
        ros::Publisher  pub_ego_map_;

        LayerdMap *ego_map_;
        LayerdMap *socio_map_;


    private:
        /**
        * @brief        レイヤ地図受信コールバック関数
        * @param[in]    msg nav_msgs::OccupancyGridConstPtr型のメッセージデータ
        * @param[in]    map_name std::String型　サブスクライバ毎に地図トピック名が入る
        * @return       void
        * @details      受信したレイヤ地図(静的・準静的・動的・進入禁止領域)をグローバルで保持する
        */
        void recvLayerMapCb(const nav_msgs::OccupancyGridConstPtr& msg, const std::string map_name);

        /**
        * @brief        局所経路探索用地図の生成関数
        * @param[in]    void
        * @return       void
        * @details      各レイヤ地図を統合し、ソシオ地図の生成を行う
        */
        void mixSocioMap(void);

        /**
        * @brief        自己位置探索用地図の生成Timer関数
        * @param[in]    const ros::TimerEvent& タイマーイベントのポインタ 
        * @return       void
        * @details      指定秒毎に各レイヤ地図を統合し、ソシオ地図の生成を行う
        */
        void mixEgoMap(const ros::TimerEvent&);

        std::string entity_id_;
        bool is_sub_static_layer_;
        bool is_sub_quasi_static_layer_;
        bool is_sub_dynamic_layer_;
        bool is_sub_no_entry_layer_;
        bool is_egomap_mix_only_once_;
        bool is_published_ego_map_;
        bool is_published_sosio_map_;
        int dynamic_layer_occupansy_th_;

        nav_msgs::OccupancyGrid *static_layer_map_;
        nav_msgs::OccupancyGrid *quasi_static_layer_map_;
        nav_msgs::OccupancyGrid *dynamic_layer_map_;
        nav_msgs::OccupancyGrid *no_entry_layer_map_;
        
        ros::Timer      egomap_mixing_timer_;
};

#endif