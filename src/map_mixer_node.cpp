/**
* @file     map_mixer_node.cpp
* @brief    地図生成クラスMapMixerの実装ソースファイル
* @author   S.Kumada
* @date     2021/09/28
* @note     map_updaterにて更新したレイヤ地図から、自己位置推定用地図と局所経路探索用地図を生成する
*/

#include <map_mixer/map_mixer.hpp>


MapMixer::MapMixer(ros::NodeHandle paramNode, ros::NodeHandle node)
{
       
    // ローカル変数定義
    std::string static_layer_topic;
    std::string q_static_layer_topic;
    std::string dynamic_layer_topic;
    std::string no_entry_layer_topic;
    std::string socio_map_topic;
    std::string ego_map_topic;
    std::string topic_entity_id;
    double      egomap_mix_interval;   
    
    // グローバル変数初期化
    entity_id_                  = "";
    is_sub_static_layer_        = false;
    is_sub_quasi_static_layer_  = false;
    is_sub_dynamic_layer_       = false;
    is_sub_no_entry_layer_      = false;
    is_published_ego_map_       = false;
    is_published_sosio_map_     = false;

    static_layer_map_       = new nav_msgs::OccupancyGrid();
    quasi_static_layer_map_ = new nav_msgs::OccupancyGrid();
    dynamic_layer_map_      = new nav_msgs::OccupancyGrid();
    no_entry_layer_map_     = new nav_msgs::OccupancyGrid();

    ego_map_                = new LayerdMap();
    ego_map_->active(static_layer_map_, quasi_static_layer_map_);
    ego_map_->map_name_     = DEF_EGO_MAP_TOPIC_NAME;
    
    socio_map_              = new LayerdMap();
    socio_map_->active(static_layer_map_, quasi_static_layer_map_, dynamic_layer_map_, no_entry_layer_map_);
    socio_map_->map_name_   = DEF_SOCIO_MAP_TOPIC_NAME;

    // パラメータ読み込み


    // ロボットの固有ID名取得
    if (paramNode.getParam("entity_id", entity_id_))
    {
        ROS_INFO("entity_id (%s)", entity_id_.c_str());
    }
    else
    {
        entity_id_ = DEFAULT_ENTITY_ID; // DEFAULT_ROBOT_ID = ""
    }
    if(!entity_id_.empty())
    {
        topic_entity_id = "/" + entity_id_;
    }


    // 静的レイヤ地図トピック名取得
    if (paramNode.getParam("static_layer_topic", static_layer_topic))
    {
        ROS_INFO("static_layer_topic (%s)", static_layer_topic.c_str());
    }
    else
    {
        static_layer_topic = DEF_STATIC_LAYER_TOPIC_NAME; // DEF_STATIC_LAYER_TOPIC_NAME = static_layer_map
    }

    // 準静的レイヤ地図トピック名取得
    if (paramNode.getParam("q_static_layer_topic", q_static_layer_topic))
    {
        ROS_INFO("q_static_layer_topic (%s)", q_static_layer_topic.c_str());
    }
    else
    {
        q_static_layer_topic = DEF_QUASI_STATIC_LAYER_TOPIC_NAME; // DEF_QUASI_STATIC_LAYER_TOPIC_NAME = q_static_layer_map
    }

    // 動的レイヤ地図トピック名取得
    if (paramNode.getParam("dynamic_layer_topic", dynamic_layer_topic))
    {
        ROS_INFO("dynamic_layer_topic (%s)", dynamic_layer_topic.c_str());
    }
    else
    {
        dynamic_layer_topic = DEF_DYNAMIC_LAYER_TOPIC_NAME; // DEF_DYNAMIC_LAYER_TOPIC_NAME = dynamic_layer_map
    }

    // 進入禁止領域レイヤ地図トピック名取得
    if (paramNode.getParam("no_entry_layer_topic", no_entry_layer_topic))
    {
        ROS_INFO("no_entry_layer_topic (%s)", no_entry_layer_topic.c_str());
    }
    else
    {
        no_entry_layer_topic = DEF_NO_ENTRY_LAYER_TOPIC_NAME; // DEF_NO_ENTRY_LAYER_TOPIC_NAME = no_entry_layer_map
    }

    // ソシオ地図トピック名取得
    if (paramNode.getParam("socio_map_topic", socio_map_topic))
    {
        ROS_INFO("socio_map_topic (%s)", socio_map_topic.c_str());
    }
    else
    {
        socio_map_topic = DEF_SOCIO_MAP_TOPIC_NAME; // DEF_SOCIO_MAP_TOPIC_NAME = map_movebase
    }

    // エゴ地図トピック名取得
    if (paramNode.getParam("ego_map_topic", ego_map_topic))
    {
        ROS_INFO("ego_map_topic (%s)", ego_map_topic.c_str());
    }
    else
    {
        ego_map_topic = DEF_EGO_MAP_TOPIC_NAME; // DEF_EGO_MAP_TOPIC_NAME = map
    }

    // EGO地図のmix間隔時間（秒）
    if (paramNode.getParam("egomap_mix_interval", egomap_mix_interval))
    {
        ROS_INFO("egomap_mix_interval (%lf)", egomap_mix_interval);
    }
    else
    {
        egomap_mix_interval = DEF_EGO_MAP_MIX_INTERVAL;  // 5秒
    }

    // EGO地図の統合モード(true:1回のみ)
    if (paramNode.getParam("egomap_mix_only_once", is_egomap_mix_only_once_))
    {
        if(is_egomap_mix_only_once_)
        {    
            ROS_INFO("egomap_mix_only_once (%s)", "true");
        }
        else
        {
            ROS_INFO("egomap_mix_only_once (%s)", "false");
        }
    }
    else
    {
        is_egomap_mix_only_once_ = false;  // 複数回実行
    }

    // 統合する動的レイヤ地図の占有率のしきい値設定(0~100%)
    if (paramNode.getParam("dynamic_layer_occupancy_thresholds", dynamic_layer_occupansy_th_))
    {
        ROS_INFO("dynamic_layer_occupancy_thresholds (%d)", dynamic_layer_occupansy_th_);
    }
    else
    {
        dynamic_layer_occupansy_th_ = OBSTACLE_COST_GRIDMAP;  // 100%
    }

    if(dynamic_layer_occupansy_th_ < 0 || dynamic_layer_occupansy_th_ > 100)
    {
        ROS_WARN("Param [dynamic_layer_occupancy_thresholds] must be specified from 0 to 100.");
        
        dynamic_layer_occupansy_th_ = OBSTACLE_COST_GRIDMAP;  // 100%
    }

    // パブリッシャ定義
    pub_socio_map_      = node.advertise<nav_msgs::OccupancyGrid>(topic_entity_id + "/" + socio_map_topic, ROS_QUEUE_SIZE_1, true);
    pub_ego_map_        = node.advertise<nav_msgs::OccupancyGrid>(topic_entity_id + "/" + ego_map_topic, ROS_QUEUE_SIZE_1, true);
    // サブスクライバ定義
    sub_static_layer_   = node.subscribe<nav_msgs::OccupancyGrid>(topic_entity_id + "/" + static_layer_topic, ROS_QUEUE_SIZE_1, boost::bind(&MapMixer::recvLayerMapCb, this, _1, DEF_STATIC_LAYER_TOPIC_NAME));
    sub_q_static_layer_ = node.subscribe<nav_msgs::OccupancyGrid>(topic_entity_id + "/" + q_static_layer_topic, ROS_QUEUE_SIZE_1, boost::bind(&MapMixer::recvLayerMapCb, this, _1, DEF_QUASI_STATIC_LAYER_TOPIC_NAME));
    sub_dynamic_layer_  = node.subscribe<nav_msgs::OccupancyGrid>(topic_entity_id + "/" + dynamic_layer_topic, ROS_QUEUE_SIZE_1, boost::bind(&MapMixer::recvLayerMapCb, this, _1, DEF_DYNAMIC_LAYER_TOPIC_NAME));
    sub_no_entry_layer_ = node.subscribe<nav_msgs::OccupancyGrid>(topic_entity_id + "/" + no_entry_layer_topic, ROS_QUEUE_SIZE_1, boost::bind(&MapMixer::recvLayerMapCb, this, _1, DEF_NO_ENTRY_LAYER_TOPIC_NAME));
    
    // Timer設定
    //　エゴ地図合成タイマー
    egomap_mixing_timer_ = node.createTimer(ros::Duration(egomap_mix_interval), &MapMixer::mixEgoMap, this, false, true);
    
}

MapMixer::~MapMixer()
{
    free(ego_map_);
    free(socio_map_);
    free(static_layer_map_);       
    free(quasi_static_layer_map_); 
    free(dynamic_layer_map_);      
    free(no_entry_layer_map_ );             
}

void MapMixer::recvLayerMapCb(const nav_msgs::OccupancyGridConstPtr& msg, const std::string map_name)
{
    // ROS_INFO("subscribe %s", map_name.c_str());

    unsigned int map_size = msg->info.width * msg->info.height;
    nav_msgs::OccupancyGrid *map_p = nullptr;

    if(map_name == DEF_STATIC_LAYER_TOPIC_NAME)
    { // static layer
    
#if DEBUG
        ROS_INFO("DEBUG : Sub Static layer map");
#endif

        map_p = static_layer_map_;

        is_sub_static_layer_ = true;
    }
    if(map_name == DEF_QUASI_STATIC_LAYER_TOPIC_NAME)
    { // quasi static layer
        
#if DEBUG
        ROS_INFO("DEBUG : Sub Semi static layer map");
#endif

        map_p = quasi_static_layer_map_;

        is_sub_quasi_static_layer_ = true;
    }
    if(map_name == DEF_DYNAMIC_LAYER_TOPIC_NAME)
    { // dynamic layer
        
#if DEBUG
        ROS_INFO("DEBUG : Sub Dynamic layer map");
#endif

        map_p = dynamic_layer_map_;

        is_sub_dynamic_layer_ = true;
    }
    if(map_name == DEF_NO_ENTRY_LAYER_TOPIC_NAME)
    { // no entry layer
        
#if DEBUG
        ROS_INFO("DEBUG : No entry layer map");
#endif

        map_p = no_entry_layer_map_;

        is_sub_no_entry_layer_ = true;
    }

    map_p->header = msg->header;
    map_p->info = msg->info;

    map_p->data.resize(map_size);
    map_p->data = msg->data;

    
    
    if( is_published_sosio_map_ &&      // ソシオ地図が配信済み
        is_published_ego_map_ &&        // エゴ地図が配信済み
        is_sub_static_layer_ &&         // 静的レイヤ地図受信済み 
        is_sub_no_entry_layer_ &&       // 進入禁止レイヤ地図受信済み 
        is_sub_quasi_static_layer_      // 準静的レイヤ地図受信済み
    )
    { // 配信リセット処理
        is_published_ego_map_ = false;
        is_published_sosio_map_ = false;
    }

    return;
}

void MapMixer::mixing(LayerdMap *map)
{
    nav_msgs::OccupancyGrid buf_map;

    unsigned int map_size = map->getMapSize();

    buf_map.data.resize(map_size);
    map->mixed_maps_.data.resize(map_size);

    // 各レイヤ地図の障害物情報を静的レイヤ地図に上書きしていく
    buf_map = *static_layer_map_; // データのコピー

    // 地図の有効化フラグ
    bool is_act_qstaic_layer    = is_sub_quasi_static_layer_ && map->getQStaticLayerValid();
    bool is_act_dynamic_layer   = is_sub_dynamic_layer_      && map->getDynamicLayerValid();
    bool is_act_no_entry_layer  = is_sub_no_entry_layer_     && map->getNoEntryLayerValid();

    for(unsigned int i = 0; i < map_size; i++)
    {
        // 既に静的オブジェクトが存在する場合はスキップ
        if( buf_map.data[i] == OBSTACLE_COST_GRIDMAP )  // 100
        { // 不明領域や静的障害物領域は無視する
            continue;
        }

        // 占有確立100％の時は問答無用で障害物とする（静的障害物レイヤは既に反映済み）
        if( (is_act_no_entry_layer && no_entry_layer_map_->data[i] == OBSTACLE_COST_GRIDMAP ) ||
            (is_act_dynamic_layer && dynamic_layer_map_->data[i] >= dynamic_layer_occupansy_th_ ) || // 動的レイヤ地図の占有率1～99%を反映する場合も考慮
            (is_act_qstaic_layer && quasi_static_layer_map_->data[i] == OBSTACLE_COST_GRIDMAP ) )
        {
            buf_map.data[i] = OBSTACLE_COST_GRIDMAP;
        }
    }
    
    map->mixed_maps_ = buf_map;

    // エゴ地図とソシオ地図が配信済みなら、受信フラグをクリア
    if( is_published_ego_map_ &&
        is_published_sosio_map_
    )
    {
        is_sub_static_layer_        = false; // 静的レイヤ地図は1回しか配信されないことも有る
        is_sub_quasi_static_layer_  = false;
        is_sub_dynamic_layer_       = false;
        is_sub_no_entry_layer_      = false;
    }

}

void MapMixer::mixSocioMap(void)
{
    
    // ROS_INFO("run mixSocioMap");
    
    if( is_sub_static_layer_ &&         // 静的レイヤ地図受信済み 
        is_sub_quasi_static_layer_ &&   // 準静的レイヤ地図受信済み
        is_sub_no_entry_layer_ &&       // 進入禁止レイヤ地図受信済み
        !is_published_sosio_map_)       // ソシオ地図が未配信
    { // 一つでも存在しない時点でソシオ地図は作成しない
    
        ROS_INFO("run mixSosioMap");

        if(!is_published_sosio_map_)
        {   // 配信済みフラグが未配信の場合
            is_published_sosio_map_ = true;    // 配信済み
        }

        // 合成関数呼び出し
        mixing(socio_map_);

    
        
#if DEBUG
        ROS_INFO("DEBUG : pub mixSocioMap");
#endif

        // ソシオ地図パブリッシュ
        pub_socio_map_.publish(socio_map_->mixed_maps_);
    }


    return;
}

void MapMixer::mixEgoMap(const ros::TimerEvent&)
{
    nav_msgs::OccupancyGrid pub_map;

    if( is_sub_static_layer_ &&          // 静的レイヤ地図受信済み
        is_sub_quasi_static_layer_ &&    // 準静的レイヤ地図受信済み
        !is_published_ego_map_)           // エゴ地図が未配信
    { // 一つでも存在しない時点でエゴ地図は作成しない

        ROS_INFO("run mixEgoMap");

        if(!is_published_ego_map_)
        {   // 配信済みフラグが未配信の場合
            is_published_ego_map_ = true;    // 配信済み
        }
    
        // 合成関数呼び出し
        mixing(ego_map_);

#if DEBUG
        ROS_INFO("DEBUG : pub mixEgoMap");
#endif

        // エゴ地図パブリッシュ
        pub_ego_map_.publish(ego_map_->mixed_maps_);

        if(is_egomap_mix_only_once_)
        {
    
            ROS_INFO("stop mixEgoMap");
            egomap_mixing_timer_.stop();
        }
    }


    return;
}

void MapMixer::mixMapLoop(void)
{
    ros::Rate loop_rate = ROS_RATE_30HZ;
    // unsigned int cnt=0;
    while(ros::ok())
    {
        loop_rate.sleep();// スリープ
        ros::spinOnce();// コールバック処理

        mixSocioMap();
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "map_updater_node");
    ros::NodeHandle paramNode("~"), node;

    MapMixer mixer(paramNode, node);

    mixer.mixMapLoop();

    return (0);
}