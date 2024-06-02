# include "explorer_point_finder.h"

FrontierServer::FrontierServer(ros::NodeHandle& nh, FrontierClusterInfo::Ptr fc_info_ptr):nh(nh), fc_info_ptr_(fc_info_ptr){
    nh.param("frontier_node/max_vel", v_max, -1.0);
    nh.param("frontier_node/classic_pub_interval", classic_pub_interval, -1.0);
    nh.param("frontier_node/rapid_pub_interval", rapid_pub_interval, -1.0);
    nh.param("frontier_node/arrive_goal_dis_thres", arrive_goal_dis_thres_, -1.0);
    nh.param("frontier_node/arrive_cruise_goal_dis_thres", arrive_cruise_goal_dis_thres_, -1.0);
    nh.param("frontier_node/arrive_local_goal_dis_thres", arrive_local_goal_dis_thres_, -1.0);
    nh.param("map/init_x", _init_x,  0.0);
    nh.param("map/init_y", _init_y,  0.0);
    nh.param("map/init_z", _init_z,  1.0);
    
    nh.param("fsm/explore_start_x", explore_start_position_[0], 0.0);
    nh.param("fsm/explore_start_y", explore_start_position_[1], 0.0);
    nh.param("fsm/explore_start_z", explore_start_position_[2], 0.0);

    home_position_ << 0.0, 0.0, 1.0;

    nh.param("is_on_car", is_on_car, false);
    nh.param("is_coordinate", is_coordinate, false);
    nh.param("comm_range", comm_range, 999999.0);
    

    // m_nextAimPub = nh.advertise<nav_msgs::Odometry>("/next_aim", 1);
    m_nextAimPub = nh.advertise<geometry_msgs::PointStamped>("/next_aim", 1);

    trigger_sub_ = nh.subscribe("trigger", 1, &FrontierServer::triggerCallback, this);

    finish_sub = nh.subscribe("/finish_signal", 1, &FrontierServer::finishCallBack, this);

    state_str_ = { "OFF", "TOSTART", "RAPID", "CLASSIC", "GOHOME", "GOSAFEPOINT", "APPROACH", "FINISH" };
    m_currentState = ExplorationState::OFF;
    pub_log_.reset(new PubTicToc(1.5));

    last_check_time = std::chrono::high_resolution_clock::now();
    last_check_pos = Vector3d(0,0,0);


    MPG.reset(new MultiPoseGraph);
    MPG->main_inx = 0;

    last_pub_time = ros::Time::now();

    ros::Duration(1.0).sleep();

    frontier_timer_ = nh.createTimer(ros::Duration(0.05), &FrontierServer::runonce, this);

    // 多车交互相关
    mcar_FreeSpaceAndFrontierInfo_pub = nh.advertise<msg_utils::FreeSpaceAndFrontierInfo>("/mcar_FreeSpaceAndFrontierInfo", 1);
    old_svp_pub = nh.advertise<frontier_space::old_svp>("/mcar_old_svp", 1);
    mcar_FreeSpaceAndFrontierInfo_sub = nh.subscribe("/mcar_FreeSpaceAndFrontierInfo", 1, &FrontierServer::mcar_FreeSpaceAndFrontierInfoCallback, this);
    old_svp_sub = nh.subscribe("/mcar_old_svp", 1, &FrontierServer::old_svp_callback, this);
    comm2others_timer = nh.createTimer(ros::Duration(0.1), &FrontierServer::comm2others_once, this);
    comm2others_once_freespace_timer = nh.createTimer(ros::Duration(0.5), &FrontierServer::comm2others_once_freespace, this);
    // comm2others_once_frontiers_timer = nh.createTimer(ros::Duration(0.1), &FrontierServer::comm2others_once_frontiers, this);
    // mcar_FrontierInfo_sub = nh.subscribe("/mcar_FrontierInfo", 1, &FrontierServer::mcar_FrontierInfo_callback, this);
    mcar_FreeSpaceInfo_sub = nh.subscribe("/mcar_FreeSpaceInfo", 1, &FrontierServer::mcar_FreeSpaceInfo_callback, this);
    tour_sub = nh.subscribe("/grid_tour2", 1, &FrontierServer::tourCallback, this);
    mcar_FreeSpaceInfo_pub = nh.advertise<frontier_space::FSLAndFCLMsg>("/mcar_FreeSpaceInfo", 1);
    // 通信约束测试
    odom1_sub = nh.subscribe("/ant01/state_estimation", 1, &FrontierServer::odomCallback, this);
    odom2_sub = nh.subscribe("/ant02/state_estimation", 1, &FrontierServer::odomCallback, this);
    odom1_pos = Vector3d(0,0,0);
    odom2_pos = Vector3d(0,0,0);
    inx_of_freespace_pub = 0;
    inx_of_frontier_pub = 0;
}

void FrontierServer::tourCallback(const msg_utils::GridTourConstPtr msg) {
    //
    msg_utils::GridTour grid_tour_tmp;
    grid_tour_tmp = *msg;
    tour_positions.clear();
    tour_positions2.clear();
    // 读取tour信息，填充tour_positions
    for(int i = 0; i < grid_tour_tmp.points.size(); i++) {
        Vector3d pos_tmp;
        pos_tmp << grid_tour_tmp.points[i].x, grid_tour_tmp.points[i].y, grid_tour_tmp.points[i].z;
        tour_positions.push_back(pos_tmp);
    }
    for(int i = 0; i < grid_tour_tmp.points2.size(); i++) {
        Vector3d pos_tmp;
        pos_tmp << grid_tour_tmp.points2[i].x, grid_tour_tmp.points2[i].y, grid_tour_tmp.points2[i].z;
        tour_positions2.push_back(pos_tmp);
    }

}

void FrontierServer::odomCallback(const nav_msgs::OdometryConstPtr msg) {
    // ROS_WARN
    // ROS_WARN("odomCallback start");
    if(msg->pose.covariance[0] == 1) {
        odom1_pos = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    } else {
        odom2_pos = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    }
}

void FrontierServer::comm2others_once_freespace(const ros::TimerEvent& /*event*/) {
    return;
    // 测试一下通信约束，如果距离大于一定值，就不通信
    if((odom1_pos - odom2_pos).norm() > comm_range) {
        // 打印两个位置
        // ROS_WARN("odom1_pos: (%f, %f, %f)", odom1_pos(0), odom1_pos(1), odom1_pos(2));
        // ROS_WARN("odom2_pos: (%f, %f, %f)", odom2_pos(0), odom2_pos(1), odom2_pos(2));
        ROS_WARN("comm is tmino");
        return;
    }

    // 发送自由空间
    frontier_space::FreeSpaceInfoMsg free_space_info_msg_tmp;

    free_space_info_msg_tmp.drone_id_from = fc_info_ptr_->car_id;
    free_space_info_msg_tmp.last_send_inx = inx_of_freespace_pub;

    // 需要给每一个自由空间一个特征值，确保每一个只发送一次

    for(vector<FreeSpace>::iterator 
        it2 = fc_info_ptr_->free_space_info.free_space_list.begin(); 
        it2 != fc_info_ptr_->free_space_info.free_space_list.end(); it2++) {
        
        // 关键位姿填充
        geometry_msgs::Point key_pose_tmp;
        key_pose_tmp.x = it2->key_pose(0);
        key_pose_tmp.y = it2->key_pose(1);
        key_pose_tmp.z = it2->key_pose(2);
        free_space_info_msg_tmp.key_pose_list.push_back(key_pose_tmp);
        // 凸包顶点填充
        frontier_space::FreeSpaceMsg free_space_list_tmp;
        for(int i = 0; i < it2->convex_points->size(); i++) {
            geometry_msgs::Point convexhull_tmp;
            convexhull_tmp.x = it2->convex_points->at(i).x;
            convexhull_tmp.y = it2->convex_points->at(i).y;
            convexhull_tmp.z = it2->convex_points->at(i).z;
            free_space_list_tmp.convex_points.push_back(convexhull_tmp);
        }
        free_space_info_msg_tmp.free_space_list.push_back(free_space_list_tmp);
    }
    
    inx_of_freespace_pub++;

    frontier_space::FrontierClusterListMsg frontier_cluster_list_msg_tmp;
    frontier_cluster_list_msg_tmp.drone_id_from = fc_info_ptr_->car_id;
    for(list<SuperViewPoint>::iterator 
        it = fc_info_ptr_->svp_list.begin(); 
        it != fc_info_ptr_->svp_list.end(); it++) {
        // 关键位姿填充
        frontier_space::FrontierClusterMsg frontier_list_tmp;
/**
 *  int32 fc_id

    geometry_msgs/Point[] cell_list

    geometry_msgs/Point center
*/
        for(int i = 0; i < it->fc_list.size(); i++) {
            geometry_msgs::Point center_tmp;
            center_tmp.x = it->fc_list[i].center(0);
            center_tmp.y = it->fc_list[i].center(1);
            center_tmp.z = it->fc_list[i].center(2);
            geometry_msgs::Point vp_tmp;
            vp_tmp.x = it->fc_list[i].viewpoints[0].pos_.x();
            vp_tmp.y = it->fc_list[i].viewpoints[0].pos_.y();
            vp_tmp.z = it->fc_list[i].viewpoints[0].pos_.z();
            geometry_msgs::Point yaw_tmp;
            yaw_tmp.x = it->fc_list[i].viewpoints[0].yaw_;
            yaw_tmp.y = 0;
            yaw_tmp.z = 0;
            frontier_list_tmp.center = center_tmp;
            frontier_list_tmp.viewpoint = vp_tmp;
            frontier_list_tmp.yaw = yaw_tmp;
            // 如果距离自己较近，就不发送
            if((fc_info_ptr_->drone_pose - Vector3d(vp_tmp.x, vp_tmp.y, vp_tmp.z)).norm() < 2.0 * fc_info_ptr_->free_space_fac_->sensor_range) {
                continue;
            }
            for(int j = 0; j < it->fc_list[i].cell_list.size(); j++) {
                geometry_msgs::Point cell_tmp, cell_tmp2, cell_tmp3;
                cell_tmp.x = it->fc_list[i].cell_list[j]._a(0);
                cell_tmp.y = it->fc_list[i].cell_list[j]._a(1);
                cell_tmp.z = it->fc_list[i].cell_list[j]._a(2);
                cell_tmp2.x = it->fc_list[i].cell_list[j]._b(0);
                cell_tmp2.y = it->fc_list[i].cell_list[j]._b(1);
                cell_tmp2.z = it->fc_list[i].cell_list[j]._b(2);
                cell_tmp3.x = it->fc_list[i].cell_list[j]._c(0);
                cell_tmp3.y = it->fc_list[i].cell_list[j]._c(1);
                cell_tmp3.z = it->fc_list[i].cell_list[j]._c(2);
                frontier_list_tmp.cell_list.push_back(cell_tmp);
                frontier_list_tmp.cell_list.push_back(cell_tmp2);
                frontier_list_tmp.cell_list.push_back(cell_tmp3);
            }
            frontier_list_tmp.fc_id = it->fc_list[i].id;
        }

        frontier_cluster_list_msg_tmp.frontier_list.push_back(frontier_list_tmp);
    }
    // inx_of_frontier_pub++;
    // mcar_FrontierInfo_pub.publish(frontier_cluster_list_msg_tmp);
    frontier_space::FSLAndFCLMsg free_space_frontier_msg_tmp;
    free_space_frontier_msg_tmp.freeSpaceInfoMsg = free_space_info_msg_tmp;
    free_space_frontier_msg_tmp.frontierClusterListMsg = frontier_cluster_list_msg_tmp;
    mcar_FreeSpaceInfo_pub.publish(free_space_frontier_msg_tmp);

    // 同时发布一个删除列表

}

// void FrontierServer::comm2others_once_frontiers(const ros::TimerEvent& /*event*/) {
//     // 测试一下通信约束，如果距离大于一定值，就不通信
//     if((odom1_pos - odom2_pos).norm() > comm_range) {
//         // 打印两个位置
//         // ROS_WARN("odom1_pos: (%f, %f, %f)", odom1_pos(0), odom1_pos(1), odom1_pos(2));
//         // ROS_WARN("odom2_pos: (%f, %f, %f)", odom2_pos(0), odom2_pos(1), odom2_pos(2));
//         ROS_WARN("comm is tmino");
//         return;
//     }
//     frontier_space::FrontierClusterListMsg frontier_cluster_list_msg_tmp;
//     frontier_cluster_list_msg_tmp.drone_id_from = fc_info_ptr_->car_id;
//     for(list<SuperViewPoint>::iterator 
//         it = fc_info_ptr_->svp_list.begin(); 
//         it != fc_info_ptr_->svp_list.end(); it++) {
//         // 关键位姿填充
//         frontier_space::FrontierClusterMsg frontier_list_tmp;
// /**
//  *  int32 fc_id
//     geometry_msgs/Point[] cell_list
//     geometry_msgs/Point center
// */
//         for(int i = 0; i < it->fc_list.size(); i++) {
//             geometry_msgs::Point center_tmp;
//             center_tmp.x = it->fc_list[i].center(0);
//             center_tmp.y = it->fc_list[i].center(1);
//             center_tmp.z = it->fc_list[i].center(2);
//             frontier_list_tmp.center = center_tmp;
//             for(int j = 0; j < it->fc_list[i].cell_list.size(); j++) {
//                 geometry_msgs::Point cell_tmp, cell_tmp2, cell_tmp3;
//                 cell_tmp.x = it->fc_list[i].cell_list[j]._a(0);
//                 cell_tmp.y = it->fc_list[i].cell_list[j]._a(1);
//                 cell_tmp.z = it->fc_list[i].cell_list[j]._a(2);
//                 cell_tmp2.x = it->fc_list[i].cell_list[j]._b(0);
//                 cell_tmp2.y = it->fc_list[i].cell_list[j]._b(1);
//                 cell_tmp2.z = it->fc_list[i].cell_list[j]._b(2);
//                 cell_tmp3.x = it->fc_list[i].cell_list[j]._c(0);
//                 cell_tmp3.y = it->fc_list[i].cell_list[j]._c(1);
//                 cell_tmp3.z = it->fc_list[i].cell_list[j]._c(2);
//                 frontier_list_tmp.cell_list.push_back(cell_tmp);
//                 frontier_list_tmp.cell_list.push_back(cell_tmp2);
//                 frontier_list_tmp.cell_list.push_back(cell_tmp3);
//             }
//             frontier_list_tmp.fc_id = it->fc_list[i].id;
//         }
//         frontier_cluster_list_msg_tmp.frontier_list.push_back(frontier_list_tmp);
//     }
//     inx_of_frontier_pub++;
//     mcar_FrontierInfo_pub.publish(frontier_cluster_list_msg_tmp);
// }

// void FrontierServer::mcar_FrontierInfo_callback(const frontier_space::FrontierClusterListMsgConstPtr msg) {
// }

void FrontierServer::mcar_FreeSpaceInfo_callback(const frontier_space::FSLAndFCLMsgConstPtr msg) {

    // 以下是跳过条件，包括过于频繁的交互，自己发布的
    if(msg->freeSpaceInfoMsg.drone_id_from == fc_info_ptr_->car_id) return;
    if(msg->freeSpaceInfoMsg.key_pose_list.size() < 3) return;
    if(fc_info_ptr_->free_space_info.posegraph->getSize() < 3) return;

    
    frontier_space::FreeSpaceInfoMsg free_space_info_msg_tmp = msg->freeSpaceInfoMsg;
    frontier_space::FrontierClusterListMsg frontier_cluster_list_msg_tmp = msg->frontierClusterListMsg;
    // 读取自由空间信息，关于恢复哪些自由空间，检查keypose是否处于现在自己的keypose附近，如果太近了，就跳过
    // 1、检查是否有重复的keypose，如果有，就跳过
    // free_space_info_msg_tmp.last_send_inx 通过此控制接收频率
    // 打印关键位姿数量
    PoseGraph::Ptr posegraph_tmp;
    // 初始化
    posegraph_tmp.reset(new PoseGraph);
    ROS_WARN("free_space_info_msg_tmp.key_pose_list.size(): %i", free_space_info_msg_tmp.key_pose_list.size());
    // 打印所有的关键位置
    // for(int i = 0; i < free_space_info_msg_tmp.key_pose_list.size(); i++) {
    //     ROS_WARN("key_pose: (%f, %f, %f)", free_space_info_msg_tmp.key_pose_list[i].x, 
    //                                       free_space_info_msg_tmp.key_pose_list[i].y, 
    //                                       free_space_info_msg_tmp.key_pose_list[i].z);
    // }
    int num_too_close = 0;
    for(int i = 0; i < free_space_info_msg_tmp.key_pose_list.size(); i++) {
        // 检查这个位置自己是否已经到达
        pcl::PointXYZ key_pose_pcl(free_space_info_msg_tmp.key_pose_list[i].x, 
                                    free_space_info_msg_tmp.key_pose_list[i].y, 
                                    free_space_info_msg_tmp.key_pose_list[i].z);
        std::vector<int> pointIdxRadiusSearch;  
        std::vector<float> pointRadiusSquaredDistance; 

        if (fc_info_ptr_->free_space_info.posegraph->kdtree_keypose->radiusSearch(
                                    key_pose_pcl, 
                                    0.5*fc_info_ptr_->free_space_fac_->sensor_range, 
                                    pointIdxRadiusSearch, 
                                    pointRadiusSquaredDistance) > 0 
            // &&
            // !fc_info_ptr_->free_space_info.posegraph->kdtree_keypose->radiusSearch(
            //                         key_pose_pcl, 
            //                         1.5*fc_info_ptr_->free_space_fac_->sensor_range, 
            //                         pointIdxRadiusSearch, 
            //                         pointRadiusSquaredDistance) > 0 
                                    ) {

            // 如果这个位置和已经有的位置相距在一个传感器范围之内，跳过   
            // 每次跳过，打印一次提示
            ROS_WARN("keypose is too far, skip."); 
            // posegraph_tmp->key_pose_list->resize(0);
            num_too_close++;
            // continue;   
        } else {
            // ROS_WARN("keypose is very good, push back."); 
        // if(1) {
            // 如果这个位置还没去过，根据keypose和convexhull来重建别的机器人的已探索空间
            // 新的图需要和旧的合并起来
            FreeSpace fs_tmp;
            
            fs_tmp.key_pose = Vector3d(free_space_info_msg_tmp.key_pose_list[i].x,
                                        free_space_info_msg_tmp.key_pose_list[i].y,
                                        free_space_info_msg_tmp.key_pose_list[i].z);
            for(int j = 0; j < free_space_info_msg_tmp.free_space_list[i].convex_points.size(); j++) {
                pcl::PointXYZ convex_point_tmp;
                convex_point_tmp.x = free_space_info_msg_tmp.free_space_list[i].convex_points[j].x;
                convex_point_tmp.y = free_space_info_msg_tmp.free_space_list[i].convex_points[j].y;
                convex_point_tmp.z = free_space_info_msg_tmp.free_space_list[i].convex_points[j].z;
                fs_tmp.convex_points->push_back(convex_point_tmp);
            }
            // ROS_WARN("keypose is very good, i get the convex."); 
            
            std::vector<Vector3d> v;
            std::vector<size_t> inxs;
            fs_tmp.create_convexhull(v, inxs);
            // ROS_WARN("keypose is very good, i get the mesh."); 
            fc_info_ptr_->free_space_info.free_space_list.push_back(fs_tmp);
            // ROS_WARN("keypose is very good, i get the mesh2."); 
            // 把其他车的位置图合并进来，不能直接合并，需要重新构建
            
            pcl::PointXYZ key_pose_pcl2(free_space_info_msg_tmp.key_pose_list[i].x, 
                                    free_space_info_msg_tmp.key_pose_list[i].y, 
                                    free_space_info_msg_tmp.key_pose_list[i].z);
            // 打印一下key_pose_pcl2
            ROS_WARN("key_pose_pcl2: (%f, %f, %f)", key_pose_pcl2.x, key_pose_pcl2.y, key_pose_pcl2.z);
            posegraph_tmp->push_back(key_pose_pcl2);
            // fc_info_ptr_->temp_num++;
            // ROS_WARN("keypose is very good, kd tree renew start."); 
            
            // ROS_WARN("keypose is very good, kd tree renew."); 
            // if(fc_info_ptr_->free_space_info.free_space_list.size() > fc_info_ptr_->frontier_gen_frenq 
            //     && fc_info_ptr_->temp_num % fc_info_ptr_->frontier_gen_frenq == 0){
            //     for(int k = 0; k < fc_info_ptr_->frontier_gen_frenq; k++){
            //         fc_info_ptr_->free_space_info.free_space_list.pop_back();
            //         fc_info_ptr_->free_space_info.posegraph->erase_back();
            //     }
            //     fc_info_ptr_->free_space_info.free_space_list.push_back(fs_tmp);
            //     fc_info_ptr_->free_space_info.posegraph->push_back(key_pose_pcl2);
            //     fc_info_ptr_->temp_num = 0;
                // fc_info_ptr_->addPoseEdge();
            // }
        }
        // ROS_WARN("free_space_info_msg_tmp.key_pose_list check end.%i, %i", free_space_info_msg_tmp.key_pose_list.size(), num_too_close);
        if(num_too_close == free_space_info_msg_tmp.key_pose_list.size()) {
            ROS_WARN("num_too_close: %i", num_too_close);
            return;
        }
    }
    ROS_WARN("free_space_info_msg_tmp.key_pose_list check end.%i", posegraph_tmp->getSize());
    // 打印警告，开始合并
    // if(posegraph_tmp->getSize() <= 0) {
        // ROS_WARN("posegraph_tmp size is 0, return.");
        // return;
    // }
    ROS_WARN("merge other car's posegraph and free space.");
    // 把另一个车的位置图合并进本车里边
    posegraph_tmp->renewKdtree();
    // 首先把上面帅选出来的关键位置加进去
    int ori_num = fc_info_ptr_->free_space_info.posegraph->getSize(); // 当前的关键位姿数量
    // 去除key_pose_list的最后一点
    // pcl::PointXYZ last_key_pose = fc_info_ptr_->free_space_info.posegraph->key_pose_list->back();
    // fc_info_ptr_->free_space_info.posegraph->erase_back();
    for(int k = 0; k < posegraph_tmp->getSize(); k++) {
        fc_info_ptr_->free_space_info.posegraph->push_back(posegraph_tmp->getCor(k));
    }
    // 在此基础上再加边
    for(int k = 0; k < posegraph_tmp->getSize(); k++) {
        // fc_info_ptr_->free_space_info.posegraph->push_back(posegraph_tmp->getCor(k));
        pcl::PointXYZ q_p = posegraph_tmp->getCor(k);
        // 打印q_p
        ROS_WARN("q_p: (%f, %f, %f)", q_p.x, q_p.y, q_p.z);
        map<int, double> res_set;
        ROS_WARN("search start.");
        // fc_info_ptr_->free_space_info.posegraph->getPotenConnectSetSelf(fc_info_ptr_->free_space_info.posegraph->getSize()-1, 
        //                                                                 res_set, 0.5*fc_info_ptr_->free_space_fac_->sensor_range, 
        //                                                                 2.0*fc_info_ptr_->free_space_fac_->sensor_range);
        fc_info_ptr_->free_space_info.posegraph->renewKdtree();
        fc_info_ptr_->free_space_info.posegraph->getPotenConnectSetSelf(ori_num + k, res_set, 0.5*fc_info_ptr_->free_space_fac_->sensor_range, 0.1, 6.0);
        // fc_info_ptr_->free_space_info.posegraph->getPotenConnectSetB(q_p, res_set, 0.5*fc_info_ptr_->free_space_fac_->sensor_range, 
        //                                                                             2.0*fc_info_ptr_->free_space_fac_->sensor_range);
        ROS_WARN("search end.");
        for(map<int, double>::iterator iter = res_set.begin(); iter != res_set.end(); iter++){
            pcl::PointXYZ endp = fc_info_ptr_->free_space_info.posegraph->getCor(iter->first);
            ROS_WARN("addPoseEdge: %i, %i, %f", k, iter->first, iter->second);
            Vector3d end_p(endp.x, endp.y, endp.z);
            Vector3d cur_p(q_p.x, q_p.y, q_p.z);
            // 应该删除原先所有的边，然后重新添加
            if(fc_info_ptr_->isVisible(cur_p, end_p, 0.0)){
            fc_info_ptr_->free_space_info.posegraph->addPoseEdge(ori_num + k, iter->first, iter->second);}
        }
    }

// void FrontierClusterInfo::addPoseEdge(){
//     map<int, double> nearby_v_set;
//     int inx2 = free_space_info.posegraph->getSize()-1;
//     pcl::PointXYZ cur = free_space_info.posegraph->getCor(inx2);
//     Vector3d cur_p(cur.x, cur.y, cur.z);
//     free_space_info.posegraph->renewKdtree();
//     free_space_info.posegraph->getPotenConnectSetSelf(inx2, nearby_v_set, 0.5*free_space_fac_->sensor_range, 2.0*free_space_fac_->sensor_range);
//     for(map<int, double>::iterator it = nearby_v_set.begin(); it != nearby_v_set.end(); it++){
//         pcl::PointXYZ endp = free_space_info.posegraph->getCor(it->first);
//         Vector3d end_p(endp.x, endp.y, endp.z);
//         if(isVisible(cur_p, end_p, 0.0)){
//             free_space_info.posegraph->addPoseEdge(it->first, inx2, it->second);
//         }
//     }
// }
    // 再把末尾元素加回去
    // fc_info_ptr_->free_space_info.posegraph->push_back(last_key_pose);
    fc_info_ptr_->free_space_info.posegraph->renewKdtree();

    fc_info_ptr_->pubPoseEdges(ros::Time::now());
    ROS_WARN("merge other car's posegraph and free space end.");

    ROS_WARN("build keypose graph and free space successful.");
    // 打印一下free_space_info中的自由空间列表长度以及对应的id
    ROS_WARN("free_space_info car id(): %i", fc_info_ptr_->car_id);
    ROS_WARN("free_space_info.free_space_list.size(): %i", fc_info_ptr_->free_space_info.free_space_list.size());

return;

    // 读取视点信息，需要自行计算恢复视点
    // 读取前言聚类列表的中心以及对应的网格
    // 对应相当于从Cluster.cpp的第319行开始
    for(int i = 0; i < frontier_cluster_list_msg_tmp.frontier_list.size(); i++) {
        FrontierCluster fc_tmp;
        // 聚类中心位置
        // 如果聚类中心位于已经到访过的地方，就跳过
        pcl::PointXYZ key_pose_pcl(frontier_cluster_list_msg_tmp.frontier_list[i].center.x, 
                                    frontier_cluster_list_msg_tmp.frontier_list[i].center.y,
                                    frontier_cluster_list_msg_tmp.frontier_list[i].center.z);
        std::vector<int> pointIdxRadiusSearch;  
        std::vector<float> pointRadiusSquaredDistance; 
        // if (fc_info_ptr_->free_space_info.posegraph->kdtree_keypose->radiusSearch(
        //                             key_pose_pcl, 
        //                             1.4*fc_info_ptr_->free_space_fac_->sensor_range, 
        //                             pointIdxRadiusSearch, 
        //                             pointRadiusSquaredDistance) > 0 ) {
        //     //
        // } else {
        if(1) {
            for(int j = 0; j < frontier_cluster_list_msg_tmp.frontier_list[i].cell_list.size(); j+=3) {
                geometry_msgs::Polygon mesh_tmp;
                mesh_tmp.points.resize(3);
                mesh_tmp.points[0].x = frontier_cluster_list_msg_tmp.frontier_list[i].cell_list[j].x;
                mesh_tmp.points[0].y = frontier_cluster_list_msg_tmp.frontier_list[i].cell_list[j].y;
                mesh_tmp.points[0].z = frontier_cluster_list_msg_tmp.frontier_list[i].cell_list[j].z;
                mesh_tmp.points[1].x = frontier_cluster_list_msg_tmp.frontier_list[i].cell_list[j+1].x;
                mesh_tmp.points[1].y = frontier_cluster_list_msg_tmp.frontier_list[i].cell_list[j+1].y;
                mesh_tmp.points[1].z = frontier_cluster_list_msg_tmp.frontier_list[i].cell_list[j+1].z;
                mesh_tmp.points[2].x = frontier_cluster_list_msg_tmp.frontier_list[i].cell_list[j+2].x;
                mesh_tmp.points[2].y = frontier_cluster_list_msg_tmp.frontier_list[i].cell_list[j+2].y;
                mesh_tmp.points[2].z = frontier_cluster_list_msg_tmp.frontier_list[i].cell_list[j+2].z;
                Triangle tri_tmp(mesh_tmp);
                fc_tmp.addCell(tri_tmp);
                // fc_tmp.cell_list.push_back(tri_tmp);
            }
            ROS_WARN("fc_tmp update.");
            fc_tmp.update();
            ROS_WARN("fc_tmp vp sample.");
            // 视点改为直接传过来，不要采样了
            Vector3d sample_pos = Vector3d(frontier_cluster_list_msg_tmp.frontier_list[i].viewpoint.x, 
                                           frontier_cluster_list_msg_tmp.frontier_list[i].viewpoint.y,
                                           frontier_cluster_list_msg_tmp.frontier_list[i].viewpoint.z);
            double yaw = frontier_cluster_list_msg_tmp.frontier_list[i].yaw.x;
            Viewpoint vp2 = { sample_pos, yaw, AIR };
            fc_tmp.viewpoints.push_back(vp2);
            // fc_info_ptr_->sampleViewpoints2(fc_tmp);
            // fc_info_ptr_->frontier_cluster_list.push_back(fc_tmp);
            ROS_WARN("fc_tmp add to list.");
            fc_info_ptr_->addCluster(fc_tmp);
        }
        // fc_tmp.center = Vector3d(frontier_cluster_list_msg_tmp.frontier_list[i].center.x, 
        //                          frontier_cluster_list_msg_tmp.frontier_list[i].center.y,
        //                          frontier_cluster_list_msg_tmp.frontier_list[i].center.z);

    }
    ROS_WARN("get frontier cluster lists successful.");
    fc_info_ptr_->addSuperPoint2();

    // 加一个距离检查，如果距离太近，就不要了
    ROS_WARN("build svp successful.");
    // 打印所有的svp
    ROS_WARN("svp_list.size(): %i, %i", fc_info_ptr_->svp_list.size(), fc_info_ptr_->car_id);
    for(list<SuperViewPoint>::iterator it = fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++) {
        ROS_WARN("svp id: %i", it->id);
    }

    // 此时发布自己的超级视点列表
}

// 接受超级视点列表，检查后发布删除列表

// 接受删除列表，检查后删除自己的超级视点列表



void FrontierServer::comm2others_once(const ros::TimerEvent& /*event*/) {
    // return;

    // 测试一下通信约束，如果距离大于一定值，就不通信
    if((odom1_pos - odom2_pos).norm() > comm_range) {
        // 打印两个位置
        // ROS_WARN("odom1_pos: (%f, %f, %f)", odom1_pos(0), odom1_pos(1), odom1_pos(2));
        // ROS_WARN("odom2_pos: (%f, %f, %f)", odom2_pos(0), odom2_pos(1), odom2_pos(2));
        ROS_WARN("comm is tmino");
        return;
    }

    // ROS_WARN("comm2others_once start");
    // 定期发布自车的信息
    // return;

    // if(m_currentState == ExplorationState::)

    if(fc_info_ptr_->svp_list.size() <= 0 || fc_info_ptr_->free_space_info.free_space_list.size() <= 0 || fc_info_ptr_->free_space_info.posegraph->key_pose_list->size() <= 0) 
        return;

    msg_utils::FreeSpaceAndFrontierInfo cur_exp_state;  // 自由空间与前沿信息汇总消息
    msg_utils::FreeSpaceInfo cur_free_space_info;  // 自由空间消息
    msg_utils::FrontierInfo cur_frontier_info;  // 前沿消息

    /****自由空间相关消息填充*****************************************************************************************************************************************/
    for( vector<FreeSpace>::iterator it2 = fc_info_ptr_->free_space_info.free_space_list.begin(); it2 != fc_info_ptr_->free_space_info.free_space_list.end(); it2++) {
        msg_utils::FreeSpace cur_free_space;
        for( int i = 0; i < it2->meshes.size(); i++ ) {
            geometry_msgs::Point cur_mesh;
            cur_mesh.x = it2->meshes[i]._a.x;
            cur_mesh.y = it2->meshes[i]._a.y;
            cur_mesh.z = it2->meshes[i]._a.z;
            cur_free_space.convex_points.push_back(cur_mesh);
        }
        cur_free_space_info.free_space_list.push_back(cur_free_space);
    }

    /**
     *      FreeSpace fs;
            fs = free_space_fac_->genFreeSpcePoints(drone_pose);    填充了keypose和convex_points
            std::vector<Vector3d> v;
            std::vector<size_t> inxs;
            fs.create_convexhull(v, inxs);  构建一下网格

            pcl::PointXYZ pt(drone_pose(0), drone_pose(1), drone_pose(2));
            free_space_info.free_space_list.push_back(fs);
            free_space_info.posegraph->push_back(pt);  填充空间列表和位姿图
            addPoseEdge();

            综上，关键是发布填充了keypose和convex_points，其余的再构建就行
    */

    // 不直接填充位姿图，改成接受自由空间信息，重新增量式构建位姿图
    // // cout << "comm1"  << endl;
    // msg_utils::Posegra cur_posegra;
    // for(int jj = 0; jj < fc_info_ptr_->free_space_info.posegraph->key_pose_list->size(); jj++) {
    //     geometry_msgs::Point cur_pose;
    //     cur_pose.x = (*fc_info_ptr_->free_space_info.posegraph->key_pose_list)[jj].x;
    //     cur_pose.y = (*fc_info_ptr_->free_space_info.posegraph->key_pose_list)[jj].y;
    //     cur_pose.z = (*fc_info_ptr_->free_space_info.posegraph->key_pose_list)[jj].z;
    //     cur_posegra.key_pose_list.push_back(cur_pose);
    // }
    // cur_free_space_info.posegraph.key_pose_list = cur_posegra.key_pose_list;
    // // cout << "comm2"  << endl;
    // msg_utils::EdgeTable cur_edge_table;
    // for( int i = 0; i < fc_info_ptr_->free_space_info.posegraph->pose_edge.size(); i++ ) {
    //     msg_utils::Edge cur_edge;
    //     // cur_edge.dis = fc_info_ptr_->free_space_info.posegraph.pose_edge[i]
    //     cur_edge.dis = fc_info_ptr_->free_space_info.posegraph->pose_edge[i].begin()->weight;
    //     cur_edge.key_pose_inx_to = fc_info_ptr_->free_space_info.posegraph->pose_edge[i].begin()->v_inx;
    //     cur_edge.key_pose_inx_from = i;
    //     cur_edge_table.data.push_back(cur_edge);
    // }
    // // cout << "comm3"  << endl;
    // cur_free_space_info.posegraph.edges_in_PG = cur_edge_table;

    /**** 前沿相关消息填充******************************************************************************************************************************************/
    for( list<SuperViewPoint>::iterator it = fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++) {
        // 填充超级视点
        msg_utils::SuperViewPoint cur_svp;

        cur_svp.robot_id = fc_info_ptr_->car_id;

        cur_svp.super_viewpoin_id = it->id;

        geometry_msgs::Point cur_svp_viewpoint;
        cur_svp_viewpoint.x = it->super_vp.x;
        cur_svp_viewpoint.y = it->super_vp.y;
        cur_svp_viewpoint.z = it->super_vp.z;
        cur_svp.super_viewpoint = cur_svp_viewpoint;
        // cout << "comm3.1"  << endl;

        for(int i = 0; i < it->fc_list.size(); i++) {
            // cout << "comm_a"  << endl;
            // 填充视点
            msg_utils::ViewPoint cur_viewpoint;
            cur_viewpoint.viewpoint_id = it->fc_list[i].id;
            geometry_msgs::Point frontier_cluster;
            frontier_cluster.x = it->fc_list[i].center(0);
            frontier_cluster.y = it->fc_list[i].center(1);
            frontier_cluster.z = it->fc_list[i].center(2);
            cur_viewpoint.frontier_cluster = frontier_cluster;
            // cout << "comm_b"  << endl;
            for(int j = 0; j < it->fc_list[i].cell_list.size(); j++) {
                // cout << "comm_c"  << endl;
                geometry_msgs::Polygon cur_cell;
                cur_cell.points.resize(3);
                cur_cell.points[0].x = it->fc_list[i].cell_list[j]._a(0);
                cur_cell.points[0].y = it->fc_list[i].cell_list[j]._a(1);
                cur_cell.points[0].z = it->fc_list[i].cell_list[j]._a(2);
                cur_cell.points[1].x = it->fc_list[i].cell_list[j]._b(0);
                cur_cell.points[1].y = it->fc_list[i].cell_list[j]._b(1);
                cur_cell.points[1].z = it->fc_list[i].cell_list[j]._b(2);
                cur_cell.points[2].x = it->fc_list[i].cell_list[j]._c(0);
                cur_cell.points[2].y = it->fc_list[i].cell_list[j]._b(1);
                cur_cell.points[2].z = it->fc_list[i].cell_list[j]._b(2);
                cur_viewpoint.cell_list.push_back(cur_cell);
                // cout << "comm_d"  << endl;
            }

            cur_svp.viewpoints.push_back(cur_viewpoint);
        }
        // cout << "comm3.2"  << endl;

        cur_frontier_info.super_viewpoints.push_back(cur_svp);
    }
    // cout << "comm4"  << endl;
    // cur_exp_state.robot_id = fc_info_ptr_->car_id;
    cur_exp_state.from_id = fc_info_ptr_->car_id;
    cur_exp_state.free_space_info = cur_free_space_info;
    cur_exp_state.frontier_info = cur_frontier_info;
    mcar_FreeSpaceAndFrontierInfo_pub.publish(cur_exp_state);

    // ROS_WARN("comm2others_once end");
}

void FrontierServer::mcar_FreeSpaceAndFrontierInfoCallback(const msg_utils::FreeSpaceAndFrontierInfoConstPtr msg) {

    // sleep(3.0);

    // ROS_WARN("mcar_FreeSpaceAndFrontierInfoCallback start");

    // 先使用最简单的思路
    // 1、把别人的超级视点信息存起来，如果发现其中某个是自己曾经到过附近的，就把这个点发布出去
    // 2、接受别车发布的已到达点，如果发现是自己的发的，就把这个点从自己的表中删除

    // 接受其他车的探索情况，首先是排除自己的
    if(msg->from_id == fc_info_ptr_->car_id) return;

    // 接受并合并其他车辆的探索信息
    FreeSpaceInfo fs_info_tmp;
    SuperViewPointList svp_list_tmp;

    /*******消息解析*********************************************************************/
    // 获取临时超级视点信息
    // ROS_WARN("获取临时超级视点信息 start");
    for(int i = 0; i < msg->frontier_info.super_viewpoints.size(); i++) { 
        SuperViewPoint svp_tmp;
        // svp_tmp.super_vp.
        svp_tmp.id = msg->frontier_info.super_viewpoints[i].super_viewpoin_id;
        svp_tmp.super_vp.x = msg->frontier_info.super_viewpoints[i].super_viewpoint.x;
        svp_tmp.super_vp.y = msg->frontier_info.super_viewpoints[i].super_viewpoint.y;
        svp_tmp.super_vp.z = msg->frontier_info.super_viewpoints[i].super_viewpoint.z;

        // for(int j = 0; j < msg->frontier_info.super_viewpoints[i].viewpoints.size(); j++) {
        //     FrontierCluster fc_tmp;
            
        //     for(int k = 0; k < msg->frontier_info.super_viewpoints[i].viewpoints[j].cell_list.size(); k++) {
        //         geometry_msgs::Polygon poly_tmp;
        //         poly_tmp = msg->frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k];
        //         Triangle tri_tmp(poly_tmp);
        //         fc_tmp.cell_list.push_back(tri_tmp);
        //     }
        //     svp_tmp.fc_list.push_back(fc_tmp);
        // }

        svp_list_tmp.push_back(svp_tmp);
    }
    // ROS_WARN("获取临时超级视点信息 end");

    // 打印svp_list_tmp
    // ROS_WARN("打印svp_list_tmp start");
    // ROS_WARN("ID: %i", msg->from_id+1);
    // for(SuperViewPointList::iterator it = svp_list_tmp.begin(); it != svp_list_tmp.end(); it++) {
    //     ROS_INFO("SuperViewPoint ID: %d", it->id);
    //     ROS_INFO("SuperViewPoint Position: (%f, %f, %f)", it->super_vp.x, it->super_vp.y, it->super_vp.z);
    // }
    // ROS_WARN("打印svp_list_tmp end");
    

    // 逐个检查svp_list_tmp中的点是否在自己的空间中
    frontier_space::old_svp oldSVP;
    oldSVP.svp_source_robot = msg->from_id;
    oldSVP.svp_check_robot = fc_info_ptr_->car_id;

// ROS_WARN("逐个检查svp_list_tmp start");
    // if(fc_info_ptr_->free_space_info.posegraph->getSize() >= 1){
        if(fc_info_ptr_->free_space_info.posegraph->getSize() < 2) return;

        for(SuperViewPointList::iterator it = svp_list_tmp.begin(); it != svp_list_tmp.end(); it++) {
            // bool deleted = false;
            pcl::PointXYZ searchPoint(it->super_vp.x, it->super_vp.y, it->super_vp.z);
            std::vector<int> pointIdxRadiusSearch;  
            std::vector<float> pointRadiusSquaredDistance; 
            // ROS_WARN("check radiusSearch start");
            if ( fc_info_ptr_->free_space_info.posegraph->kdtree_keypose->radiusSearch(searchPoint, 1.0*fc_info_ptr_->free_space_fac_->sensor_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                // for (size_t k = 0; k < pointIdxRadiusSearch.size (); ++k){
                    // ROS_WARN("check isMeshInFreeSapce start");
                    // if(pointIdxRadiusSearch[k] == fc_info_ptr_->free_space_info.posegraph->getSize()-1) continue;
                    // Vector3d cen = Vector3d(it->super_vp.x, it->super_vp.y, it->super_vp.z);
                    // if(fc_info_ptr_->isMeshInFreeSapce(fc_info_ptr_->free_space_info.free_space_list[pointIdxRadiusSearch[k]], cen)){
                        // 曾经到过，需要标记删除
                        // ROS_WARN("shanchu start");
                        oldSVP.old_svp_list.push_back(it->id);
                    // }
                // }
            }
        }


    // }
    // ROS_WARN("逐个检查svp_list_tmp end");

    if((odom1_pos - odom2_pos).norm() > comm_range) {
        ROS_WARN("comm is tmino 2");
        return;
    }
    old_svp_pub.publish(oldSVP);

    // ROS_WARN("mcar_FreeSpaceAndFrontierInfoCallback end");

}

void FrontierServer::old_svp_callback(const frontier_space::old_svpConstPtr msg) {
    if(!is_coordinate) return;
    // ROS_WARN("old_svp_callback start");
    // 接受oldsvp消息，如果是自己的，跳过；如果souce不是自己跳过
    if(msg->svp_check_robot == fc_info_ptr_->car_id) return;
    if(msg->svp_source_robot != fc_info_ptr_->car_id) return;

    // 检查这个列表，检查和自己的svp_list的区别，如果其中出现了自己还保存着的，那么删除他。
    // 但是注意id_pool的回收导致的编号重复出现，已删除编号回收
    for(list<SuperViewPoint>::iterator it = fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end();){
        bool is_affected = false;
        for(int i = 0; i < msg->old_svp_list.size(); i++) {
            if(msg->old_svp_list[i] == it->id) {
                is_affected = true;
                // 再加一个限制，如果要删除的点据离自己很近，跳过，只删除远的
                // 这个限制导致后到达的还是要访问全部的地区，不妙
                // 还有一个策略，既是如果没有待访问点，就沿着前车的轨迹走，一旦发现有带访问点，就迅速转为前往SVP 
                // 
                // if((Vector3d(it->super_vp.x, it->super_vp.y, it->super_vp.z) - fc_info_ptr_->drone_pose).norm() < 1.1*fc_info_ptr_->free_space_fac_->sensor_range) {
                //     it++;
                //     break;
                // }
                //
                ROS_WARN("it->id %i is delet",it->id);
                it = fc_info_ptr_->eraseSuperPoint(it);
                
                break;
            }
        }
        if(!is_affected){
            it++;
        }
    }

    // 如果此时没有待访问点，直接跟前车，把前车位置发布为目标点
    // geometry_msgs::PointStamped odom;
    // odom.header.stamp = ros::Time::now();
    // odom.header.frame_id = "map";

    // if(fc_info_ptr_->car_id == 0) {
    //     if(fc_info_ptr_->svp_list.size() <= 0) {
    //         odom.point.x = odom2_pos(0);
    //         odom.point.y = odom2_pos(1);
    //         odom.point.z = odom2_pos(2);
    //         m_nextAimPub.publish(odom);
    //     }
    // } else {
    //     if(fc_info_ptr_->svp_list.size() <= 0) {
    //         odom.point.x = odom1_pos(0);
    //         odom.point.y = odom1_pos(1);
    //         odom.point.z = odom1_pos(2);
    //         m_nextAimPub.publish(odom);
    //     }
    // }


    // ROS_WARN("old_svp_callback end");

}

void FrontierServer::triggerCallback(const geometry_msgs::PoseStampedConstPtr msg) {
    if (m_currentState != ExplorationState::OFF ) {
        // ROS_WARN_STREAM("wait for triggered!!");   
        return;
    }
    trigger_ = true;
    ROS_WARN_STREAM("start exploter");

    last_pub_time = ros::Time(ros::Time::now().toSec() - 10.0);
    transitState(TOSTART);

    start_time = ros::Time::now().toSec();
}

void FrontierServer::finishCallBack(const std_msgs::Int8 msg)
{
    ROS_ERROR_STREAM("Get Finish Signal!");
    need_finish = true;
    return;
}


//! Note! This timer is only for single robot exploration FSM.
//! For multi-robot exploration, please comment this timer!
void FrontierServer::runonce(const ros::TimerEvent& /*event*/)
{    
    if (m_currentState == FINISH)
    {
        ROS_INFO_THROTTLE(1.0, "\033[1;31mFinish!\033[0m");
        return;
    }
    if(m_currentState == CLASSIC)
   { ROS_ERROR_STREAM("[frontier_finder] State: " << stateStr(m_currentState));
    cout<<"------------------------run once-----------------------------"<<endl;}
    // ROS_INFO_STREAM("[frontier_finder] State: " << stateStr(m_currentState));
        
// cout<<"------------------------updateFrontierSpace s-----------------------------"<<endl;
    if(m_currentState != GOHOME && m_currentState != FINISH) 
        fc_info_ptr_->updateFrontierSpace();
// cout<<"------------------------updateFrontierSpace e-----------------------------"<<endl;
    // return;

    if(First_frame_cnt++ < 20 || fc_info_ptr_->free_space_fac_->grid_map_->recieve_cnt <= 10) {
        aim_pose = Vector3d(0,0,1);
        cout<<"aim_pose01: "<<aim_pose.transpose()<<endl;
        return;
    }

    fc_info_ptr_->free_space_info.posegraph->deepCopy(MPG->posegraphes[0], fc_info_ptr_->free_space_info.posegraph->getSize()-fc_info_ptr_->temp_num);

    double time1 = ros::Time::now().toSec();

    //! Result container
    Vector3d aim_p = Vector3d(aim_pose(0), aim_pose(1), aim_pose(2));
    double dis2goal;

    Vector3d drone_pos_ = fc_info_ptr_->drone_pose;
    vector<Vector3d> classic_aim;

    Vector3d safe_pose;

    ros::Time ros_time = ros::Time::now();

    Vector3d cur_pos = fc_info_ptr_->drone_pose;

    // if(m_currentState != FINISH && m_currentState != TOSTART && m_currentState != GOHOME && m_currentState != OFF && m_currentState != RAPID) {

    //     if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - last_check_time).count() > 10000) {
    //         if(fc_info_ptr_->svp_list.size() <= 0) return;

    //         // if((last_check_pos - cur_pos).norm() < 0.6 && !fc_info_ptr_->isVisible(cur_pos, Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z), fc_info_ptr_->grid_map_->getResolution())) {
    //         if((last_check_pos - cur_pos).norm() < 0.3) {
    //             ROS_ERROR_STREAM("[frontier_finder] Not wait_planner_succ!");

    //             for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
    //             {
    //                 if (best_svp.id == it->id && (Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z) - aim_pose).norm() < 1.0)
    //                 {
    //                     fc_info_ptr_->svp_list.erase(it);
    //                     ROS_ERROR_STREAM("[frontier_finder] erase it!");
    //                     break;
    //                 }

    //             }
    //         }
    //         last_check_time = std::chrono::high_resolution_clock::now();
    //         last_check_pos = cur_pos;
    //     }
    // }

    if(m_currentState == CLASSIC)
    ROS_ERROR_STREAM("[frontier_finder] State: " << stateStr(m_currentState));

    switch (m_currentState)
    {

    case FINISH: {
        ROS_INFO_THROTTLE(1.0, "\033[1;31mFinish!\033[0m");
        break;
    }

    case TOSTART: {
        ROS_INFO("\033[1;31mTOSTART Mode Active!\033[0m");  //红
        double dis2localaim = (explore_start_position_ - fc_info_ptr_->drone_pose).head(2).norm();
        if(dis2localaim < arrive_cruise_goal_dis_thres_)
        {
            ROS_INFO("\033[1;31mTOSTART Done! -  %f s\033[0m",(ros::Time::now().toSec()-start_time));  //红
            transitState(CLASSIC);
            break;
        }

        if ((ros_time - last_pub_time).toSec() > 10.0)
        {
            publishNextAim(ros_time, explore_start_position_, Vector3d::Zero());
            cout << "[TOSTART] pub start aim: " << explore_start_position_.transpose() << endl;
            last_pub_time = ros_time;
        }
        break;
    }

    case RAPID: {
        ROS_INFO("\033[1;31mRAPID Mode Active!\033[0m");  //红
        transitState(CLASSIC);
        break;
    }

    case CLASSIC: {
        // if(fc_info_ptr_->svp_list.size() <= 0) {
        //     ROS_ERROR("no svp, stop and find");
        //     break;
        // }
        ROS_INFO("\033[1;31mClassic Mode Active!\033[0m");  //红
        vector<Vector3d> path_res_tmp = path_res;
        path_res.clear();
        SuperViewPoint best_svp_last = best_svp;
        best_svp = classicFronitierFind(aim_pose, aim_vel); // will fill the path_res
        ROS_ERROR("best_svp.id: %i", best_svp.id);
            
        if(best_svp.id == -1){
            ROS_ERROR_STREAM("classFind Fail! cnt: " << classic_fail_cnt_);
            classic_fail_cnt_++;
            if (classic_fail_cnt_ < 20) {
                break;
            } 
            //  TODO
            //  如果暂且没有找到新的svp，就去依照task_allocation的走法
            // 根据地图判断是否是回家
            // 打印tour_positions2和tour_positions的长度
            cout << "tour_positions2 size: " << tour_positions2.size() << endl;
            cout << "tour_positions size: " << tour_positions.size() << endl;
            ROS_INFO("\033[1;31mtour_positions size, tour_positions size2 \033[0m", tour_positions.size(),  tour_positions2.size());  //红

            if(tour_positions2.size() > 2) {
                transitState(RAPID);
                // 如果任务分配器还能给出目标点，就继续走
                path_res.clear();   
                for(int i = 1; i < tour_positions.size(); i++) {
                    // 计算距离
                    double dis = (fc_info_ptr_->drone_pose - tour_positions[i]).head(2).norm();
                    if(dis > 2.0) {
                        aim_pose = tour_positions[i];
                        // path_res.push_back(tour_positions[i]);
                        break;
                    }
                    aim_pose = tour_positions[i];
                }


                // aim_pose = tour_positions[1];
                path_res.push_back(aim_pose);

                // 假设path_res已经存储了可达点
            } else {
                transitState(GOHOME);
                path_res.clear();
                bool plan_home_success = false;
                if((fc_info_ptr_->drone_pose - home_position_).head(2).norm() > 0.1){
                    aim_pose = home_position_;
                    local_aim = aim_pose;
                    if (!fc_info_ptr_->grid_map_->getInflateOccupancy(aim_pose)) 
                    {
                        getPathHome(home_position_, fc_info_ptr_->drone_pose, path_res);
                        plan_home_success = true;
                        ROS_ERROR("plan_home_success!");
                    }
                }
                if (!plan_home_success) break;
            }

        } else {
            ROS_ERROR_STREAM("classFind Success!" );
            classic_fail_cnt_ = 0;
            bool fc_changed = true;
            double dis = (Vector3d(best_svp_last.super_vp.x, best_svp_last.super_vp.y, best_svp_last.super_vp.z) - 
                Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z)).norm();
            if(dis < 0.1)
            {
                fc_changed = false;
                ROS_ERROR_STREAM("lower than change dis");
            }
            if (!fc_changed)
            {
                path_res = path_res_tmp;
                publishNextAim(ros_time, local_aim, local_vel);
                cout << "pub local aim: " << local_aim.transpose() << endl;
                last_pub_time = ros_time;
                transitState(APPROACH);
                break;
            }

        }

        cout<<"classic find: "<<aim_pose.transpose()<<endl;
        // if(m_currentState != RAPID)
        

        if(path_res.size() <= 2) {   // directly aim to the svp

            local_aim = aim_pose;
            aim_vel = calResVel(aim_pose, fc_info_ptr_->drone_pose);
            if (m_currentState == GOHOME || m_currentState == RAPID) aim_vel = Vector3d(0, 0, 0);
            if (fc_info_ptr_->grid_map_->getInflateOccupancy(aim_pose) && m_currentState !=  RAPID) {
                ROS_INFO("\033[1;31m[frontier_finder] Aim Occ, Replan!\033[0m");  //红
                ROS_INFO_STREAM("aim occ: " << aim_pose.transpose());
                transitState(CLASSIC);
                break;
            }
            publishNextAim(ros_time, aim_pose, aim_vel);
            cout << "pub aim: " << aim_pose.transpose() << endl;
            dis2goal = (aim_pose-fc_info_ptr_->drone_pose).norm();
            last_pub_time = ros_time;
        } else {
            path_inx = 1;
            local_aim = getLocalAim();
            local_vel = Vector3d::Zero();
            if (fc_info_ptr_->grid_map_->getInflateOccupancy(local_aim)) {
                ROS_INFO("\033[1;31m[frontier_finder] Aim Occ, Replan!\033[0m");  //红
                ROS_INFO_STREAM("aim occ: " << local_aim.transpose());
                transitState(CLASSIC);
                break;
            }
            publishNextAim(ros_time, local_aim, local_vel);
            cout << "pub local aim: " << local_aim.transpose() << endl;
            dis2goal = (aim_pose-fc_info_ptr_->drone_pose).norm();
            dis2localaim = (local_aim-fc_info_ptr_->drone_pose).norm();
            last_pub_time = ros_time;
        }
        if (m_currentState != GOHOME && m_currentState != RAPID)
            transitState(APPROACH);
        break;
    }

    
    case GOHOME: {
        double dis_2_home = (fc_info_ptr_->drone_pose - home_position_).head(2).norm(); 
        // if (pub_log_->canPub()) 
            // cout << "dis_2_home: "<< dis_2_home << endl;

        // if (pub_log_->canPub()) 
            // ROS_INFO("\033[1;31mGo Home!\033[0m");  //红

        if(path_res.size() > 2){
            double dis2localaim = (local_aim - fc_info_ptr_->drone_pose).norm();
            if(dis2localaim < arrive_local_goal_dis_thres_){
                local_aim = getLocalAim();
                if(path_inx > path_res.size()){
                    ROS_ERROR_STREAM("path size out!");
                    break;
                }
                local_vel = Vector3d::Zero();
                publishNextAim(ros_time, local_aim, local_vel);
                cout << "pub local aim: " << local_aim.transpose() << endl;
                last_pub_time = ros_time;
            }
        }

        last_pub_time = ros_time;
        break;
    }


    case APPROACH: {
        dis2goal = (aim_pose-fc_info_ptr_->drone_pose).norm();
        if (pub_log_->canPub()) ROS_INFO("\033[1;33mApproach...\033[0m");  //黄
        double dis2goal2d = sqrt(pow(aim_pose(0)-fc_info_ptr_->drone_pose(0),2) + pow(aim_pose(1)-fc_info_ptr_->drone_pose(1),2));
        double dis2localaim = (local_aim - fc_info_ptr_->drone_pose).norm();

        if (pub_log_->canPub()) {
            cout<<"Dis to Aim: "<<dis2goal2d<<endl;
            cout<<"Dis to LocalAim: "<<dis2localaim<<endl;
            cout << "path size: " << path_res.size() <<endl;
            cout << "path_inx: " << path_inx <<endl;
        } 

        // 目标fc是否变化
        bool fc_changed = false;
        //todo
        SuperViewPoint best_svp_now = fc_info_ptr_->getSVP(best_svp.id);
        if(best_svp_now.id == -1){
            fc_changed = true;
            // cout<<"changed no id"<<endl;
        } else {
            // cout<<"best_svp old: "<<Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z).transpose()<<endl;
            // cout<<"best_svp new: "<<Vector3d(best_svp_now.super_vp.x, best_svp_now.super_vp.y, best_svp_now.super_vp.z).transpose()<<endl;
            double dis = (Vector3d(best_svp_now.super_vp.x, best_svp_now.super_vp.y, best_svp_now.super_vp.z) - 
                Vector3d(best_svp.super_vp.x, best_svp.super_vp.y, best_svp.super_vp.z)).norm();
            if(dis > 1.0){
                fc_changed = true;
                // cout<<"changed dis"<<endl;
            }
        }

        // Replan Rules
        if (fc_changed){
            ROS_INFO("\033[1;31m[frontier_finder] Ftr Change, Replan!\033[0m");  //红
            transitState(CLASSIC);
            break;
        }
        if ((ros_time.toSec() - last_pub_time.toSec()) > classic_pub_interval){
            ROS_INFO("\033[1;31m[frontier_finder] Time to Replan!\033[0m");  //红
            transitState(CLASSIC);
            break;
        }

        if (dis2goal2d < arrive_goal_dis_thres_){ //* arrive classic goal
            ROS_INFO_STREAM("aim_pose: " << aim_pose.transpose() << ", drone_pose: " << fc_info_ptr_->drone_pose.transpose());
            ROS_INFO_STREAM("[frontier_finder] dis2goal2d: " << dis2goal2d << ", arrive_dis_thres: " << arrive_goal_dis_thres_);
            ROS_INFO("\033[1;31mGet Classic Aim, Replan!\033[0m");  //红
            transitState(CLASSIC);
            break;
        }

        if (fc_info_ptr_->grid_map_->getInflateOccupancy(local_aim))
        {
            ROS_INFO("\033[1;31m[frontier_finder] Aim Occ, Replan!\033[0m");  //红
            ROS_INFO_STREAM("aim occ: " << local_aim.transpose());
            transitState(CLASSIC);
            break;
        }

        if(path_res.size() > 2){
            double dis2localaim = (local_aim - fc_info_ptr_->drone_pose).norm();
            if(dis2localaim < arrive_local_goal_dis_thres_){
                local_aim = getLocalAim();
                if(path_inx == path_res.size()){
                    ROS_ERROR_STREAM("path size out!");
                }
                local_vel = Vector3d::Zero();

                publishNextAim(ros_time, local_aim, local_vel);
                cout << "pub local aim: " << local_aim.transpose() << endl;
                last_pub_time = ros_time;
            }
        }
        break;
    }


    }
    
    double time2 = ros::Time::now().toSec();
    if ((m_currentState != APPROACH && m_currentState != OFF && m_currentState != GOHOME ) || 
        ((m_currentState == APPROACH || m_currentState == OFF || m_currentState == GOHOME) && pub_log_->canPub()) )
        ROS_INFO("\033[1;34m ALL Done - %f ms\033[0m", (time2-time1) * 1000.0);  //蓝
    if ((m_currentState == APPROACH || m_currentState == OFF || m_currentState == GOHOME) && pub_log_->canPub())
        pub_log_->tic();
}

Vector3d FrontierServer::getLocalAim(){
    for(int i = path_res.size()-1; i > path_inx; i--) {
        if(fc_info_ptr_->grid_map_->isInMap(path_res[i]) && 
            fc_info_ptr_->isVisible(fc_info_ptr_->drone_pose, path_res[i], fc_info_ptr_->grid_map_->getResolution()) > 0) {
            path_inx = i;
            return path_res[i];
        }
    }
    path_inx++;
    int idx = path_inx;
    if (path_inx >= path_res.size()) idx = (int)path_res.size() - 1;
    return path_res[idx];
}

SuperViewPoint FrontierServer::classicFronitierFind(Vector3d& res_aimpos, Vector3d& res_aimvel)
{
    Vector3d c_pos = fc_info_ptr_->drone_pose;
    Vector3d c_vel = fc_info_ptr_->drone_vel;
    if(first_cal){
        first_cal = false;
        aim_vel = Vector3d(1.0,0,0);
    }
    Vector3d drone_p_cur(fc_info_ptr_->drone_pose(0),fc_info_ptr_->drone_pose(1),fc_info_ptr_->drone_pose(2));
    Vector3d drone_v_cur(c_vel(0),c_vel(1),c_vel(2));
    Vector3d v_norm = drone_v_cur.normalized();

    double min_score = 99999;
    Vector3d best_aim;
    SuperViewPoint best_sv;
    double score;
    bool is_far_mode = false;
    bool is_room_mode = false;

    auto isInRoom = [](double x, double y)->bool
    {
        return (y < -9.5 && x < 0.0);
    };

    Vector3d drone_aim_vel_last = Vector3d(aim_vel(0), aim_vel(1), aim_vel(2));
    if(fc_info_ptr_->svp_list.size() <= 0){
        ROS_ERROR_STREAM("svp_list empty!");
    }
    else if(fc_info_ptr_->svp_list.size() <= 7 &&
            !(isInRoom(c_pos.x(), c_pos.y())))
    {
        for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
        {
            Vector3d pos;
            pos << it->super_vp.x, it->super_vp.y, it->super_vp.z;
            if ((c_pos - pos).norm() < 15.0)
            {
                is_far_mode = false;
                break;
            }
        }
        ROS_ERROR_STREAM("far_mode!");
        is_far_mode = true;
    }
    else if (isInRoom(c_pos.x(), c_pos.y()))
    {
        for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
        {
            Vector3d pos;
            pos << it->super_vp.x, it->super_vp.y, it->super_vp.z;
            if(fc_info_ptr_->free_space_fac_->grid_map_->getInflateOccupancy(pos)) continue;
            if (isInRoom(it->super_vp.x, it->super_vp.y))
            {
                is_room_mode = true;
                ROS_ERROR_STREAM("room_mode!");
                break;
            }
        }
    }

    for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
    {
        Vector3d pos;
        pos << it->super_vp.x, it->super_vp.y, it->super_vp.z;
        // if(pos(0)==0.0 && pos(1)==0.0 && pos(2)==0.0) continue;
        if(fc_info_ptr_->free_space_fac_->grid_map_->getInflateOccupancy(pos)) continue;
        if (is_room_mode && !isInRoom(it->super_vp.x, it->super_vp.y)) continue;

        Vector3d drone2frontier = pos - fc_info_ptr_->drone_pose;
        // cout<<"SVP-keyposeInx: "<<it->keypose_inx<<" ,in all of "<<MPG->posegraphes[0].getSize()<<endl;
        // cout<<"[" << it->id << "] center: "<<pos.transpose()<<", drone2frontier: "<<drone2frontier.transpose()<<endl;
        // double cur_dis = drone2frontier.norm();
        double cur_dis = calConcretDis(*it);

        double d_theta = acos(c_vel.normalized().dot(drone2frontier.normalized())) * 180.0 / M_PI; //0~180
        double d_theta_from_last = acos(drone_aim_vel_last.normalized().dot(drone2frontier.normalized())) * 180.0 / M_PI; //0~180
        // cout<<"d_theta_from_last: "<<d_theta_from_last<<endl;

        // if(cur_dis < 3.0) continue; //TODO 

        if(cur_dis < fc_info_ptr_->free_space_fac_->sensor_range){
            // score = cur_dis;
            // score = cur_dis + (1.0/180.0) * d_theta / (cur_dis*cur_dis) + (1.0/1) * d_theta_from_last / (cur_dis*cur_dis);
            // score = cur_dis + (1.0/30.0) * d_theta_from_last + (1.0/30.0) * d_theta; 
            // score = cur_dis + (1.0/1) * d_theta_from_last / (cur_dis*cur_dis);
            score = cur_dis + (1.0/1) * d_theta_from_last / (80.0);

        }else{
            // score = cur_dis + (1.0/1) * d_theta_from_last / (cur_dis*cur_dis);
            score = cur_dis + (1.0/1) * d_theta_from_last / (80.0);
            // score = cur_dis;// + (1.0/30.0) * d_theta_from_last;
        }

        if (is_far_mode)
        {
            double dis_2_home = (pos - home_position_).norm();
            score = 100.0 - dis_2_home;  // BB
        }

        // // near boundary
        // Eigen::Vector3d min_bound = fc_info_ptr_->grid_map_->getMinBound();
        // Eigen::Vector3d max_bound = fc_info_ptr_->grid_map_->getMaxBound();
        // if (abs(pos.x() - min_bound.x()) < 10.0 ||
        //     abs(pos.y() - min_bound.y()) < 10.0 ||
        //     abs(pos.x() - max_bound.x()) < 10.0 ||
        //     abs(pos.y() - max_bound.y()) < 10.0 )
        // {
        //     score -= 10.0;
        // }


        ROS_ASSERT(score > 0 && score < 1000);
        // cout<<"score: "<<score<<endl;
        // cout<<"dis | angle(ang_score) : "<< cur_dis << ", " << d_theta_from_last << "(" << d_theta_from_last / (36.0) << ")" <<endl;
        // cout<<endl;
        if(score < min_score){
            best_sv = *it; 
            min_score = score;
        }
    }

    // TODO
    if(fc_info_ptr_->car_id == 100) {
        min_score = 99999;
        for( list<SuperViewPoint>::iterator it =  fc_info_ptr_->svp_list.begin(); it != fc_info_ptr_->svp_list.end(); it++)
        {
            Vector3d pos;
            pos << it->super_vp.x, it->super_vp.y, it->super_vp.z;
            // if(pos(0)==0.0 && pos(1)==0.0 && pos(2)==0.0) continue;
            if(fc_info_ptr_->free_space_fac_->grid_map_->getInflateOccupancy(pos)) continue;
            if (is_room_mode && !isInRoom(it->super_vp.x, it->super_vp.y)) continue;

            Vector3d drone2frontier = pos - fc_info_ptr_->drone_pose;

            double cur_dis = calConcretDis(*it);

            score = cur_dis;


            ROS_ASSERT(score > 0 && score < 1000);
            // cout<<"score: "<<score<<endl;
            // cout<<"dis | angle(ang_score) : "<< cur_dis << ", " << d_theta_from_last << "(" << d_theta_from_last / (36.0) << ")" <<endl;
            // cout<<endl;
            if(score < min_score){
                best_sv = *it; 
                min_score = score;
            }
        }
    }


    if(min_score < 99999)
    {
        path_res.clear();
        getPathtoGoal(best_sv, drone_p_cur, path_res);
        fc_info_ptr_->vis_ptr->visualize_path(path_res, "graph_path");
        res_aimpos = best_sv.getPoseForPlan();

        res_aimvel = calResVel(res_aimpos, drone_p_cur);

        cout<<"vmax: "<<v_max<<endl;
        cout<<"res_aimpos: "<<res_aimpos.transpose()<<endl;
        cout<<"res_aimvel: "<<res_aimvel.transpose()<<endl;
        return best_sv;
    }
    best_sv.id = -1;
    return best_sv;
}
// The result path contains the start & end points
double FrontierServer::getPathtoGoal(SuperViewPoint& svp, Vector3d& drone_p, vector<Vector3d>& path){
    // cout<<"In getPathtoGoal"<<endl;
    path.clear();
    Vector3d aim_svp_p = svp.getPoseForPlan();

    int aim_inx = svp.keypose_inx;
    int cur_inx = fc_info_ptr_->free_space_info.free_space_list.size() - 1 - fc_info_ptr_->temp_num;
    // If the svp is directly visible and within the sensor_range
    if(fc_info_ptr_->isVisible(drone_p, aim_svp_p, fc_info_ptr_->grid_map_->getResolution()) > 0 &&
         (drone_p-aim_svp_p).norm()<fc_info_ptr_->free_space_fac_->sensor_range)
    {
        path.push_back(aim_svp_p);
        path.push_back(drone_p);
        reverse(path.begin(), path.end());
        cout<<"Visible: "<<fc_info_ptr_->isVisible(drone_p, aim_svp_p, fc_info_ptr_->grid_map_->getResolution())<<", "<<drone_p.transpose()<<"->"<< aim_svp_p.transpose()<<endl;
        return (drone_p - aim_svp_p).norm();
    }
    // Else calculate from posegraph
    double dis =  MPG->calGraphPath(make_pair(0,cur_inx), make_pair(0,aim_inx), path);
    path.insert(path.begin(), drone_p);
    path.push_back(aim_svp_p);
    dis += (path[0]-path[1]).norm();
    dis += (path[path.size()-1]-path[path.size()-2]).norm();
    return dis;
}

double FrontierServer::getPathHome(Vector3d& home_p, Vector3d& drone_p, vector<Vector3d>& path)
{
    path.clear();

    int aim_inx = 0;
    int cur_inx = fc_info_ptr_->free_space_info.free_space_list.size() - 1 - fc_info_ptr_->temp_num;
    // If the svp is directly visible and within the sensor_range
    if(fc_info_ptr_->isVisible(drone_p, home_p, fc_info_ptr_->grid_map_->getResolution()) > 0 &&
         (drone_p-home_p).norm()<fc_info_ptr_->free_space_fac_->sensor_range) {
        path.push_back(home_p);
        path.push_back(drone_p);
        reverse(path.begin(), path.end());
        cout<<"Visible: "<<fc_info_ptr_->isVisible(drone_p, home_p, fc_info_ptr_->grid_map_->getResolution())<<", "<<drone_p.transpose()<<"->"<< home_p.transpose()<<endl;
        return (drone_p - home_p).norm();
    }
    // Else calculate from posegraph
    double dis =  MPG->calGraphPath(make_pair(0,cur_inx), make_pair(0,aim_inx), path);
    path.insert(path.begin(), drone_p);
    path.push_back(home_p);
    dis += (path[0]-path[1]).norm();
    dis += (path[path.size()-1]-path[path.size()-2]).norm();
    return dis;
}


// calculate the ditance toward an svp using posegraph
double FrontierServer::calConcretDis(SuperViewPoint& svp){
    Vector3d svp_p(svp.super_vp.x, svp.super_vp.y, svp.super_vp.z);
    // If directly visible, return stright line
    if(fc_info_ptr_->grid_map_->isInMap(svp_p) && fc_info_ptr_->isVisible(svp_p, fc_info_ptr_->drone_pose, 0.0) > 0)
    {
        return ((svp_p - fc_info_ptr_->drone_pose).norm());
    }
    else
    {
        vector<Vector3d> path;
        ROS_ERROR("svp_p!!!!: %f, %f, %f", svp_p(0), svp_p(1), svp_p(2));
        return getPathtoGoal(svp, fc_info_ptr_->drone_pose, path);
        ROS_ERROR("svp_p2!!!!: %f, %f, %f", svp_p(0), svp_p(1), svp_p(2));

    }
}


Vector3d FrontierServer::calResVel(Vector3d res_aimpos, Vector3d drone_p){
    Vector3d res_aimvel= Vector3d::Zero();
    if((res_aimpos-drone_p).norm() > fc_info_ptr_->free_space_fac_->sensor_range){
        Vector3d diff = res_aimpos-drone_p;
        double max_axis_dis = max(max(abs(diff(0)),abs(diff(1))),abs(diff(2)));
        res_aimvel = diff*(v_max/max_axis_dis);
    }else{
        res_aimvel = (res_aimpos-drone_p)*(v_max/fc_info_ptr_->free_space_fac_->sensor_range);
    }
    return res_aimvel;
}

void FrontierServer::publishNextAim(const ros::Time& rostime, const Vector3d aim_pose, const Vector3d aim_vel){
    // nav_msgs::Odometry odom;
    // odom.header.stamp = rostime;
    // odom.header.frame_id = "world";

    // odom.pose.pose.position.x = aim_pose(0);
    // odom.pose.pose.position.y = aim_pose(1);
    // odom.pose.pose.position.z = aim_pose(2);
    // // odom.pose.pose.position.z = 1e-3;


    // odom.twist.twist.linear.x = aim_vel(0);
    // odom.twist.twist.linear.y = aim_vel(1);
    // odom.twist.twist.linear.z = aim_vel(2);
    // // odom.twist.twist.linear.z = 0.0;

    // if(is_on_car){
    //     odom.pose.pose.position.z = fc_info_ptr_->car_odom_z;
    //     odom.twist.twist.linear.z = 0.0;
    // }

    geometry_msgs::PointStamped odom;
    odom.header.stamp = rostime;
    odom.header.frame_id = "map";

    odom.point.x = aim_pose(0);
    odom.point.y = aim_pose(1);
    odom.point.z = aim_pose(2);


    if(is_on_car){
        odom.point.z = fc_info_ptr_->car_odom_z;
    }


    if(fc_info_ptr_->free_space_fac_->grid_map_->getInflateOccupancy(Vector3d(aim_pose(0),aim_pose(1),aim_pose(2)))){
        ROS_ERROR_STREAM("Frontier is OCC! "<<aim_pose.transpose());
        return;
    }


    m_nextAimPub.publish(odom);
}




