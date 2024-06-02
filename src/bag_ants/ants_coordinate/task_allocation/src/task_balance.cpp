

#include <task_allocation/task_balance.h>
#include <task_allocation/HGrid.h>
// #include <task_allocation/GridTour.h>
#include <msg_utils/GridTour.h>

#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <plan_env/multi_map_manager.h>
#include <active_perception/perception_utils.h>
#include <active_perception/hgrid.h>
#include <nav_msgs/Path.h>
#include <fstream>

using Eigen::Vector4d;
 
namespace fast_planner {

shared_ptr<Astar> FastExplorationFSM::astar_;
shared_ptr<RayCaster> FastExplorationFSM::caster_;
double FastExplorationFSM::vm_;
double FastExplorationFSM::am_;
double FastExplorationFSM::yd_;
double FastExplorationFSM::ydd_;
double FastExplorationFSM::w_dir_;

void FastExplorationFSM::init(ros::NodeHandle& nh) {
    // 环境初始化
    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nh);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setMap(sdf_map_);
    // H网格初始化
    hgrid_.reset(new HGrid(edt_environment_, nh));
    // 可视化初始化
    visualization_.reset(new PlanningVisualization(nh));
    // 射线初始化
    caster_.reset(new RayCaster);
    double resolution_ = sdf_map_->getResolution();
    Eigen::Vector3d origin, size;
    sdf_map_->getRegion(origin, size);
    caster_->setParams(resolution_, origin);
    // A*初始化
    astar_.reset(new Astar);
    astar_->init(nh, edt_environment_);
    // 数据初始化
    ed_.reset(new ExplorationData);
    ep_.reset(new ExplorationParam);
    
    nh.param("exploration/refine_local", ep_->refine_local_, true);
    nh.param("exploration/refined_num", ep_->refined_num_, -1);
    nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
    nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
    nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
    nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
    nh.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
    nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
    nh.param("exploration/drone_num", ep_->drone_num_, 1);
    nh.param("exploration/drone_id", ep_->drone_id_, 1);
    nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);

    nh.param("exploration/vm", FastExplorationFSM::vm_, -1.0);
    nh.param("exploration/am", FastExplorationFSM::am_, -1.0);
    nh.param("exploration/yd", FastExplorationFSM::yd_, -1.0);
    nh.param("exploration/ydd", FastExplorationFSM::ydd_, -1.0);
    nh.param("exploration/w_dir", FastExplorationFSM::w_dir_, -1.0);

    nh.param("frontier/min_candidate_dist", min_candidate_dist_, -1.0);

    ed_->swarm_state_.resize(ep_->drone_num_);
    ed_->pair_opt_stamps_.resize(ep_->drone_num_);
    ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
    for (int i = 0; i < ep_->drone_num_; ++i) {
        ed_->swarm_state_[i].stamp_ = 0.0;
        ed_->pair_opt_stamps_[i] = 0.0;
        ed_->pair_opt_res_stamps_[i] = 0.0;
    }

    for (auto& state : ed_->swarm_state_) {
        state.stamp_ = 0.0;
        state.recent_interact_time_ = 0.0;
        state.recent_attempt_time_ = 0.0;
        state.pos_ = Vector3d(0, 0, 10000);
    }
    ed_->last_grid_ids_ = {};
    ed_->reallocated_ = true;
    ed_->pair_opt_stamp_ = 0.0;
    ed_->wait_response_ = false;
    ed_->plan_num_ = 0;

    last_ego_ids.resize(ep_->drone_num_, {});

    prev_app1 = 999999999999;

    // 初始化前沿点
    list<Frontier> frontiers_svp_new;
    frontiers_svp.resize(ep_->drone_num_, frontiers_svp_new);
    // 主计时器
    exec_timer_ = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::FSMCallback, this);
    odom_sub_ = nh.subscribe("/state_estimation", 1, &FastExplorationFSM::odomCallback, this);
    // vis_timer_ = nh.createTimer(ros::Duration(0.1), &FastExplorationFSM::VISCallback, this);
    // 别车位置接受
    for(int i = 0; i < ep_->drone_num_; i++) {
        std::string topic_name = "/ant0" + std::to_string(i + 1) + "/state_estimation";
        ros::Subscriber mcar_odom_sub_ = nh.subscribe(topic_name, 1, &FastExplorationFSM::mcarOdomCallback, this);
        subscribers_.push_back(mcar_odom_sub_);
    }
    // 别车前沿点接受
    mcar_FreeSpaceAndFrontierInfo_sub = nh.subscribe("/mcar_FreeSpaceAndFrontierInfo", 1, &FastExplorationFSM::mcar_FreeSpaceAndFrontierInfoCallback, this);
    mcar_task_allocation_pub = nh.advertise<msg_utils::FreeSpaceAndFrontierInfo>("/mcar_task_allocation", 1);

    tsp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp_" + to_string(ep_->drone_id_), true);
    acvrp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_acvrp_" + to_string(ep_->drone_id_), true);

    grid_tour2_pub = nh.advertise<msg_utils::GridTour>("/grid_tour2", 1);
    vis_pub_ = nh.advertise<nav_msgs::Path>("/tour2_vis", 1);

}


void FastExplorationFSM::mcar_FreeSpaceAndFrontierInfoCallback(const msg_utils::FreeSpaceAndFrontierInfoConstPtr msg) {
    // return;
    // 主要是接受当前的前沿点信息，所有车的，汇总之后用于更新
    msg_utils::FreeSpaceAndFrontierInfo msg_temp = *msg;
    msg_utils::FrontierInfo cur_frontier_info;

    // for(int i = 0; i < msg_temp.frontier_info.super_viewpoints.size(); i++) {
    //     msg_utils::SuperViewPoint cur_svp;

    //     cur_svp.robot_id = msg_temp.frontier_info.super_viewpoints[i].robot_id;
    //     cur_svp.super_viewpoin_id = msg_temp.frontier_info.super_viewpoints[i].super_viewpoin_id;
    //     cur_svp.super_viewpoint.x = msg_temp.frontier_info.super_viewpoints[i].super_viewpoint.x;
    //     cur_svp.super_viewpoint.y = msg_temp.frontier_info.super_viewpoints[i].super_viewpoint.y;
    //     cur_svp.super_viewpoint.z = msg_temp.frontier_info.super_viewpoints[i].super_viewpoint.z;

    //     for(int j = 0; j < msg_temp.frontier_info.super_viewpoints[i].viewpoints.size(); j++) {
    //         cur_svp.viewpoints.push_back(msg_temp.frontier_info.super_viewpoints[i].viewpoints[j]);
    //     }

    //     cur_frontier_info.super_viewpoints.push_back(cur_svp);
    // }

    for(int i = 0; i < msg_temp.frontier_info.super_viewpoints.size(); i++) {
        for(int j = 0; j < msg_temp.frontier_info.super_viewpoints[i].viewpoints.size(); j++) {
            msg_utils::SuperViewPoint cur_svp;

            cur_svp.robot_id = msg_temp.frontier_info.super_viewpoints[i].robot_id;
            cur_svp.super_viewpoin_id = msg_temp.frontier_info.super_viewpoints[i].super_viewpoin_id;
            cur_svp.super_viewpoint.x = msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].frontier_cluster.x;
            cur_svp.super_viewpoint.y = msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].frontier_cluster.y;
            cur_svp.super_viewpoint.z = msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].frontier_cluster.z;

            for(int k = 0; k < msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list.size(); k++) {
                msg_utils::ViewPoint cur_vp_tmp;
                double tri_x = (msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k].points[0].x + 
                                   msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k].points[1].x +
                                   msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k].points[2].x)/3;
                double tri_y = (msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k].points[0].y + 
                                   msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k].points[1].y +
                                   msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k].points[2].y)/3;
                double tri_z = (msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k].points[0].z + 
                                   msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k].points[1].z +
                                   msg_temp.frontier_info.super_viewpoints[i].viewpoints[j].cell_list[k].points[2].z)/3;
                cur_vp_tmp.viewpoint.x = tri_x;
                cur_vp_tmp.viewpoint.y = tri_y;
                cur_vp_tmp.viewpoint.z = tri_z;
                cur_svp.viewpoints.push_back(cur_vp_tmp);
            }

            cur_frontier_info.super_viewpoints.push_back(cur_svp);
        }
    }

    // // 打印视点列表信息，包含车辆id和视点id以及视点坐标
    // // 打印队列长度
    // std::cout << "super_viewpoints size: " << cur_frontier_info.super_viewpoints.size() << std::endl;
    // for(int i = 0; i < cur_frontier_info.super_viewpoints.size(); i++) {
    //     std::cout << "robot_id: " << cur_frontier_info.super_viewpoints[i].robot_id 
    //               << ", super_viewpoin_id: " << cur_frontier_info.super_viewpoints[i].super_viewpoin_id 
    //               << ", super_viewpoint: " 
    //               << cur_frontier_info.super_viewpoints[i].super_viewpoint.x << ", " 
    //               << cur_frontier_info.super_viewpoints[i].super_viewpoint.y << ", " 
    //               << cur_frontier_info.super_viewpoints[i].super_viewpoint.z << std::endl;
    // }

    // 更新前沿信息
    updateFrontierStates(cur_frontier_info);
}

void FastExplorationFSM::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    Vector3d rot_x = odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
    odom_yaw_ = atan2(rot_x(1), rot_x(0));
}

void FastExplorationFSM::mcarOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // if(msg->pose.covariance[0] == car) {
    //     return;
    // }
    // ROS_WARN("mcarOdomCallback");
    int robot_id = msg->pose.covariance[0];  // 仿真里边，里程计随便填到了covariance里边
    Vector3d pos;
    pos(0) = msg->pose.pose.position.x;
    pos(1) = msg->pose.pose.position.y;
    pos(2) = msg->pose.pose.position.z;

    Vector3d vel;
    vel(0) = msg->twist.twist.linear.x;
    vel(1) = msg->twist.twist.linear.y;
    vel(2) = msg->twist.twist.linear.z;

    // double yaw = atan2(rot_x(1), rot_x(0));

    // 更新车辆状态
    ed_->swarm_state_[robot_id - 1].pos_ = pos;
    ed_->swarm_state_[robot_id - 1].vel_ = vel;
    // ed_->swarm_state_[robot_id - 1].yaw_ = yaw;
    ed_->swarm_state_[robot_id - 1].stamp_ = ros::Time::now().toSec();
}

// void FastExplorationFSM::VISCallback(const ros::TimerEvent& e) {
//     if(last_ego_ids.empty()) {
//         return;
//     }
//     visualize(last_ego_ids);
// }


/**
 * hgrid 使用
 * 1、hgrid_->getNextGrid(grid_ids, grid_pos, grid_yaw)  
 *      获取到序列上网格位置、yaw
 * 2、hgrid_->getFrontiersInGrid(ego_ids, ftr_ids, ftr2_ids)
 * 
 * 3、hgrid_->getCenter(grid_ids.front())
 * 
 * 4、hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat)
 * 
 * 5、hgrid_->getUnknownCellsNum(grid_ids[i])
 * 
 * 6、hgrid_->getCostDroneToGrid(pos, grid_ids[0], first)
 * 
 * 7、hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size())
 * 
 * 8、hgrid_->inputFrontiers(ed_->averages_)
 * 
 * 9、hgrid_->updateGridData(ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids)
 *
 * 10、hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_)
 * 
 * 
*/

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
    // 有必要的时候再执行下面的代码
    // 1 时间超过stop_time之前，不执行下面的代码
    if (ros::Time::now() < stop_time) {
        return;
    }

    // // 2 两车距离太远时，不执行下面的代码
    // for(int i = 0; i < ep_->drone_num_; i++) {
    //     // std::cout << "positions[" << i << "]: " << positions[i].transpose() << std::endl;
    //     if(ed_->swarm_state_[i].pos_[2] == 10000) {
    //         ROS_WARN("car[%d] position is not updated", i);
    //         return;
    //     }
    // }
    // if((ed_->swarm_state_[0].pos_ - ed_->swarm_state_[1].pos_).norm() > 10) {
    //     ROS_WARN("car distance is too far");
    //     return;
    // }


    Vector3d pos = odom_pos_;
    Vector3d vel = odom_vel_;
    Vector3d yaw = Vector3d(odom_yaw_, 0, 0);
    // std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose() << std::endl;
    vector<int> grid_ids, frontier_ids, frontier2_ids;



    // 生成单车初始路径
    findGridAndFrontierPath(pos, vel, yaw, grid_ids, frontier_ids, frontier2_ids);

    // // 执行与别车的任务分配，直接求解一个五车VRP问题
    // //1 执行分配
    // vector<Eigen::Vector3d> positions;
    // // positions.resize(ep_->drone_num_);
    // for(int i = 0; i < ep_->drone_num_; i++) {
    //     positions.push_back(ed_->swarm_state_[i].pos_);
    // }

    // vector<Eigen::Vector3d> velocities;
    // vector<vector<int>> first_ids1, second_ids1;
    // // velocities.resize(ep_->drone_num_);
    // // first_ids1.resize(ep_->drone_num_);
    // // second_ids1.resize(ep_->drone_num_);
    // for(int i = 0; i < ep_->drone_num_; i++) {
    //     first_ids1.push_back({});
    //     second_ids1.push_back({});
    //     velocities.push_back(Eigen::Vector3d(0, 0, 0));
    // }
    // // 希望保持稳定的网格
    // // hgrid_->getConsistentGrid(
    // //     state1.grid_ids_, state1.grid_ids_, first_ids1, second_ids1, 0);
    // // hgrid_->getConsistentGrid(
    // //     state2.grid_ids_, state2.grid_ids_, first_ids2, second_ids2, 1);
    // for(int i = 0; i < ep_->drone_num_; i++) {
    //     hgrid_->getConsistentGrid(
    //         last_ego_ids[i], last_ego_ids[i], first_ids1[i], second_ids1[i], i);
    // }
    // // 成对优化
    // // vector<int> ego_ids, other_ids;
    // // allocateGrids(positions, velocities, { first_ids1, first_ids2 },
    // //               { second_ids1, second_ids2 }, opt_ids, ego_ids, other_ids);
    // // 输入待分配的网格
    // vector<int> opt_ids;
    // opt_ids = grid_ids;
    // // 结果存储网格
    // vector<vector<int>> ego_ids;
    // auto allocateGrids_t = ros::Time::now();
    // allocateGrids(positions, velocities, first_ids1, second_ids1, opt_ids, ego_ids);
    // double alloc_time = (ros::Time::now() - allocateGrids_t).toSec();
    // visualize(ego_ids);

    // // 首先打印上次的分配结果
    // // for(int i = 0; i < last_ego_ids.size(); i++) {
    // //     std::cout << "last_ego_ids[" << i << "]: ";
    // //     for(int j = 0; j < last_ego_ids[i].size(); j++) {
    // //         std::cout << last_ego_ids[i][j] << ", ";
    // //     }
    // //     std::cout << "" << std::endl;
    // // }
    // // 然后打印本次的分配结果
    // // for(int i = 0; i < ego_ids.size(); i++) {
    // //     std::cout << "ego_ids[" << i << "]: ";
    // //     for(int j = 0; j < ego_ids[i].size(); j++) {
    // //         std::cout << ego_ids[i][j] << ", ";
    // //     }
    // //     std::cout << "" << std::endl;
    // // }
    // // std::cout << "" << std::endl;
    // //2 评估分配结果并发布
    // // checkAllocateResult();
    // // Check results
    // // 分配前的代价
    // // vector<double> prev_app1;
    // // for(int i = 0; i < ep_->drone_num_; i++) {
    // //     double prev_app = computeGridPathCost(positions[i], last_ego_ids[i], first_ids1[i],
    // //         first_ids1, second_ids1, true);
    // //     std::cout << "prev cost: " << prev_app << std::endl;
    // //     prev_app1.push_back(prev_app);
    // // }

    // // 分配后的代价
    // vector<double> cur_app1;
    // for(int i = 0; i < ep_->drone_num_; i++) {
    //     double cur_app = computeGridPathCost(positions[i], ego_ids[i], first_ids1[i],
    //         first_ids1, second_ids1, true);
    //     // std::cout << "cur cost: " << cur_app << std::endl;
    //     cur_app1.push_back(cur_app);
    // }

    // // 根据总代价判断是否更新，总代价反而更大了，不更新
    // double prev_app_all = prev_app1;
    // double cur_app_all = 0;
    // for(int i = 0; i < ep_->drone_num_; i++) {
    //     // prev_app_all += prev_app1[i];
    //     cur_app_all += cur_app1[i];
    // }
    // if (cur_app_all > prev_app_all + 0.1) {
    //         for(int i = 0; i < ego_ids.size(); i++) {
    //             std::cout << "ego_ids[" << i << "]: ";
    //             for(int j = 0; j < ego_ids[i].size(); j++) {
    //                 std::cout << ego_ids[i][j] << ", ";
    //             }
    //             std::cout << "" << std::endl;
    //         }
    //     ROS_ERROR("Larger cost after reallocation");
    //     // 记录此时时间，停止计算10S
    //     stop_time = ros::Time::now() + ros::Duration(4);
    //     return;
    // }
    // prev_app1 = cur_app_all;
    // // 提示分配结果过于不稳定
    // for(int i = 0; i < ep_->drone_num_; i++) {
    //     if (!last_ego_ids[i].empty() && !ego_ids[i].empty() &&
    //         !hgrid_->isConsistent(last_ego_ids[i][0], ego_ids[i][0])) {
    //         ROS_ERROR("Path 1 inconsistent");
    //     }
    // }

    // // 根据路线分配视点

    // // 更新分配结果并发布
    // msg_utils::FreeSpaceAndFrontierInfo mcar_task_allocation_msg;
    // mcar_task_allocation_pub.publish(mcar_task_allocation_msg);

    // 可视化
    // visualize(ego_ids);
    // last_ego_ids = ego_ids;

    
}

/***********************功能模块1*********************************************************/
void FastExplorationFSM::findGridAndFrontierPath(const Vector3d& cur_pos,
    const Vector3d& cur_vel, const Vector3d& cur_yaw, vector<int>& grid_ids,
    vector<int>& frontier_ids, vector<int>& frontier2_ids) {
  auto t1 = ros::Time::now();

  // Select nearby drones according to their states' stamp
  vector<Eigen::Vector3d> positions = { cur_pos };
  // vector<Eigen::Vector3d> velocities = { Eigen::Vector3d(0, 0, 0) };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // Partitioning-based tour planning
  vector<int> ego_ids;
  vector<vector<int>> other_ids;
  if (!findGlobalTourOfGrid(positions, velocities, ego_ids, other_ids)) {
    grid_ids = {};
    // ROS_WARN("No grid found after findGlobalTourOfGrid");

    msg_utils::GridTour grid_tour2_msg;
    for(int i = 0; i < 1; i++) {
        geometry_msgs::Point grid_tour2_point;
        grid_tour2_point.x = 0;
        grid_tour2_point.y = 0;
        grid_tour2_point.z = 0;
        grid_tour2_msg.points.push_back(grid_tour2_point);
        geometry_msgs::Point grid_tour2_point2;
        grid_tour2_point.x = 0;
        grid_tour2_point.y = 0;
        grid_tour2_point.z = 0;
        grid_tour2_msg.points2.push_back(grid_tour2_point2);
    }
    grid_tour2_msg.drone_id = ep_->drone_id_;
    grid_tour2_msg.stamp = ros::Time::now().toSec();
    grid_tour2_pub.publish(grid_tour2_msg);

    return;
  }

  if(ego_ids.empty()) {
    ROS_WARN("ego is empty");
    msg_utils::GridTour grid_tour2_msg;
    for(int i = 0; i < 1; i++) {
        geometry_msgs::Point grid_tour2_point;
        grid_tour2_point.x = 0;
        grid_tour2_point.y = 0;
        grid_tour2_point.z = 0;
        grid_tour2_msg.points.push_back(grid_tour2_point);
        geometry_msgs::Point grid_tour2_point2;
        grid_tour2_point.x = 0;
        grid_tour2_point.y = 0;
        grid_tour2_point.z = 0;
        grid_tour2_msg.points2.push_back(grid_tour2_point2);
    }
    grid_tour2_msg.drone_id = ep_->drone_id_;
    grid_tour2_msg.stamp = ros::Time::now().toSec();
    grid_tour2_pub.publish(grid_tour2_msg);
  }
//   if(ego_ids.empty())
//   ROS_WARN("ego is empty");
  grid_ids = ego_ids;

  double grid_time = (ros::Time::now() - t1).toSec();

  // Frontier-based single drone tour planning
  // Restrict frontier within the first visited grid
  t1 = ros::Time::now();

  vector<int> ftr_ids;
  vector<int> ftr2_ids;
  // uniform_grid_->getFrontiersInGrid(ego_ids[0], ftr_ids);
  hgrid_->getFrontiersInGrid(ego_ids, ftr_ids, ftr2_ids);
//   ROS_INFO("Find frontier tour, %d,%d,%d involved------------", ftr_ids.size(), ftr2_ids.size(), ego_ids.size());
  // 打印前沿点
    for(int i = 0; i < ftr_ids.size(); i++) {
        std::cout << "ftr_ids[" << i << "]: " << ftr_ids[i] << std::endl;
    }
    // for(int i = 0; i < ftr2_ids.size(); i++) {
    //     std::cout << "ftr2_ids[" << i << "]: " << ftr2_ids[i] << std::endl;
    // }


  if (ftr_ids.empty()) {
    frontier_ids = {};
    return;
  }
// return;
  // Consider next grid in frontier tour planning
//   Eigen::Vector3d grid_pos;
//   double grid_yaw;
//   vector<Eigen::Vector3d> grid_pos_vec;
//   if (hgrid_->getNextGrid(ego_ids, grid_pos, grid_yaw)) {
//     grid_pos_vec = { grid_pos };
//   }
  frontier2_ids = ftr2_ids;
  frontier_ids = ftr_ids;
    vector<vector<int>>  frontier_idss = {frontier_ids};
  visualize(frontier_idss);
//   findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr_ids, grid_pos_vec, frontier_ids);
  // findTourOfFrontier(cur_pos, cur_vel, cur_yaw, ftr2_ids, grid_pos_vec, frontier2_ids);
  double ftr_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Grid tour t: %lf, frontier tour t: %lf.", grid_time, ftr_time);
    // for(int i = 0; i < frontier_ids.size(); i++) {
    //     std::cout << "frontier_ids[" << i << "]: " << frontier_ids[i] << std::endl;
    // }
}

bool FastExplorationFSM::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
    bool init) {

    ROS_INFO("Find grid tour---------------");

    auto t1 = ros::Time::now();

    auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;
    if (grid_ids.empty() && ed_->swarm_state_.empty()) {
        // ROS_WARN("Empty dominance1.");
        ed_->grid_tour_.clear();
        // return false;
    }
    // ROS_WARN("Empty dominance33 %i",grid_ids.size());

    // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

    vector<int> first_ids, second_ids;

    // TODO 此处需要一个函数，把前沿的数值填入ed_->averages_
    updateFrontierStates();
    // ROS_WARN("updateFrontierStates() is over");

    hgrid_->inputFrontiers(ed_->averages_);

    hgrid_->updateGridData(
        ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids);
    // ROS_WARN("Empty dominance22 %i",grid_ids.size());

    if (grid_ids.empty()) {
        ROS_WARN("Empty dominance.");
        ed_->grid_tour_.clear();
        return false;
    }

    // std::cout << "Allocated grid: ";
    // for (auto id : grid_ids) std::cout << id << ", ";
    // std::cout << "" << std::endl;

    Eigen::MatrixXd mat;
    // uniform_grid_->getCostMatrix(positions, velocities, first_ids, grid_ids, mat);
    if (!init)
        hgrid_->getCostMatrix(positions, velocities, { first_ids }, { second_ids }, grid_ids, mat);
    else
        hgrid_->getCostMatrix(positions, velocities, { {} }, { {} }, grid_ids, mat);

    double mat_time = (ros::Time::now() - t1).toSec();

    // Find optimal path through ATSP
    t1 = ros::Time::now();
    const int dimension = mat.rows();
    const int drone_num = 1;

    // Create problem file
    ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : amtsp\n";
    file << "TYPE : ATSP\n";
    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
        }
        file << "\n";
    }
    file.close();

    // Create par file
    file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_svp.size()) / drone_num, 4)) <<
    // "\n"; file << "MTSP_MAX_SIZE = "
    //      << to_string(max(1, int(ed_->frontiers_svp.size()) / max(1, drone_num - 1))) << "\n";
    file << "RUNS = 1\n";
    file << "TRACE_LEVEL = 0\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp";
    t1 = ros::Time::now();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 2;
    if (!tsp_client_.call(srv)) {
        ROS_ERROR("Fail to solve ATSP.");
        return false;
    }

    double mtsp_time = (ros::Time::now() - t1).toSec();
    // std::cout << "AmTSP time: " << mtsp_time << std::endl;

    // Read results
    t1 = ros::Time::now();

    ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
        if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
        int id = stoi(res);
        ids.push_back(id - 1);
        if (id == -1) break;
    }
    fin.close();

    // Parse the m-tour of grid
    vector<vector<int>> tours;
    vector<int> tour;
    for (auto id : ids) {
        if (id > 0 && id <= drone_num) {
        tour.clear();
        tour.push_back(id);
        } else if (id >= dimension || id <= 0) {
        tours.push_back(tour);
        } else {
        tour.push_back(id);
        }
    }


    others.resize(drone_num - 1);
    for (int i = 1; i < tours.size(); ++i) {
        if (tours[i][0] == 1) {
        indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
        } else {
        others[tours[i][0] - 2].insert(
            others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
        }
    }
    for (auto& id : indices) {
        id -= 1 + drone_num;
    }
    for (auto& other : others) {
        for (auto& id : other) id -= 1 + drone_num;
    }
    // std::cout << "Grid tour: ";
    for (auto& id : indices) {
        id = grid_ids[id];
        // std::cout << id << ", ";
    }
    // std::cout << "" << std::endl;

    // uniform_grid_->getGridTour(indices, ed_->grid_tour_);
    grid_ids = indices;
    hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_, 1);

    // 检查2处的可见性，如果不可见，不发送了
    Vector3d aim_pose;
    for(int i = 1; i < ed_->grid_tour2_.size(); i++) {
        // 计算距离
        double dis = (positions[0] - ed_->grid_tour2_[i]).head(2).norm();
        if(dis > 2.0) {
            aim_pose = ed_->grid_tour2_[i];
            // path_res.push_back(tour_positions[i]);
            break;
        }
        aim_pose = ed_->grid_tour2_[i];
    }

    Eigen::Vector3i idx;
    bool safe = true;
    caster_->input(positions[0], aim_pose);
    while (caster_->nextId(idx)) {
        if (sdf_map_->getInflateOccupancy(idx) == 1 || !sdf_map_->isInBox(idx)) {
        // map_->getOccupancy(idx) == SDFMap::UNKNOWN
        safe = false;
        break;
        }
    }

    // 发布grid_tour2给自己
    msg_utils::GridTour grid_tour2_msg;
    for(int i = 0; i < ed_->grid_tour2_.size(); i++) {
        geometry_msgs::Point grid_tour2_point;
        grid_tour2_point.x = ed_->grid_tour2_[i](0);
        grid_tour2_point.y = ed_->grid_tour2_[i](1);
        grid_tour2_point.z = ed_->grid_tour2_[i](2);
        grid_tour2_msg.points.push_back(grid_tour2_point);
        geometry_msgs::Point grid_tour2_point2;
        grid_tour2_point.x = ed_->grid_tour_[i](0);
        grid_tour2_point.y = ed_->grid_tour_[i](1);
        grid_tour2_point.z = ed_->grid_tour_[i](2);
        grid_tour2_msg.points2.push_back(grid_tour2_point2);
    }
    grid_tour2_msg.drone_id = ep_->drone_id_;
    grid_tour2_msg.stamp = ros::Time::now().toSec();
    if(safe)
    grid_tour2_pub.publish(grid_tour2_msg);
    
    nav_msgs::Path path_vis;
    path_vis.header.frame_id = "map";
    path_vis.header.stamp = ros::Time::now();
    for(int i = 0; i < ed_->grid_tour2_.size(); i++) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = ed_->grid_tour2_[i](0);
        pose.pose.position.y = ed_->grid_tour2_[i](1);
        pose.pose.position.z = ed_->grid_tour2_[i](2);
        pose.pose.orientation.w = 1;
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        path_vis.poses.push_back(pose);
    }
    vis_pub_.publish(path_vis);

    ed_->last_grid_ids_ = grid_ids;
    ed_->reallocated_ = false;

    // hgrid_->checkFirstGrid(grid_ids.front());
    visualize(2);   
    return true;
}

void FastExplorationFSM::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
    vector<int>& indices) {

    auto t1 = ros::Time::now();

    vector<Eigen::Vector3d> positions = { cur_pos };
    vector<Eigen::Vector3d> velocities = { cur_vel };
    vector<double> yaws = { cur_yaw[0] };

    // frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, mat);
    Eigen::MatrixXd mat;
    getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
    const int dimension = mat.rows();
    // std::cout << "dim of frontier TSP mat: " << dimension << std::endl;

    double mat_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("mat time: %lf", mat_time);

    // Find optimal allocation through AmTSP
    t1 = ros::Time::now();

    // Create problem file
    ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : amtsp\n";
    file << "TYPE : ATSP\n";
    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
        }
        file << "\n";
    }
    file.close();

    // Create par file
    const int drone_num = 1;

    file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
    file << "MTSP_MAX_SIZE = "
        << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
    file << "RUNS = 1\n";
    file << "TRACE_LEVEL = 0\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";
    t1 = ros::Time::now();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 1;
    if (!tsp_client_.call(srv)) {
        ROS_ERROR("Fail to solve ATSP.");
        return;
    }

    double mtsp_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("AmTSP time: %lf", mtsp_time);

    // Read results
    t1 = ros::Time::now();

    ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
        if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
        int id = stoi(res);
        ids.push_back(id - 1);
        if (id == -1) break;
    }
    fin.close();

    // Parse the m-tour
    vector<vector<int>> tours;
    vector<int> tour;
    for (auto id : ids) {
        if (id > 0 && id <= drone_num) {
        tour.clear();
        tour.push_back(id);
        } else if (id >= dimension || id <= 0) {
        tours.push_back(tour);
        } else {
        tour.push_back(id);
        }
    }

    vector<vector<int>> others(drone_num - 1);
    for (int i = 1; i < tours.size(); ++i) {
        if (tours[i][0] == 1) {
        indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
        }
        // else {
        //   others[tours[i][0] - 2].insert(
        //       others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
        // }
    }
    for (auto& id : indices) {
        id -= 1 + drone_num;
    }
    // for (auto& other : others) {
    //   for (auto& id : other)
    //     id -= 1 + drone_num;
    // }

    if (ed_->grid_tour_.size() > 2) {  // Remove id for next grid, since it is considered in the TSP
        indices.pop_back();
    }
    // Subset of frontier inside first grid
    for (int i = 0; i < indices.size(); ++i) {
        indices[i] = ftr_ids[indices[i]];
    }

    // Get the path of optimal tour from path matrix
    // frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
    // if (!grid_pos.empty()) {
    //     ed_->frontier_tour_.push_back(grid_pos[0]);
    // }

    // ed_->other_tours_.clear();
    // for (int i = 1; i < positions.size(); ++i) {
    //   ed_->other_tours_.push_back({});
    //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
    // }

    double parse_time = (ros::Time::now() - t1).toSec();
    // ROS_INFO("Cost mat: %lf, TSP: %lf, parse: %f, %d frontiers assigned.", mat_time, mtsp_time,
    //     parse_time, indices.size());
}

void FastExplorationFSM::updateFrontierStates() {
    // 重点是填充这个数据：list<Frontier> frontiers_;
    ed_->averages_.clear();
    for (auto& ftr : frontiers_svp) {
        for(auto& f : ftr) {
            ed_->averages_.push_back(f.average_);
        } 
    }
}

void FastExplorationFSM::updateFrontierStates(const msg_utils::FrontierInfo msg) {
    // ROS_WARN("updateFrontierStates(const msg_utils::FrontierInfo msg) is start");
    // frontiers_svp[msg.robot_id].clear();
    // 以下可用
    // list<Frontier> frontiers_svp_tmp;
    // for(int i = 0; i < msg.super_viewpoints.size(); i++) {
    //     msg_utils::SuperViewPoint cur_svp = msg.super_viewpoints[i];

    //     Frontier fr_temp;
    //     fr_temp.id_ = cur_svp.super_viewpoin_id;
    //     fr_temp.id_car_ = cur_svp.robot_id;
    //     fr_temp.average_ = Eigen::Vector3d(cur_svp.super_viewpoint.x, cur_svp.super_viewpoint.y, cur_svp.super_viewpoint.z);

    //     frontiers_svp_tmp.push_back(fr_temp);

    // }
    // frontiers_svp[msg.robot_id] = frontiers_svp_tmp;

    list<Frontier> frontiers_svp_tmp;
    for(int i = 0; i < msg.super_viewpoints.size(); i++) {
        msg_utils::SuperViewPoint cur_svp = msg.super_viewpoints[i];

        Frontier fr_temp;
        fr_temp.id_ = cur_svp.super_viewpoin_id;
        fr_temp.id_car_ = cur_svp.robot_id;
        fr_temp.average_ = Eigen::Vector3d(cur_svp.super_viewpoint.x, cur_svp.super_viewpoint.y, cur_svp.super_viewpoint.z);
        fast_planner::Viewpoint cur_vp_tmp;
        cur_vp_tmp.pos_ = Eigen::Vector3d(cur_svp.super_viewpoint.x, cur_svp.super_viewpoint.y, cur_svp.super_viewpoint.z);
        cur_vp_tmp.yaw_ = 0;
        fr_temp.viewpoints_.push_back(cur_vp_tmp);

        // for(int j = 0; j < msg.super_viewpoints[i].viewpoints.size(); j++) {
        //     fast_planner::Viewpoint cur_vp_tmp;
        //     cur_vp_tmp.pos_ = Eigen::Vector3d(msg.super_viewpoints[i].viewpoints[j].viewpoint.x, 
        //                                       msg.super_viewpoints[i].viewpoints[j].viewpoint.y, 
        //                                       msg.super_viewpoints[i].viewpoints[j].viewpoint.z);
        //     cur_vp_tmp.yaw_ = 0;
        //     fr_temp.viewpoints_.push_back(cur_vp_tmp);
        // }


        frontiers_svp_tmp.push_back(fr_temp);

    }
    frontiers_svp[msg.robot_id] = frontiers_svp_tmp;
}

void FastExplorationFSM::getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double>& yaws, const vector<int>& ftr_ids,
    const vector<Eigen::Vector3d>& grid_pos, Eigen::MatrixXd& mat) {

  Eigen::MatrixXd full_mat;
  getSwarmCostMatrix(positions, velocities, yaws, full_mat);

  // Get part of the full matrix according to selected frontier

  const int drone_num = positions.size();
  const int ftr_num = ftr_ids.size();
  int dimen = 1 + drone_num + ftr_num;
  if (!grid_pos.empty()) dimen += 1;

  mat = Eigen::MatrixXd::Zero(dimen, dimen);

  // Virtual depot to drones
  for (int i = 0; i < drone_num; ++i) {
    mat(0, 1 + i) = -1000;
    mat(1 + i, 0) = 1000;
  }
  // Virtual depot to frontiers
  for (int i = 0; i < ftr_num; ++i) {
    mat(0, 1 + drone_num + i) = 1000;
    mat(1 + drone_num + i, 0) = 0;
  }
  // Costs between drones
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < drone_num; ++j) {
      mat(1 + i, 1 + j) = 10000;
    }
  }

  // Costs from drones to frontiers
  for (int i = 0; i < drone_num; ++i) {
    for (int j = 0; j < ftr_num; ++j) {
      mat(1 + i, 1 + drone_num + j) = full_mat(1 + i, 1 + drone_num + ftr_ids[j]);
      mat(1 + drone_num + j, 1 + i) = 0;
    }
  }
  // Costs between frontiers
  for (int i = 0; i < ftr_num; ++i) {
    for (int j = 0; j < ftr_num; ++j) {
      mat(1 + drone_num + i, 1 + drone_num + j) =
          full_mat(1 + drone_num + ftr_ids[i], 1 + drone_num + ftr_ids[j]);
    }
  }
  // Diag
  for (int i = 0; i < dimen; ++i) {
    mat(i, i) = 1000;
  }

  // Consider next grid in global tour
  if (!grid_pos.empty()) {
    // Depot, 1000, -1000
    mat(0, 1 + drone_num + ftr_num) = 1000;
    mat(1 + drone_num + ftr_num, 0) = -1000;

    // Drone
    for (int i = 0; i < drone_num; ++i) {
      mat(1 + i, 1 + drone_num + ftr_num) = 1000;
      mat(1 + drone_num + ftr_num, 1 + i) = 1000;
    }

    // Frontier
    vector<Eigen::Vector3d> points, tmps;
    vector<double> yaws;
    getTopViewpointsInfo(positions[0], points, yaws, tmps);
    Eigen::Vector3d next_grid = grid_pos[0];

    for (int i = 0; i < ftr_num; ++i) {
      double cost = computeCost(
          next_grid, points[ftr_ids[i]], 0, 0, Eigen::Vector3d(0, 0, 0), 0, tmps);
      mat(1 + drone_num + i, 1 + drone_num + ftr_num) = cost;
      mat(1 + drone_num + ftr_num, 1 + drone_num + i) = cost;
    }
  }
}

void FastExplorationFSM::getTopViewpointsInfo(const Vector3d& cur_pos, vector<Eigen::Vector3d>& points,
    vector<double>& yaws, vector<Eigen::Vector3d>& averages) {
  points.clear();
  yaws.clear();
  averages.clear();

  for (auto frontier2 : frontiers_svp) {
    for(auto frontier : frontier2) {
        bool no_view = true;
        for (auto view : frontier.viewpoints_) {
        // Retrieve the first viewpoint that is far enough and has highest coverage
        if ((view.pos_ - cur_pos).norm() < min_candidate_dist_) continue;
        points.push_back(view.pos_);
        yaws.push_back(view.yaw_);
        averages.push_back(frontier.average_);
        no_view = false;
        break;
        }
        if (no_view) {
        // All viewpoints are very close, just use the first one (with highest coverage).
        auto view = frontier.viewpoints_.front();
        points.push_back(view.pos_);
        yaws.push_back(view.yaw_);
        averages.push_back(frontier.average_);
        }
    }

  }
}

void FastExplorationFSM::getSwarmCostMatrix(const vector<Vector3d>& positions,
    const vector<Vector3d>& velocities, const vector<double> yaws, Eigen::MatrixXd& mat) {

    const int drone_num = positions.size();

    int ftr_num1 = 0; // 前沿点 TODO
    for(auto ftr : frontiers_svp) {
        ftr_num1 += ftr.size();
    }
    const int ftr_num = ftr_num1;

    const int dimen = 1 + drone_num + ftr_num;
    mat = Eigen::MatrixXd::Zero(dimen, dimen);

    // Virtual depot to drones
    for (int i = 0; i < drone_num; ++i) {
        mat(0, 1 + i) = -1000;
        mat(1 + i, 0) = 1000;
    }
    // Virtual depot to frontiers
    for (int i = 0; i < ftr_num; ++i) {
        mat(0, 1 + drone_num + i) = 1000;
        mat(1 + drone_num + i, 0) = 0;
    }
    // Costs between drones
    for (int i = 0; i < drone_num; ++i) {
        for (int j = 0; j < drone_num; ++j) {
        mat(1 + i, 1 + j) = 10000;
        }
    }

    // Costs from drones to frontiers
    for (int i = 0; i < drone_num; ++i) {
        int j = 0;
        // TODO 遍历全部的前沿点，计算代价值
        for (auto ftr : frontiers_svp) {
            for(auto f : ftr) {
                Viewpoint vj = f.viewpoints_.front();
                vector<Vector3d> path;
                mat(1 + i, 1 + drone_num + j) =
                    computeCost(positions[i], vj.pos_, yaws[0], vj.yaw_, velocities[i], 0.0, path);
                mat(1 + drone_num + j, 1 + i) = 0;
                ++j;
            }

        }
    }
    // Costs between frontiers
    int i = 0, j = 0;
    for (auto ftr : frontiers_svp) {
        for(auto f : ftr) {
            for (auto cs : f.costs_) {
            mat(1 + drone_num + i, 1 + drone_num + j) = cs;
            ++j;
            }
            ++i;
            j = 0;
        }

    }
    // Diag
    for (int i = 0; i < dimen; ++i) {
        mat(i, i) = 1000;
    }

    // std::cout << "mat: " << std::endl;
    // std::cout << mat << std::endl;
}

double FastExplorationFSM::computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1,
    const double& y2, const Vector3d& v1, const double& yd1, vector<Vector3d>& path) {
  // Cost of position change
  double pos_cost = searchPath(p1, p2, path) / vm_;

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir = (p2 - p1).normalized();
    Vector3d vdir = v1.normalized();
    double diff = acos(vdir.dot(dir));
    pos_cost += w_dir_ * diff;
    // double vc = v1.dot(dir);
    // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
    // if (vc < 0)
    //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
  }

  // Cost of yaw change
  double diff = fabs(y2 - y1);
  diff = min(diff, 2 * M_PI - diff);
  double yaw_cost = diff / yd_;
  return max(pos_cost, yaw_cost);

  // // Consider yaw rate change
  // if (fabs(yd1) > 1e-3)
  // {
  //   double diff1 = y2 - y1;
  //   while (diff1 < -M_PI)
  //     diff1 += 2 * M_PI;
  //   while (diff1 > M_PI)
  //     diff1 -= 2 * M_PI;
  //   double diff2 = diff1 > 0 ? diff1 - 2 * M_PI : 2 * M_PI + diff1;
  // }
  // else
  // {
  // }
}

double FastExplorationFSM::searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path) {
    // Try connect two points with straight line
    bool safe = true;
    Eigen::Vector3i idx;
    caster_->input(p1, p2);
    while (caster_->nextId(idx)) {
        if (sdf_map_->getInflateOccupancy(idx) == 1 || !sdf_map_->isInBox(idx)) {
        // map_->getOccupancy(idx) == SDFMap::UNKNOWN
        safe = false;
        break;
        }
    }
    if (safe) {
        path = { p1, p2 };
        return (p1 - p2).norm();
    }
    // Search a path using decreasing resolution
    vector<double> res = { 0.4 };
    for (int k = 0; k < res.size(); ++k) {
        astar_->reset();
        astar_->setResolution(res[k]);
        if (astar_->search(p1, p2) == Astar::REACH_END) {
        path = astar_->getPath();
        return astar_->pathLength(path);
        }
    }
    // Use Astar early termination cost as an estimate
    path = { p1, p2 };
    return 100;
}

void FastExplorationFSM::visualize(vector<vector<int>>& ego_ids) {
    auto grid_tour = ego_ids;
    // 转到坐标点序列
    
    // auto grid_tour = expl_manager_->ed_->grid_tour2_;
    // for (auto& pt : grid_tour) pt = pt + trans;

    // if (1) {
    //   vector<Eigen::Vector3d> pts1, pts2;
    //   hgrid_->getGridMarker(pts1, pts2);
    //   visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 4);

    //   vector<Eigen::Vector3d> pts;
    //   vector<string> texts;
    //   hgrid_->getGridMarker2(pts, texts);
    //   static int last_text_num = 0;
    //   for (int i = 0; i < pts.size(); ++i) {
    //     visualization_->drawText(pts[i], texts[i], 1.5, Eigen::Vector4d(0, 0, 0, 1), "text", i, 4);
    //   }
    //   for (int i = pts.size(); i < last_text_num; ++i) {
    //     visualization_->drawText(
    //         Eigen::Vector3d(0, 0, 0), string(""), 1.3, Eigen::Vector4d(0, 0, 0, 1), "text", i, 4);
    //   }
    //   last_text_num = pts.size();
    // }

    for(int i = 0; i < grid_tour.size(); i++) {
        // std::cout << "grid_tour[" << i << "]: ";
        // for(int j = 0; j < grid_tour[i].size(); j++) {
        //     std::cout << grid_tour[i][j] << ", ";
        // }
        // std::cout << "" << std::endl;
        vector<Eigen::Vector3d> grid_tour_vec, grid_tour_vec2;
        hgrid_->getGridTour(grid_tour[i], ed_->swarm_state_[i].pos_, grid_tour_vec, grid_tour_vec2);
        string name_ns = "grid_tour" + to_string(i+1);
        visualization_->drawLines(grid_tour_vec, 0.05,
        PlanningVisualization::getColor(
            (i) / double(ep_->drone_num_)),
        name_ns, i, 4);
    }

}

void FastExplorationFSM::visualize(int content) {

  if (content == 1) {

    // if (expl_manager_->ep_->drone_id_ == 1) {
    // if (1) {
    //   vector<Eigen::Vector3d> pts1, pts2;
    //   hgrid_->getGridMarker(pts1, pts2);
    //   visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 6);

    //   vector<Eigen::Vector3d> pts;
    //   vector<string> texts;
    //   hgrid_->getGridMarker2(pts, texts);
    //   static int last_text_num = 0;
    //   for (int i = 0; i < pts.size(); ++i) {
    //     visualization_->drawText(pts[i], texts[i], 0.5, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
    //   }
    //   for (int i = pts.size(); i < last_text_num; ++i) {
    //     visualization_->drawText(
    //         Eigen::Vector3d(0, 0, 0), string(""), 0.3, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
    //   }
    //   last_text_num = pts.size();
    // }

    // auto grid_tour = ed_->grid_tour_;
    // // auto grid_tour = expl_manager_->ed_->grid_tour2_;
    // // for (auto& pt : grid_tour) pt = pt + trans;

    // visualization_->drawLines(grid_tour, 0.05,
    //     PlanningVisualization::getColor(
    //         (ep_->drone_id_ - 1) / double(ep_->drone_num_)),
    //     "grid_tour", 0, 6);

    // 分别画出每个无人机的路径
    auto grid_tour = ed_->grid_tour_;
    // auto grid_tour = expl_manager_->ed_->grid_tour2_;
    // for (auto& pt : grid_tour) pt = pt + trans;

    visualization_->drawLines(grid_tour, 0.05,
        PlanningVisualization::getColor(
            (ep_->drone_id_ - 1) / double(ep_->drone_num_)),
        "grid_tour", 0, 6);

  } else if (content == 2) {

    // Hierarchical grid and global tour --------------------------------
    // vector<Eigen::Vector3d> pts1, pts2;
    // expl_manager_->uniform_grid_->getPath(pts1, pts2);
    // visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0.3, 0, 1), "partition", 0,
    // 6);

    // if (expl_manager_->ep_->drone_id_ == 1) {
    if (1) {
      vector<Eigen::Vector3d> pts1, pts2;
      hgrid_->getGridMarker(pts1, pts2);
      visualization_->drawLines(pts1, pts2, 0.05, Eigen::Vector4d(1, 0, 1, 0.5), "partition", 1, 6);

      vector<Eigen::Vector3d> pts;
      vector<string> texts;
      hgrid_->getGridMarker2(pts, texts);
      static int last_text_num = 0;
      for (int i = 0; i < pts.size(); ++i) {
        visualization_->drawText(pts[i], texts[i], 0.5, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      for (int i = pts.size(); i < last_text_num; ++i) {
        visualization_->drawText(
            Eigen::Vector3d(0, 0, 0), string(""), 0.3, Eigen::Vector4d(0, 0, 0, 1), "text", i, 6);
      }
      last_text_num = pts.size();
    }

    auto grid_tour = ed_->grid_tour_;
    // auto grid_tour = expl_manager_->ed_->grid_tour2_;
    // for (auto& pt : grid_tour) pt = pt + trans;

    visualization_->drawLines(grid_tour, 0.05,
        PlanningVisualization::getColor(
            (ep_->drone_id_ - 1) / double(ep_->drone_num_)),
        "grid_tour", 0, 6);

    // Publish grid tour to ground node
    // task_allocation::GridTour tour;
    // for (int i = 0; i < grid_tour.size(); ++i) {
    //   geometry_msgs::Point point;
    //   point.x = grid_tour[i][0];
    //   point.y = grid_tour[i][1];
    //   point.z = grid_tour[i][2];
    //   tour.points.push_back(point);
    // }
    // tour.drone_id = expl_manager_->ep_->drone_id_;
    // tour.stamp = ros::Time::now().toSec();
    // grid_tour_pub_.publish(tour);

    // visualization_->drawSpheres(
    //     expl_manager_->ed_->grid_tour_, 0.3, Eigen::Vector4d(0, 1, 0, 1), "grid_tour", 1, 6);
    // visualization_->drawLines(
    //     expl_manager_->ed_->grid_tour2_, 0.05, Eigen::Vector4d(0, 1, 0, 0.5), "grid_tour", 2, 6);

    // Top viewpoints and frontier tour-------------------------------------

    // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1), "point-average", 0, 6);

    // auto frontier = ed_ptr->frontier_tour_;
    // for (auto& pt : frontier) pt = pt + trans;
    // visualization_->drawLines(frontier, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "frontier_tour", 0, 6);

    // for (int i = 0; i < ed_ptr->other_tours_.size(); ++i) {
    //   visualization_->drawLines(
    //       ed_ptr->other_tours_[i], 0.07, Eigen::Vector4d(0, 0, 1, 1), "other_tours", i, 6);
    // }

    // Locally refined viewpoints and refined tour-------------------------------

    // visualization_->drawSpheres(
    //     ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05, Vector4d(0.5, 0, 1, 1),
    //     "refined_view", 0, 6);
    // visualization_->drawLines(
    //     ed_ptr->refined_tour_, 0.07,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_), 0.6),
    //     "refined_tour", 0, 6);

    // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0,
    // 0, 0, 1),
    //                           "refined_view", 0, 6);
    // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05,
    // Vector4d(1, 1, 0, 1),
    //                           "refine_pair", 0, 6);
    // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
    //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
    //                               ed_ptr->frontiers_.size()),
    //                               "n_points", i, 6);
    // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
    //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

    // Trajectory-------------------------------------------

    // visualization_->drawSpheres(
    //     { ed_ptr->next_goal_ /* + trans */ }, 0.3, Vector4d(0, 0, 1, 1), "next_goal", 0, 6);

    // vector<Eigen::Vector3d> next_yaw_vis;
    // next_yaw_vis.push_back(ed_ptr->next_goal_ /* + trans */);
    // next_yaw_vis.push_back(
    //     ed_ptr->next_goal_ /* + trans */ +
    //     2.0 * Eigen::Vector3d(cos(ed_ptr->next_yaw_), sin(ed_ptr->next_yaw_), 0));
    // visualization_->drawLines(next_yaw_vis, 0.1, Eigen::Vector4d(0, 0, 1, 1), "next_goal", 1, 6);
    // visualization_->drawSpheres(
    //     { ed_ptr->next_pos_ /* + trans */ }, 0.3, Vector4d(0, 1, 0, 1), "next_pos", 0, 6);

    // Eigen::MatrixXd ctrl_pt = info->position_traj_.getControlPoint();
    // for (int i = 0; i < ctrl_pt.rows(); ++i) {
    //   for (int j = 0; j < 3; ++j) ctrl_pt(i, j) = ctrl_pt(i, j) + trans[j];
    // }
    // NonUniformBspline position_traj(ctrl_pt, 3, info->position_traj_.getKnotSpan());

    // visualization_->drawBspline(info->position_traj_, 0.1,
    //     PlanningVisualization::getColor(
    //         (expl_manager_->ep_->drone_id_ - 1) / double(expl_manager_->ep_->drone_num_)),
    //     false, 0.15, Vector4d(1, 1, 0, 1));

    // visualization_->drawLines(
    //     expl_manager_->ed_->path_next_goal_, 0.1, Eigen::Vector4d(0, 1, 0, 1), "astar", 0, 6);
    // visualization_->drawSpheres(
    //     expl_manager_->ed_->kino_path_, 0.1, Eigen::Vector4d(0, 0, 1, 1), "kino", 0, 6);
    // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0,
    // 0); visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1),
    // "next_goal", 1, 6);

    // // Draw trajs of other drones
    // vector<NonUniformBspline> trajs;
    // planner_manager_->swarm_traj_data_.getValidTrajs(trajs);
    // for (int k = 0; k < trajs.size(); ++k) {
    //   visualization_->drawBspline(trajs[k], 0.1, Eigen::Vector4d(1, 1, 0, 1), false, 0.15,
    //       Eigen::Vector4d(0, 0, 1, 1), k + 1);
    // }
  }
}


/**************************功能模块2**************************************************/
/**
 * @brief 分配网格
 * 
 * @param positions 位置
 * @param velocities 速度
 * @param first_ids 第一层级第一个大网格中的小网格，各个机器人攒一块的
 * @param second_ids 第一层级第二个大网格中的小网格，各个机器人攒一块的
 * @param grid_ids 网格id
 * 
 * @param ego_ids 序列
 * @param other_ids 其他id
*/
void FastExplorationFSM::allocateGrids(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids,
    vector<int>& other_ids) {
    // ROS_INFO("Allocate grid.");

    auto t1 = ros::Time::now();
    auto t2 = t1;

    if (grid_ids.size() == 1) {  // Only one grid, no need to run ACVRP
        auto pt = hgrid_->getCenter(grid_ids.front());
        // double d1 = (positions[0] - pt).norm();
        // double d2 = (positions[1] - pt).norm();
        vector<Eigen::Vector3d> path;
        double d1 = computeCost(positions[0], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
        double d2 = computeCost(positions[1], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
        if (d1 < d2) {
        ego_ids = grid_ids;
        other_ids = {};
        } else {
        ego_ids = {};
        other_ids = grid_ids;
        }
        return;
    }

    Eigen::MatrixXd mat;
    // uniform_grid_->getCostMatrix(positions, velocities, prev_first_ids, grid_ids, mat);
    hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat);

    // int unknown = hgrid_->getTotalUnknwon();
    int unknown;

    double mat_time = (ros::Time::now() - t1).toSec();

    // Find optimal path through AmTSP
    t1 = ros::Time::now();
    const int dimension = mat.rows();
    const int drone_num = positions.size();

    vector<int> unknown_nums;
    int capacity = 0;
    for (int i = 0; i < grid_ids.size(); ++i) {
        int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
        unknown_nums.push_back(unum);
        capacity += unum;
        // std::cout << "Grid " << i << ": " << unum << std::endl;
    }
    // std::cout << "Total: " << capacity << std::endl;
    capacity = capacity * 0.75 * 0.1;

    // int prob_type;
    // if (grid_ids.size() >= 3)
    //   prob_type = 2;  // Use ACVRP
    // else
    //   prob_type = 1;  // Use AmTSP

    const int prob_type = 2;

    // Create problem file--------------------------
    ofstream file(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : pairopt\n";

    if (prob_type == 1)
        file << "TYPE : ATSP\n";
    else if (prob_type == 2)
        file << "TYPE : ACVRP\n";

    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

    if (prob_type == 2) {
        file << "CAPACITY : " + to_string(capacity) + "\n";   // ACVRP
        file << "VEHICLES : " + to_string(drone_num) + "\n";  // ACVRP
    }

    // Cost matrix
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
        }
        file << "\n";
    }

    if (prob_type == 2) {  // Demand section, ACVRP only
        file << "DEMAND_SECTION\n";
        file << "1 0\n";
        for (int i = 0; i < drone_num; ++i) {
        file << to_string(i + 2) + " 0\n";
        }
        for (int i = 0; i < grid_ids.size(); ++i) {
        int grid_unknown = unknown_nums[i] * 0.1;
        file << to_string(i + 2 + drone_num) + " " + to_string(grid_unknown) + "\n";
        }
        file << "DEPOT_SECTION\n";
        file << "1\n";
        file << "EOF";
    }

    file.close();

    // Create par file------------------------------------------
    int min_size = int(grid_ids.size()) / 2;
    int max_size = ceil(int(grid_ids.size()) / 2.0);
    file.open(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp\n";
    if (prob_type == 1) {
        file << "SALESMEN = " << to_string(drone_num) << "\n";
        file << "MTSP_OBJECTIVE = MINSUM\n";
        // file << "MTSP_OBJECTIVE = MINMAX\n";
        file << "MTSP_MIN_SIZE = " << to_string(min_size) << "\n";
        file << "MTSP_MAX_SIZE = " << to_string(max_size) << "\n";
        file << "TRACE_LEVEL = 0\n";
    } else if (prob_type == 2) {
        file << "TRACE_LEVEL = 1\n";  // ACVRP
        file << "SEED = 0\n";         // ACVRP
    }
    file << "RUNS = 1\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour\n";

    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp";
    t1 = ros::Time::now();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 3;
    // if (!tsp_client_.call(srv)) {
    if (!acvrp_client_.call(srv)) {
        ROS_ERROR("Fail to solve ACVRP.");
        return;
    }
    // system("/home/boboyu/software/LKH-3.0.6/LKH
    // /home/boboyu/workspaces/hkust_swarm_ws/src/swarm_exploration/utils/lkh_mtsp_solver/resource/amtsp3_1.par");

    double mtsp_time = (ros::Time::now() - t1).toSec();
    std::cout << "Allocation time1 this: " << mtsp_time << std::endl;
std::cout << "file sys " << std::endl;
    // Read results
    t1 = ros::Time::now();
std::cout << "file sys start " << std::endl;
    ifstream fin(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour");
std::cout << "file sys end " << std::endl;
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
        if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
        int id = stoi(res);
        ids.push_back(id - 1);
        if (id == -1) break;
    }
    fin.close();

    // std::cout << "ACVRP ansys start " << std::endl;
    // Parse the m-tour of grid
    vector<vector<int>> tours;
    vector<int> tour;
    for (auto id : ids) {
        if (id > 0 && id <= drone_num) {
        tour.clear();
        tour.push_back(id);
        } else if (id >= dimension || id <= 0) {
        tours.push_back(tour);
        } else {
        tour.push_back(id);
        }
    }
    // // Print tour ids
    // for (auto tr : tours) {
    //   std::cout << "tour: ";
    //   for (auto id : tr) std::cout << id << ", ";
    //   std::cout << "" << std::endl;
    // }

    for (int i = 1; i < tours.size(); ++i) {
        if (tours[i][0] == 1) {
        ego_ids.insert(ego_ids.end(), tours[i].begin() + 1, tours[i].end());
        } else {
        other_ids.insert(other_ids.end(), tours[i].begin() + 1, tours[i].end());
        }
    }
    for (auto& id : ego_ids) {
        id = grid_ids[id - 1 - drone_num];
    }
    for (auto& id : other_ids) {
        id = grid_ids[id - 1 - drone_num];
    }
    // std::cout << "ACVRP ansys end " << std::endl;
    // // Remove repeated grid
    // unordered_map<int, int> ego_map, other_map;
    // for (auto id : ego_ids) ego_map[id] = 1;
    // for (auto id : other_ids) other_map[id] = 1;

    // ego_ids.clear();
    // other_ids.clear();
    // for (auto p : ego_map) ego_ids.push_back(p.first);
    // for (auto p : other_map) other_ids.push_back(p.first);

    // sort(ego_ids.begin(), ego_ids.end());
    // sort(other_ids.begin(), other_ids.end());
}


/**
 * @brief 分配网格
 * 
 * @param positions 位置
 * @param velocities 速度
 * @param first_ids 第一层级第一个大网格中的小网格，各个机器人攒一块的
 * @param second_ids 第一层级第二个大网格中的小网格，各个机器人攒一块的
 * @param grid_ids 网格id
 * 
 * @param ego_ids 优化后的访问序列集合
*/
void FastExplorationFSM::allocateGrids(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<vector<int>>& ego_ids) {
    // ROS_INFO("Allocate grid.");

    auto t1 = ros::Time::now();
    auto t2 = t1;

    if (grid_ids.size() == 1) {  // Only one grid, no need to run ACVRP
        auto pt = hgrid_->getCenter(grid_ids.front());
        // double d1 = (positions[0] - pt).norm();
        // double d2 = (positions[1] - pt).norm();
        vector<Eigen::Vector3d> path;
        double min_cost = 999999999;
        int min_id = -1;
        for(int i = 0; i < positions.size(); i++) {
            double d1 = computeCost(positions[i], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
            if(d1 < min_cost) {
                min_cost = d1;
                min_id = i;
            }
        }
        if(min_id != -1) {
            for(int i = 0; i < positions.size(); i++) {
                if(i == min_id) {
                    ego_ids.push_back(grid_ids);
                } else {
                    ego_ids.push_back({});
                }
            }
        } else {
            ROS_ERROR("grid_ids.size() == 1, min_cost is too large.");
            exit(0);
        }
        return;
    }

    Eigen::MatrixXd mat;
    // uniform_grid_->getCostMatrix(positions, velocities, prev_first_ids, grid_ids, mat);
    hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat);

    // int unknown = hgrid_->getTotalUnknwon();
    int unknown;

    double mat_time = (ros::Time::now() - t1).toSec();

    // Find optimal path through AmTSP
    t1 = ros::Time::now();
    const int dimension = mat.rows();
    const int drone_num = positions.size();

    vector<int> unknown_nums;
    int capacity = 0;
    for (int i = 0; i < grid_ids.size(); ++i) {
        int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
        unknown_nums.push_back(unum);
        capacity += unum;
        // std::cout << "Grid " << i << ": " << unum << std::endl;
    }
    // std::cout << "Total: " << capacity << std::endl;
    capacity = capacity * 0.75 * 0.1;

    // int prob_type;
    // if (grid_ids.size() >= 3)
    //   prob_type = 2;  // Use ACVRP
    // else
    //   prob_type = 1;  // Use AmTSP

    const int prob_type = 2;

    // Create problem file--------------------------
    ofstream file(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp");
    file << "NAME : pairopt\n";

    if (prob_type == 1)
        file << "TYPE : ATSP\n";
    else if (prob_type == 2)
        file << "TYPE : ACVRP\n";

    file << "DIMENSION : " + to_string(dimension) + "\n";
    file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

    if (prob_type == 2) {
        file << "CAPACITY : " + to_string(capacity) + "\n";   // ACVRP
        file << "VEHICLES : " + to_string(drone_num) + "\n";  // ACVRP
    }

    // Cost matrix
    file << "EDGE_WEIGHT_SECTION\n";
    for (int i = 0; i < dimension; ++i) {
        for (int j = 0; j < dimension; ++j) {
        int int_cost = 100 * mat(i, j);
        file << int_cost << " ";
        }
        file << "\n";
    }

    if (prob_type == 2) {  // Demand section, ACVRP only
        file << "DEMAND_SECTION\n";
        file << "1 0\n";
        for (int i = 0; i < drone_num; ++i) {
        file << to_string(i + 2) + " 0\n";
        }
        for (int i = 0; i < grid_ids.size(); ++i) {
        int grid_unknown = unknown_nums[i] * 0.1;
        file << to_string(i + 2 + drone_num) + " " + to_string(grid_unknown) + "\n";
        }
        file << "DEPOT_SECTION\n";
        file << "1\n";
        file << "EOF";
    }

    file.close();

    // Create par file------------------------------------------
    int min_size = int(grid_ids.size()) / 2;
    int max_size = ceil(int(grid_ids.size()) / 2.0);
    file.open(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par");
    file << "SPECIAL\n";
    file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp\n";
    if (prob_type == 1) {
        file << "SALESMEN = " << to_string(drone_num) << "\n";
        file << "MTSP_OBJECTIVE = MINSUM\n";
        // file << "MTSP_OBJECTIVE = MINMAX\n";
        file << "MTSP_MIN_SIZE = " << to_string(min_size) << "\n";
        file << "MTSP_MAX_SIZE = " << to_string(max_size) << "\n";
        file << "TRACE_LEVEL = 0\n";
    } else if (prob_type == 2) {
        file << "TRACE_LEVEL = 1\n";  // ACVRP
        file << "SEED = 0\n";         // ACVRP
    }
    file << "RUNS = 1\n";
    file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour\n";

    file.close();

    auto par_dir = ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp";
    t1 = ros::Time::now();

    lkh_mtsp_solver::SolveMTSP srv;
    srv.request.prob = 3;
    // if (!tsp_client_.call(srv)) {
    if (!acvrp_client_.call(srv)) {
        ROS_ERROR("Fail to solve ACVRP.");
        return;
    }

    double mtsp_time = (ros::Time::now() - t1).toSec();
    std::cout << "Allocation time: " << mtsp_time << std::endl;

    // Read results
    t1 = ros::Time::now();

    ifstream fin(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour");
    string res;
    vector<int> ids;
    while (getline(fin, res)) {
        if (res.compare("TOUR_SECTION") == 0) break;
    }
    while (getline(fin, res)) {
        int id = stoi(res);
        ids.push_back(id - 1);
        if (id == -1) break;
    }
    fin.close();
// std::cout << "ACVRP ansys start " << std::endl;
    // Parse the m-tour of grid
    vector<vector<int>> tours;
    vector<int> tour;
    for (auto id : ids) {
        if (id > 0 && id <= drone_num) {
        tour.clear();
        tour.push_back(id);
        } else if (id >= dimension || id <= 0) {
        tours.push_back(tour);
        } else {
        tour.push_back(id);
        }
    }
    // // Print tour ids
    // for (auto tr : tours) {
    //   std::cout << "tour: ";
    //   for (auto id : tr) std::cout << id << ", ";
    //   std::cout << "" << std::endl;
    // }

    vector<vector<int>> tours_s;
    for (auto& tr : tours) {
        if (!tr.empty()) {
            tours_s.push_back(tr);
        }
    }
    sort(tours_s.begin(), tours_s.end(), [](const vector<int>& a, const vector<int>& b) {
        return a[0] < b[0];
    });

    // for (auto tr : tours_s) {
    //   std::cout << "tour_s: ";
    //   for (auto id : tr) std::cout << id << ", ";
    //   std::cout << "" << std::endl;
    // }

    ego_ids = tours_s;

    // for (int i = 1; i < drone_num + 1; ++i) {
    //     for(int k = 0; k < drone_num; k++) {
    //         if (tours[i][0] == k + 1) {
    //             ego_ids[k].insert(ego_ids[k].end(), tours[i].begin() + 1, tours[i].end());
    //         }
    //     }
    //     // if (tours[i][0] == 1) {
    //     // ego_ids.insert(ego_ids.end(), tours[i].begin() + 1, tours[i].end());
    //     // } else {
    //     // other_ids.insert(other_ids.end(), tours[i].begin() + 1, tours[i].end());
    //     // }
    // }
// std::cout << "ACVRP ansys p2 " << std::endl;

    // 打印ego_ids
    // for (int i = 0; i < ego_ids.size(); i++) {
    //     std::cout << "ego_ids beforce " << i << ": ";
    //     for (auto id : ego_ids[i]) std::cout << id << ", ";
    //     std::cout << "" << std::endl;
    // }

    for(int j = 0; j < ego_ids.size(); j++) {
        for (auto& id : ego_ids[j]) {
            id = grid_ids[id - 1 - drone_num];
        }
    }
    // for(int k = 0; k < ego_ids.size(); k++) {
    //     if(ego_ids[k][0] == 0) {
    //         ego_ids[k].erase(ego_ids[k].begin());
    //     }
    // }

    // for (int i = 0; i < ego_ids.size(); i++) {
    //     std::cout << "ego_ids after " << i << ": ";
    //     for (auto id : ego_ids[i]) std::cout << id << ", ";
    //     std::cout << "" << std::endl;
    // }
// std::cout << "ACVRP ansys end " << std::endl;
    // for (auto& id : ego_ids) {
    //     id = grid_ids[id - 1 - drone_num];
    // }
    // for (auto& id : other_ids) {
    //     id = grid_ids[id - 1 - drone_num];
    // }
    // // Remove repeated grid
    // unordered_map<int, int> ego_map, other_map;
    // for (auto id : ego_ids) ego_map[id] = 1;
    // for (auto id : other_ids) other_map[id] = 1;

    // ego_ids.clear();
    // other_ids.clear();
    // for (auto p : ego_map) ego_ids.push_back(p.first);
    // for (auto p : other_map) other_ids.push_back(p.first);

    // sort(ego_ids.begin(), ego_ids.end());
    // sort(other_ids.begin(), other_ids.end());
}

void FastExplorationFSM::checkAllocateResult() {
    // Check if the allocation is better


    // if it is better, pub it.
    msg_utils::FreeSpaceAndFrontierInfo mcar_task_allocation_msg;
    mcar_task_allocation_pub.publish(mcar_task_allocation_msg);
}

double FastExplorationFSM::computeGridPathCost(const Eigen::Vector3d& pos,
    const vector<int>& grid_ids, const vector<int>& first, const vector<vector<int>>& firsts,
    const vector<vector<int>>& seconds, const double& w_f) {
  if (grid_ids.empty()) return 0.0;

  double cost = 0.0;
  vector<Eigen::Vector3d> path;
  cost += hgrid_->getCostDroneToGrid(pos, grid_ids[0], first);
  for (int i = 0; i < grid_ids.size() - 1; ++i) {
    cost += hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size());
  }
  return cost;
}


}  // namespace fast_planner
