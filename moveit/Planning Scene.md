

# Planning Scene

## 八叉树导入

### 创建八叉树

```bash
collision_detection::OccMapTreePtr createOctomap(const octomap_msgs::Octomap& map)
{
  std::shared_ptr<collision_detection::OccMapTree> om =
      std::make_shared<collision_detection::OccMapTree>(map.resolution);
  if (map.binary)
  {
    octomap_msgs::readTree(om.get(), map);
  }
  else
  {
    std::stringstream datastream;
    if (!map.data.empty())
    {
      datastream.write((const char*)&map.data[0], map.data.size());
      om->readData(datastream);
    }
  }
  return om;
}

```

| 特性     | binary           | 非binary      |
| -------- | ---------------- | ------------- |
| 数据内容 | 修建后的叶子节点 | 所有          |
| 体积     | 极小             | 极大ROS       |
| ROS默认  | 1                | 0             |
| Moveit   | 完美支持         | 支持但低效    |
| 适用     | 实时,部署        | 调试,兼容旧版 |

##### C++语法

```bash
std::make_shared<collision_detection::OccMapTree>(map.resolution);
```



### 导入planning_scene

#### 1.无位姿

```bash
void PlanningScene::processOctomapMsg(const octomap_msgs::Octomap& map)
{
  // each octomap replaces any previous one
  world_->removeObject(OCTOMAP_NS);

  if (map.data.empty())
    return;

  if (map.id != "OcTree")
  {
    ROS_ERROR_NAMED(LOGNAME, "Received octomap is of type '%s' but type 'OcTree' is expected.", map.id.c_str());
    return;
  }

  std::shared_ptr<collision_detection::OccMapTree> om = createOctomap(map);
  if (!map.header.frame_id.empty())
  {
    const Eigen::Isometry3d& t = getFrameTransform(map.header.frame_id);
    world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), t);
  }
  else
  {
    world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), Eigen::Isometry3d::Identity());
  }
}
```



>### octomap支持多种树类型(SemanticsOcTree,ColorOcTree),但moveit只支持OctTree,如果类型不对直接抛异常



|      | frame_id is empty     | frame_id is not                         |
| ---- | --------------------- | --------------------------------------- |
| 区别 | map默认在全局坐标系下 | map的坐标位于局部坐标系                 |
|      |                       | 如果位于连杆,则通过robotstate计算       |
|      |                       | 如果位于外部,比如相机,则通过TF2listener |



#### 2.有位姿

```bash
void PlanningScene::processOctomapMsg(const octomap_msgs::OctomapWithPose& map)
{
  // each octomap replaces any previous one
  world_->removeObject(OCTOMAP_NS);

  if (map.octomap.data.empty())
    return;

  if (map.octomap.id != "OcTree")
  {
    ROS_ERROR_NAMED(LOGNAME, "Received octomap is of type '%s' but type 'OcTree' is expected.", map.octomap.id.c_str());
    return;
  }

  std::shared_ptr<collision_detection::OccMapTree> om = createOctomap(map.octomap);

  const Eigen::Isometry3d& t = getFrameTransform(map.header.frame_id);
  Eigen::Isometry3d p;
  PlanningScene::poseMsgToEigen(map.origin, p);
  p = t * p;
  world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), p);
}
```



|        | 无位姿                      | 有位姿                                |
| ------ | --------------------------- | ------------------------------------- |
| 区别   | octomap原点即frame_id的原点 | map原点在frame_id有位姿               |
| t变换  | frame->world                | frame->world                          |
| p变换  | 无                          | origin->frame                         |
| 总变换 | t                           | p*t                                   |
| 适用   | 传感器实时建图              | 全局地图,SLAM后端,离线地图,多地图融合 |

##### C++语法

```bash
world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), t);
world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(om)), Eigen::Isometry3d::Identity());
```



## 碰撞检测

### 核心函数

```bash
void PlanningScene::checkCollision(const collision_detection::CollisionRequest& req,
                                   collision_detection::CollisionResult& res,
                                   const moveit::core::RobotState& robot_state,
                                   const collision_detection::AllowedCollisionMatrix& acm) const
{
  // check collision with the world using the padded version
  getCollisionEnv()->checkRobotCollision(req, res, robot_state, acm);

  // do self-collision checking with the unpadded version of the robot
  if (!res.collision || (req.contacts && res.contacts.size() < req.max_contacts))
    getCollisionEnvUnpadded()->checkSelfCollision(req, res, robot_state, acm);
}
```



|      | 带padding          | 无padding      |
| ---- | ------------------ | -------------- |
| 区别 | 机器人与环境的碰撞 | 机器人的自碰撞 |
|      | 膨胀处理过         | 无碰撞处理     |

### 状态碰撞检测

```bash
bool PlanningScene::isStateColliding(const moveit::core::RobotState& state, const std::string& group, bool verbose) const
{
  collision_detection::CollisionRequest req;
  #是否打印碰撞信息(调试)
  req.verbose = verbose;
  req.group_name = group;
  collision_detection::CollisionResult res;
  checkCollision(req, res, state);
  return res.collision;
}
```



### 获取碰撞信息

```bash
void PlanningScene::getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts)
{
  if (getCurrentState().dirtyCollisionBodyTransforms())
  #NonConst 表示会更新 getAllowedCollisionMatrix() 表示ACM(哪些碰撞可以忽略)
    getCollidingPairs(contacts, getCurrentStateNonConst(), getAllowedCollisionMatrix());
  else
    getCollidingPairs(contacts, getCurrentState(), getAllowedCollisionMatrix());
}

void PlanningScene::getCollidingPairs(collision_detection::CollisionResult::ContactMap& contacts,
                                      const moveit::core::RobotState& robot_state,
                                      const collision_detection::AllowedCollisionMatrix& acm,
                                      const std::string& group_name) const
{
  collision_detection::CollisionRequest req;
  req.contacts = true;
  #连杆数量+1 表示最大接触点对数,防止安全截断
  req.max_contacts = getRobotModel()->getLinkModelsWithCollisionGeometry().size() + 1;
  req.max_contacts_per_pair = 1;
  req.group_name = group_name;
  collision_detection::CollisionResult res;
  checkCollision(req, res, robot_state, acm);
  res.contacts.swap(contacts);
}
```



|                 | 无参            | 有参                  |
| --------------- | --------------- | --------------------- |
| 定义            | 获取当前默认ACM | 向碰撞函数传入任意ACM |
| 是否影响全局ACM | 否(只读)        | 否(局部使用)          |
| 用途            | 常规规划        | 临时策略              |
| 来源            | SRDF/初始化     | 用户构造              |



#### C++语法

```bash
getCollidingPairs(contacts, getCurrentStateNonConst(), getAllowedCollisionMatrix());
res.contacts.swap(contacts);
```





## 路径验证

```bash
#验证路径是否合法,不仅考虑碰撞,还考虑路径约束和目标约束
bool PlanningScene::isPathValid(const robot_trajectory::RobotTrajectory& trajectory,
                                const moveit_msgs::Constraints& path_constraints,
                                const std::vector<moveit_msgs::Constraints>& goal_constraints, const std::string& group,
                                bool verbose, std::vector<std::size_t>* invalid_index) const
{
  bool result = true;
  if (invalid_index)
    invalid_index->clear();
  kinematic_constraints::KinematicConstraintSet ks_p(getRobotModel());
  ks_p.add(path_constraints, getTransforms());
  std::size_t n_wp = trajectory.getWayPointCount();
  for (std::size_t i = 0; i < n_wp; ++i)
  {
    const moveit::core::RobotState& st = trajectory.getWayPoint(i);

    bool this_state_valid = true;
    #三大约束检验,AND关系
    if (isStateColliding(st, group, verbose))
      this_state_valid = false;
    if (!isStateFeasible(st, verbose))
      this_state_valid = false;
    if (!ks_p.empty() && !ks_p.decide(st, verbose).satisfied)
      this_state_valid = false;

    if (!this_state_valid)
    {
      if (invalid_index)
        invalid_index->push_back(i);
      else
        return false;
      result = false;
    }

    // check goal for last state
    # 只检验最后一个waypoint
    if (i + 1 == n_wp && !goal_constraints.empty())
    {
      bool found = false;
      # 多个目标约束满足一个即可
      for (const moveit_msgs::Constraints& goal_constraint : goal_constraints)
      {
        if (isStateConstrained(st, goal_constraint))
        {
          found = true;
          break;
        }
      }
      if (!found)
      {
        if (verbose)
          ROS_INFO_NAMED(LOGNAME, "Goal not satisfied");
        if (invalid_index)
          invalid_index->push_back(i);
        result = false;
      }
    }
  }
  return result;
}
```



># moveit_msgs::Constraints
>
>###### moveit_msgs::Constraints支持多种类型
>
>PositionConstraints
>
>OrientationConstraints
>
>VisibilityConstraints(相机可见性)
>
>JointConstraints
>
>###### getTransforms():坐标变换,可将一个约束从坐标一变换到坐标二(world->base_link)



## 实时的Octomap环境更新

```bash
void PlanningScene::processOctomapPtr(const std::shared_ptr<const octomap::OcTree>& octree, const Eigen::Isometry3d& t)
{
  collision_detection::CollisionEnv::ObjectConstPtr map = world_->getObject(OCTOMAP_NS);
  if (map)
  {
    if (map->shapes_.size() == 1)
    {
      // check to see if we have the same octree pointer & pose.
      const shapes::OcTree* o = static_cast<const shapes::OcTree*>(map->shapes_[0].get());
      if (o->octree == octree)
      {
        // if the pose changed, we update it
        if (map->shape_poses_[0].isApprox(t, std::numeric_limits<double>::epsilon() * 100.0))
        {
          if (world_diff_)
            world_diff_->set(OCTOMAP_NS, collision_detection::World::DESTROY | collision_detection::World::CREATE |
                                             collision_detection::World::ADD_SHAPE);
        }
        else
        {
          shapes::ShapeConstPtr shape = map->shapes_[0];
          map.reset();  // reset this pointer first so that caching optimizations can be used in CollisionWorld
          world_->moveShapeInObject(OCTOMAP_NS, shape, t);
        }
        return;
      }
    }
  }
  // if the octree pointer changed, update the structure
  world_->removeObject(OCTOMAP_NS);
  world_->addToObject(OCTOMAP_NS, shapes::ShapeConstPtr(new shapes::OcTree(octree)), t);
}
```

