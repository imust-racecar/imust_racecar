## find_nearest_pose.py
1. 构建一条 全局路径（读取文件）
2. 每次接收到里程计的信息
    1. 在 全局路径 中找到距离当前位置最近的点
    2. 把 最近点 的 index 和 数据 分别发送到两个话题

## adaptive_lookahead.py
1. 构建一条 数组（index -> 前瞻距离）
2. 每次接收到 find_nearest_pose.py 的 最近距离 的 index
    1. 把 对应的前瞻距离 发布到话题上

## find_nearest_goal.py
1. 构建一条 全局路径（读取文件）
2. 每次收到 最近点的 index
    1. 构建两个 goal pose
        1. 一个加角度前瞻距离，作为角度目标发布
        2. 一个加速度前瞻距离，作为速度目标发布
2. 每次接收到一个 前瞻（开启 adaptive_lookahead ）
    1. 根据状态 "break", "caution", 其他 判断 角度前瞻，距离前瞻 是 角度前瞻的 两倍

## vehicle_controller.py
1. 每次接收到 角度目标，存起来
1. 每次皆受到 速度目标，存起来
1. 每次接收到 里程计 的信息
    1. 计算打角
        1. 计算打角，当成 pid 中的 error
    2. 计算速度
        1. 通过之前接收到的信息，判断当前状态，从而进行刹车，caution，加速