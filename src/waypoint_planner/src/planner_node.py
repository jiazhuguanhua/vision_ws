#!/usr/bin/env python3
"""
Wildlife Patrol Waypoint Planner Node
野生动物巡查航点规划节点

功能：
- 基于10x10网格地图规划无人机巡查路径
- 避开禁飞区，遍历所有可通行点
- 使用A*算法优化路径规划
- 发布航点序列供飞行控制使用

作者：ROS Developer
日期：2025-07-30
版本：1.0.0
"""

import rospy
import yaml
import os
import numpy as np
from collections import deque
import heapq
from typing import List, Tuple, Set, Optional

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
from waypoint_planner.msg import PointArray


class GridCoordinate:
    """网格坐标系统处理类"""
    
    @staticmethod
    def parse_coordinate(coord_str: str) -> Tuple[int, int]:
        """
        解析坐标字符串 "A9B1" -> (9, 1)
        A对应行(1-9)，B对应列(1-9)
        """
        try:
            # 移除空格并转换为大写
            coord_str = coord_str.strip().upper()
            
            # 查找A和B的位置
            a_idx = coord_str.find('A')
            b_idx = coord_str.find('B')
            
            if a_idx == -1 or b_idx == -1:
                raise ValueError(f"Invalid coordinate format: {coord_str}")
            
            # 提取行号和列号
            row_str = coord_str[a_idx+1:b_idx]
            col_str = coord_str[b_idx+1:]
            
            row = int(row_str)
            col = int(col_str)
            
            # 验证范围
            if not (1 <= row <= 9) or not (1 <= col <= 9):
                raise ValueError(f"Coordinate out of range: {coord_str}")
            
            return (row, col)
            
        except Exception as e:
            rospy.logerr(f"❌ Failed to parse coordinate '{coord_str}': {e}")
            raise
    
    @staticmethod
    def coordinate_to_string(row: int, col: int) -> str:
        """将坐标转换为字符串格式 (9, 1) -> "A9B1" """
        return f"A{row}B{col}"
    
    @staticmethod
    def to_world_point(row: int, col: int, altitude: float = 1.2) -> Point:
        """
        将网格坐标转换为世界坐标点
        注意：这是2D规划问题，所有航点保持相同的固定飞行高度
        """
        point = Point()
        # 网格坐标(1,1)对应世界坐标(0,0)
        point.x = float(col - 1)
        point.y = float(row - 1)
        point.z = altitude  # 固定飞行高度，不变化
        return point


class PathPlanner:
    """路径规划算法类"""
    
    def __init__(self, grid_size: Tuple[int, int] = (9, 9)):
        self.rows, self.cols = grid_size
        self.obstacles = set()  # 禁飞区坐标集合
    
    def set_obstacles(self, obstacles: Set[Tuple[int, int]]):
        """设置禁飞区"""
        self.obstacles = obstacles
        rospy.loginfo(f"🚫 Set {len(obstacles)} no-fly zones: {obstacles}")
    
    def is_valid_position(self, row: int, col: int) -> bool:
        """检查位置是否有效（在边界内且不是禁飞区）"""
        return (1 <= row <= self.rows and 
                1 <= col <= self.cols and 
                (row, col) not in self.obstacles)
    
    def is_safe_diagonal_move(self, from_pos: Tuple[int, int], to_pos: Tuple[int, int]) -> bool:
        """
        检查对角线移动是否安全
        确保对角线移动不会"擦边"穿过禁飞区
        """
        from_row, from_col = from_pos
        to_row, to_col = to_pos
        
        # 如果不是对角线移动，直接返回True
        if abs(to_row - from_row) != 1 or abs(to_col - from_col) != 1:
            return True
        
        # 检查对角线移动的两条边
        edge1 = (from_row, to_col)  # 水平边
        edge2 = (to_row, from_col)  # 垂直边
        
        # 两条边都必须是安全的
        return self.is_valid_position(*edge1) and self.is_valid_position(*edge2)
    
    def get_neighbors(self, pos: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        获取有效邻居节点（智能8连通）
        在禁飞区附近限制对角线移动，避免"擦边"飞行
        """
        row, col = pos
        neighbors = []
        
        # 8个方向：上下左右+对角线
        directions = [
            (-1, -1), (-1, 0), (-1, 1),  # 上排
            (0, -1),           (0, 1),   # 中排（左右）
            (1, -1),  (1, 0),  (1, 1)    # 下排
        ]
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            
            # 检查目标位置是否有效
            if not self.is_valid_position(new_row, new_col):
                continue
            
            # 对角线移动需要额外检查（避免擦边飞行）
            if abs(dr) == 1 and abs(dc) == 1:  # 对角线移动
                # 检查对角线移动的两个相邻边是否安全
                # 例如从(1,1)到(2,2)，需要检查(1,2)和(2,1)都不是禁飞区
                edge1_safe = self.is_valid_position(row + dr, col)
                edge2_safe = self.is_valid_position(row, col + dc)
                
                # 只有两条边都安全时才允许对角线移动
                if not (edge1_safe and edge2_safe):
                    continue
            
            neighbors.append((new_row, new_col))
        
        return neighbors
    
    def manhattan_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> int:
        """计算曼哈顿距离"""
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])
    
    def euclidean_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """计算欧几里得距离"""
        return ((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2) ** 0.5
    
    def astar_path(self, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        A*算法寻找最短路径
        优化版本：禁飞区边缘安全移动，避免对角线"擦边"飞行
        """
        if not self.is_valid_position(*start) or not self.is_valid_position(*goal):
            rospy.logwarn(f"⚠️ Invalid start {start} or goal {goal}")
            return []
        
        # 优先队列：(f_score, g_score, position)
        open_set = [(0, 0, start)]
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.euclidean_distance(start, goal)}
        closed_set = set()
        
        while open_set:
            current_f, current_g, current = heapq.heappop(open_set)
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            if current == goal:
                # 重构路径
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]  # 反转得到正确顺序
            
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # 额外的安全检查：确保移动路径不会擦边穿过禁飞区
                if not self.is_safe_diagonal_move(current, neighbor):
                    continue
                
                # 计算移动代价（对角线移动代价更高）
                if abs(neighbor[0] - current[0]) + abs(neighbor[1] - current[1]) == 2:
                    move_cost = 1.414  # sqrt(2)
                else:
                    move_cost = 1.0
                
                tentative_g = g_score[current] + move_cost
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.euclidean_distance(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], tentative_g, neighbor))
        
        rospy.logwarn(f"⚠️ No path found from {start} to {goal}")
        return []
    
    def plan_coverage_path(self, start: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        规划覆盖路径：遍历所有可通行点并返回起点
        
        算法解释：
        1. 使用改进的最近邻贪心策略
        2. 每次选择距离当前位置最近的未访问点作为下一个目标
        3. 使用安全A*算法计算两点间的最优路径（禁飞区边缘安全移动）
        4. 去重处理：确保每个网格点只访问一次
        5. 最后使用A*算法规划返回起点的路径（同样避开禁飞区）
        
        安全移动策略：
        - 禁飞区边缘禁止对角线移动（避免擦边飞行）
        - 例如：A3B3->A4B4，如果A4B3是禁飞区，路径变为A3B3->A3B4->A4B4
        - 确保无人机不会意外进入禁飞区域
        
        点重叠原因解析：
        - A*路径可能经过多个中间点到达目标
        - 这些中间点也是有效的网格点，会被标记为已访问
        - 但在最终路径中仍会出现，这是正常的路径规划结果
        - 重叠确保了路径的连续性和避障的完整性
        
        注意：这是2D路径规划，只考虑X-Y平面的移动
        """
        rospy.loginfo("🗺️ Planning 2D coverage path...")
        rospy.loginfo("📖 Algorithm: Nearest Neighbor + A* with obstacle avoidance")
        
        # 获取所有可通行点
        all_valid_points = set()
        for row in range(1, self.rows + 1):
            for col in range(1, self.cols + 1):
                if self.is_valid_position(row, col):
                    all_valid_points.add((row, col))
        
        rospy.loginfo(f"📍 Total valid points in {self.rows}x{self.cols} grid: {len(all_valid_points)}")
        
        if start not in all_valid_points:
            rospy.logerr(f"❌ Start point {start} is not valid!")
            return []
        
        # 改进的贪心算法：最近邻策略 + 去重优化
        path = [start]
        unvisited = all_valid_points.copy()
        unvisited.remove(start)
        current_pos = start
        
        rospy.loginfo("🔍 Starting nearest neighbor coverage...")
        
        iteration = 0
        while unvisited:
            iteration += 1
            
            # 找到距离当前位置最近的未访问点
            nearest_point = min(unvisited, 
                               key=lambda p: self.euclidean_distance(current_pos, p))
            
            rospy.logdebug(f"🎯 Iteration {iteration}: Going from A{current_pos[0]}B{current_pos[1]} to A{nearest_point[0]}B{nearest_point[1]}")
            
            # 使用A*算法规划到最近点的路径（避开禁飞区）
            sub_path = self.astar_path(current_pos, nearest_point)
            
            if not sub_path:
                rospy.logwarn(f"⚠️ Cannot reach point {nearest_point}, skipping...")
                unvisited.remove(nearest_point)
                continue
            
            # 添加路径（排除重复的起点）
            path.extend(sub_path[1:])
            unvisited.remove(nearest_point)
            current_pos = nearest_point
            
            # 将A*路径上的所有点标记为已访问（去重处理）
            for point in sub_path[1:]:
                unvisited.discard(point)
            
            rospy.logdebug(f"📈 Progress: {len(all_valid_points) - len(unvisited)}/{len(all_valid_points)} points covered")
        
        # 返回起点（使用A*算法避开禁飞区）
        rospy.loginfo("🏠 Planning return to start point...")
        if current_pos != start:
            return_path = self.astar_path(current_pos, start)
            if return_path:
                path.extend(return_path[1:])  # 排除重复的当前位置
                rospy.loginfo("✅ Return path planned successfully")
            else:
                rospy.logwarn("⚠️ Cannot return to start point!")
        
        rospy.loginfo(f"✅ 2D coverage path completed with {len(path)} waypoints")
        rospy.loginfo(f"🔄 Path forms closed loop: {path[0] == path[-1] if len(path) > 1 else 'N/A'}")
        return path


class WaypointPlannerNode:
    """野生动物巡查航点规划节点"""
    
    def __init__(self):
        """初始化节点"""
        rospy.init_node('waypoint_planner_node', anonymous=True)
        rospy.loginfo("🛰️ Wildlife Patrol Waypoint Planner Node Started")
        
        # 加载配置
        self.load_config()
        
        # 初始化路径规划器
        self.planner = PathPlanner((9, 9))
        
        # 解析输入参数
        self.parse_mission_parameters()
        
        # 初始化ROS发布器
        self.init_publishers()
        
        # 规划路径
        self.plan_mission()
        
        rospy.loginfo("✅ Waypoint Planner initialized successfully")
    
    def load_config(self):
        """加载配置文件"""
        try:
            config_path = rospy.get_param('~config_file', 
                                        os.path.join(os.path.dirname(__file__), 
                                                   '../config/map.yaml'))
            with open(config_path, 'r', encoding='utf-8') as file:
                self.config = yaml.safe_load(file)
            rospy.loginfo(f"📋 Config loaded from: {config_path}")
            
        except Exception as e:
            rospy.logwarn(f"⚠️ Failed to load config: {e}, using defaults")
            self.config = self.get_default_config()
    
    def get_default_config(self):
        """获取默认配置"""
        return {
            'map': {
                'size': {'rows': 9, 'cols': 9},
                'cell_size': 1.0,
                'origin': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            },
            'mission': {
                'start_point': 'A1B1',
                'no_fly_zones': ['A3B3', 'A4B3', 'A5B3'],
                'altitude': 1.2,
                'algorithm': 'astar'
            }
        }
    
    def parse_mission_parameters(self):
        """解析任务参数"""
        # 起飞点
        start_point_str = rospy.get_param('~start_point', 
                                         self.config['mission']['start_point'])
        self.start_point = GridCoordinate.parse_coordinate(start_point_str)
        rospy.loginfo(f"🛫 Start point: {start_point_str} -> {self.start_point}")
        
        # 禁飞区
        no_fly_zones_param = rospy.get_param('~no_fly_zones', 
                                           self.config['mission']['no_fly_zones'])
        
        # 如果是字符串，按空格分割
        if isinstance(no_fly_zones_param, str):
            no_fly_zones_list = no_fly_zones_param.split()
        else:
            no_fly_zones_list = no_fly_zones_param
        
        self.no_fly_zones = set()
        for zone_str in no_fly_zones_list:
            try:
                zone_coord = GridCoordinate.parse_coordinate(zone_str)
                self.no_fly_zones.add(zone_coord)
            except Exception as e:
                rospy.logwarn(f"⚠️ Invalid no-fly zone coordinate: {zone_str}")
        
        rospy.loginfo(f"🚫 No-fly zones: {self.no_fly_zones}")
        
        # 飞行高度
        self.altitude = rospy.get_param('~altitude', 
                                       self.config['mission']['altitude'])
        
        # 设置规划器的禁飞区
        self.planner.set_obstacles(self.no_fly_zones)
    
    def init_publishers(self):
        """初始化ROS发布器"""
        # 航点序列发布器
        self.waypoints_pub = rospy.Publisher('/waypoints', PointArray, queue_size=1, latch=True)
        
        # 可视化发布器
        self.viz_pub = rospy.Publisher('/waypoint_viz', MarkerArray, queue_size=1, latch=True)
        
        rospy.loginfo("📡 Publishers initialized")
    
    def plan_mission(self):
        """规划巡查任务"""
        rospy.loginfo("🚁 Planning 2D patrol mission...")
        
        # 规划覆盖路径
        waypoint_coords = self.planner.plan_coverage_path(self.start_point)
        
        if not waypoint_coords:
            rospy.logerr("❌ Failed to plan mission path!")
            return
        
        # 转换为Point消息（所有航点保持相同的固定高度）
        waypoints = []
        for row, col in waypoint_coords:
            point = GridCoordinate.to_world_point(row, col, self.altitude)
            waypoints.append(point)
        
        # 发布航点序列
        self.publish_waypoints(waypoints)
        
        # 发布可视化
        self.publish_visualization(waypoint_coords, waypoints)
        
        rospy.loginfo(f"✅ 2D mission planned: {len(waypoints)} waypoints at altitude {self.altitude}m")
        rospy.loginfo(f"📍 Path: {' -> '.join([f'A{r}B{c}' for r, c in waypoint_coords[:5]])}{'...' if len(waypoint_coords) > 5 else ''}")
    
    def publish_waypoints(self, waypoints: List[Point]):
        """发布航点序列"""
        try:
            msg = PointArray()
            msg.header = Header()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "map"
            msg.points = waypoints
            
            self.waypoints_pub.publish(msg)
            rospy.loginfo(f"📤 Published {len(waypoints)} waypoints to /waypoints")
            
        except Exception as e:
            rospy.logerr(f"❌ Failed to publish waypoints: {e}")
    
    def publish_visualization(self, coords: List[Tuple[int, int]], points: List[Point]):
        """发布RViz可视化标记"""
        try:
            marker_array = MarkerArray()
            
            # 清除之前的标记
            clear_marker = Marker()
            clear_marker.action = Marker.DELETEALL
            marker_array.markers.append(clear_marker)
            
            # 路径线条
            path_marker = Marker()
            path_marker.header.frame_id = "map"
            path_marker.header.stamp = rospy.Time.now()
            path_marker.ns = "waypoint_path"
            path_marker.id = 0
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD
            path_marker.pose.orientation.w = 1.0
            path_marker.scale.x = 0.1  # 线宽
            path_marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # 绿色
            path_marker.points = points
            marker_array.markers.append(path_marker)
            
            # 起点标记
            start_marker = Marker()
            start_marker.header.frame_id = "map"
            start_marker.header.stamp = rospy.Time.now()
            start_marker.ns = "start_point"
            start_marker.id = 1
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position = points[0]
            start_marker.pose.orientation.w = 1.0
            start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.3
            start_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  # 蓝色
            marker_array.markers.append(start_marker)
            
            # 禁飞区标记
            for i, (row, col) in enumerate(self.no_fly_zones):
                obstacle_marker = Marker()
                obstacle_marker.header.frame_id = "map"
                obstacle_marker.header.stamp = rospy.Time.now()
                obstacle_marker.ns = "no_fly_zones"
                obstacle_marker.id = i + 2
                obstacle_marker.type = Marker.CUBE
                obstacle_marker.action = Marker.ADD
                obstacle_marker.pose.position = GridCoordinate.to_world_point(row, col, self.altitude)
                obstacle_marker.pose.orientation.w = 1.0
                obstacle_marker.scale.x = obstacle_marker.scale.y = obstacle_marker.scale.z = 0.5
                obstacle_marker.color = ColorRGBA(1.0, 0.0, 0.0, 0.7)  # 红色半透明
                marker_array.markers.append(obstacle_marker)
            
            # 航点编号标记
            for i, (point, (row, col)) in enumerate(zip(points[::5], coords[::5])):  # 每5个点显示一个编号
                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = "waypoint_numbers"
                text_marker.id = i + 100
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                # 创建一个独立的位置，不影响原始航点高度
                text_position = Point()
                text_position.x = point.x
                text_position.y = point.y
                text_position.z = point.z + 0.5  # 只有显示文字抬高，不影响实际航点
                text_marker.pose.position = text_position
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.3  # 文字大小
                text_marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)  # 白色
                text_marker.text = f"{i*5+1}"
                marker_array.markers.append(text_marker)
            
            self.viz_pub.publish(marker_array)
            rospy.loginfo("🎨 Published visualization markers to /waypoint_viz")
            
        except Exception as e:
            rospy.logerr(f"❌ Failed to publish visualization: {e}")
    
    def run(self):
        """主运行循环"""
        rospy.loginfo("🚀 Waypoint Planner running...")
        rospy.loginfo("💡 Use 'rostopic echo /waypoints' to see waypoint sequence")
        rospy.loginfo("🎨 Open RViz and add MarkerArray display for /waypoint_viz to visualize")
        
        try:
            rospy.spin()
        except KeyboardInterrupt:
            rospy.loginfo("🛑 Waypoint planner interrupted by user")


if __name__ == "__main__":
    try:
        planner = WaypointPlannerNode()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 Waypoint planner node interrupted")
    except Exception as e:
        rospy.logerr(f"❌ Waypoint planner node failed: {e}")
