"""
GPS 웨이포인트 그래프 + 다익스트라 예제 노드.
(production 경로는 onemin_action/waypoint_graph_planner.py 참고)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray
import yaml
import math
import matplotlib.pyplot as plt
import threading


class GPSPathFinder(Node):
    


    def __init__(self):
        super().__init__('gps_path_finder')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10)
        #self.subscription  # 미사용 변수 경고 방지
        self.goal_subscription = self.create_subscription(
            Float64MultiArray,
            '/goal/gps/pose',
            self.goal_callback,
            10)
        self.current_gps = None

        self.goal_gps = None

        self.last_goal_gps = None  # 목표 위치 변경 감지용

        #self.node_coordinates() # = self.load_coordinates()
        self.load_coordinates()
        self.obstacle_gps = (38.161860356310804, -122.4552556394544)
        self.special_graph = {i: {} for i in range(1, len(self.node_coordinates) + 1)}
        self.setup_graph()
        
    def gps_callback(self, msg):
        self.current_gps = (msg.latitude, msg.longitude)
        if self.goal_gps:
            self.calculate_path()

    # 가장 가까운 노드를 찾는 함수 (노드 앞의 'n' 제거)
    def find_closest_node(self, node_coordinates, gps_coord):
        min_distance = float('inf')
        closest_node = None
        for node, coord in node_coordinates.items():
            dist = math.sqrt((coord['latitude'] - gps_coord[0])**2 + (coord['longitude'] - gps_coord[1])**2)
            if dist < min_distance:
                min_distance = dist
                closest_node = node
        return closest_node  # 'n'을 제외하고 숫자만 반환

    def load_coordinates(self):
        file_path = './waypoints.yaml'
        with open(file_path, 'r') as file:
            self.node_coordinates = yaml.safe_load(file)

    def setup_graph(self):
        self.special_graph = {i: {} for i in range(1, len(self.node_coordinates) + 1)}
        for i in range(1, len(self.node_coordinates)):
            node1, node2 = i, i + 1
            self.special_graph[node1][node2] = self.distance(self.node_coordinates[node1], self.node_coordinates[node2])
            self.special_graph[node2][node1] = self.distance(self.node_coordinates[node1], self.node_coordinates[node2])
        special_nodes = [(1, 2), (2, 3)]
        for n1, n2 in special_nodes:
            self.special_graph[n1][n2] = self.distance(self.node_coordinates[n1], self.node_coordinates[n2])
            self.special_graph[n2][n1] = self.distance(self.node_coordinates[n1], self.node_coordinates[n2])

            # 장애물 노드를 찾고 그래프에서 제거 (노드 앞의 'n' 제거)
        excluded_node = self.find_closest_obstacle_node(self.node_coordinates, self.obstacle_gps)
        if excluded_node:
            self.remove_excluded_node_from_graph(self.special_graph, excluded_node)

    def distance(self, coord1, coord2):
        if isinstance(coord1, dict):
            lat1, lon1 = coord1['latitude'], coord1['longitude']
        else:  # coord1 is a tuple
            lat1, lon1 = coord1

        if isinstance(coord2, dict):
            lat2, lon2 = coord2['latitude'], coord2['longitude']
        else:  # coord2 is a tuple
            lat2, lon2 = coord2

        d_lat = math.radians(lat2 - lat1)
        d_lon = math.radians(lon2 - lon1)
        a = math.sin(d_lat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(d_lon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return 6371000.0 * c  # 지구 반지름 m (가중치 단위 통일)

    def goal_callback(self, msg):
        new_goal_gps = (msg.data[0], msg.data[1])
        if new_goal_gps != self.last_goal_gps:
            self.last_goal_gps = new_goal_gps
            self.goal_gps = new_goal_gps
            self.setup_graph()  # Reset and rebuild the graph with the new goal
            if self.current_gps:
                self.calculate_path()

    def calculate_path(self):
        # if not self.current_gps:
        #     return

        if not self.current_gps or not self.goal_gps:
            return
        
        start_node = 'robot start pose'
        #goal_gps = ( 38.16159365606373,  -122.45476833210664)
        #goal_gps = ( 38.16145829961485,  -122.45460657032564)
        goal_node = 'goal pose'
        self.closest_start_node, self.closest_goal_node = self.connect_temporary_nodes(self.special_graph, start_node, goal_node, self.current_gps, self.goal_gps, self.node_coordinates)  # 변경된 부분
        self.distances, previous_nodes = self.dijkstra_path(start_node)
        shortest_path = self.extract_shortest_path(previous_nodes, start_node, goal_node)
        #adjusted_path = self.adjust_path_to_goal(self.node_coordinates, shortest_path, goal_gps, self.current_gps)
        #adjusted_path = self.adjust_path_to_start(self.node_coordinates, adjusted_path, self.current_gps)  # 여기에서 호출
        adjusted_path = self.adjust_path_to_start_and_goal(self.node_coordinates, shortest_path, self.current_gps, self.goal_gps)
    
        self.get_logger().info(f"{start_node}에서 {goal_node}까지의 최단 경로: {adjusted_path}")
        
        
        self.save_path_to_yaml(adjusted_path, self.node_coordinates, 'path.yaml', self.goal_gps)
        # self.plot_graph(self.special_graph, self.node_coordinates, adjusted_path, self.current_gps, self.goal_gps)

        # Plot graph in a separate thread to avoid blocking main thread
        # plot_thread = threading.Thread(target=self.plot_graph, args=(self.special_graph, self.node_coordinates, adjusted_path, self.current_gps, self.goal_gps))
        # plot_thread.start()

    def save_path_to_yaml(self, path, node_coordinates, file_path, goal_gps):
        path_data = {}
        # enumerate를 시작 인덱스 1로 설정
        for index, node in enumerate(path, start=1):
            if isinstance(node, int):  # 숫자 ID 노드 처리
                path_data[index] = {
                    'latitude': node_coordinates[node]['latitude'],
                    'longitude': node_coordinates[node]['longitude']
                }
            else:  # 'robot start pose' 및 'goal pose' 문자열 처리
                if node == 'goal pose':
                    path_data[index] = {
                        'latitude': goal_gps[0],
                        'longitude': goal_gps[1]
                    }
                # 시작 위치 노드인 경우의 처리, 예를 들어 시작 GPS 좌표 저장
                elif node == 'robot start pose':
                    path_data[index] = {
                        'latitude': self.current_gps[0],  # 현재 GPS 위치 사용
                        'longitude': self.current_gps[1]
                    }

        with open(file_path, 'w') as file:
            yaml.dump(path_data, file, default_flow_style=False)

        self.get_logger().info(f"Path saved to {file_path}")
        
    # 가장 가까운 장애물 노드를 찾는 함수
    def find_closest_obstacle_node(self, node_coordinates, obstacle_gps, exclusion_radius=0.000027):
        min_obstacle_distance = exclusion_radius #float('inf')
        nearest_obstacle_node = None
        for node, coord in node_coordinates.items():
            obstacle_distance = math.sqrt((coord['latitude'] - obstacle_gps[0])**2 + (coord['longitude'] - obstacle_gps[1])**2)
            if obstacle_distance < min_obstacle_distance:
                min_obstacle_distance = obstacle_distance
                nearest_obstacle_node = node
        #print(f"Closest obstacle node: {nearest_obstacle_node} at distance {min_obstacle_distance}")  # 로그 추가
        return nearest_obstacle_node
    
    def remove_excluded_node_from_graph(self, graph, excluded_node):
        if excluded_node in graph:
            del graph[excluded_node]
        for neighbors in graph.values():
            neighbors.pop(excluded_node, None)

    

    def connect_temporary_nodes(self, graph, start_node, goal_node, start_gps, goal_gps, node_coordinates):
        # 시작 지점과 가장 가까운 노드를 찾습니다.
        closest_start_node = self.find_closest_node(node_coordinates, start_gps)
        # 도착 지점과 가장 가까운 노드를 찾습니다.
        closest_goal_node = self.find_closest_node(node_coordinates, goal_gps)
        
        # 시작 지점과 가장 가까운 노드 사이의 실제 거리를 계산합니다.
        start_distance = self.distance(node_coordinates[closest_start_node], start_gps)
        # 도착 지점과 가장 가까운 노드 사이의 실제 거리를 계산합니다.
        goal_distance = self.distance(node_coordinates[closest_goal_node], goal_gps)
        
        # 그래프에 시작 지점과 해당 노드 사이의 연결을 추가합니다.
        graph[start_node] = {closest_start_node: start_distance}
        # 그래프에 도착 지점과 해당 노드 사이의 연결을 추가합니다.
        graph[goal_node] = {closest_goal_node: goal_distance}
        
        # 역방향·양방향 간선 (기존 {start_distance} 는 set 이 되어 TypeError 났음 → 반드시 dict)
        if closest_start_node not in graph:
            graph[closest_start_node] = {}
        if closest_goal_node not in graph:
            graph[closest_goal_node] = {}

        graph[closest_start_node][start_node] = start_distance
        graph[start_node][closest_start_node] = start_distance
        graph[closest_goal_node][goal_node] = goal_distance
        graph[goal_node][closest_goal_node] = goal_distance
        
        return closest_start_node, closest_goal_node

    def dijkstra_path(self, start):
        dist = {node: float('inf') for node in self.special_graph}
        previous = {node: None for node in self.special_graph}
        dist[start] = 0
        unvisited_nodes = set(self.special_graph.keys())
        while unvisited_nodes:
            current = min(unvisited_nodes, key=lambda node: dist[node])
            unvisited_nodes.remove(current)
            for neighbor, weight in self.special_graph[current].items():
                if neighbor in unvisited_nodes:
                    new_dist = dist[current] + weight
                    if new_dist < dist[neighbor]:
                        dist[neighbor] = new_dist
                        previous[neighbor] = current
        return dist, previous

    def extract_shortest_path(self, previous, start, goal):
        path = []
        node = goal
        while node != start:
            path.insert(0, node)
            node = previous.get(node, None)
            if node is None:  # 경로를 찾을 수 없는 경우
                return []
        path.insert(0, start)
        return path

    def adjust_path_to_start_and_goal(self, node_coordinates, path, start_gps, goal_gps):
        # 경로에 충분한 노드가 있는지 확인
        self.get_logger().info(f'Current GPS received: {self.current_gps}')
        self.get_logger().info(f'Goal GPS received: {self.goal_gps}')
        if len(path) < 3:
            return path

        # 필요하다면 경로 시작을 조정
        if isinstance(path[1], int):
            first_node_num = path[1]
            second_node_num = path[2] if len(path) > 2 and isinstance(path[2], int) else None

            first_node_dist = self.distance(start_gps, node_coordinates[first_node_num])
            if second_node_num is not None:
                first_node_dist += self.distance(
                    node_coordinates[second_node_num], node_coordinates[first_node_num]
                )
            second_node_dist = (
                self.distance(start_gps, node_coordinates[second_node_num])
                if second_node_num is not None
                else float("inf")
            )

            if second_node_num is not None and second_node_dist < first_node_dist:
                path.pop(1)

        # 필요하다면 경로 끝을 조정
        if len(path) >= 3 and isinstance(path[-2], int):
            last_node_num = path[-2]
            second_last_node_num = path[-3] if isinstance(path[-3], int) else None

            last_node_dist = self.distance(goal_gps, node_coordinates[last_node_num])
            if second_last_node_num is not None:
                last_node_dist += self.distance(
                    node_coordinates[second_last_node_num], node_coordinates[last_node_num]
                )
            second_last_node_dist = (
                self.distance(goal_gps, node_coordinates[second_last_node_num])
                if second_last_node_num is not None
                else float("inf")
            )

            if second_last_node_num is not None and second_last_node_dist < last_node_dist:
                path.pop(-2)

        return path
    
    #shortest_path = extract_shortest_path(previous_nodes, start_node, goal_node)
    #adjusted_path = adjust_path_to_goal(node_coordinates, shortest_path, goal_robot_gps)

   # print("Shortest path from", start_node, "to", goal_node, ":", adjusted_path)

    def plot_graph(self, graph, node_coordinates, shortest_path, start_gps, goal_gps):
        plt.figure(figsize=(12, 10))

        # 모든 노드와 연결을 그립니다.
        for node, neighbors in graph.items():
            for neighbor, _ in neighbors.items():
                if node not in ['robot start pose', 'goal pose'] and neighbor not in ['robot start pose', 'goal pose']:  # 임시 노드 연결은 별도로 그립니다
                    plt.plot([node_coordinates[node]['longitude'], node_coordinates[neighbor]['longitude']],
                            [node_coordinates[node]['latitude'], node_coordinates[neighbor]['latitude']], 'k-')

        # 장애물 위치를 그립니다.
        plt.plot(self.obstacle_gps[1], self.obstacle_gps[0], 'rx', label='Obstacle', markersize=10)

        # 모든 노드를 그립니다.
        x, y = zip(*[(coord['longitude'], coord['latitude']) for coord in node_coordinates.values()])
        plt.scatter(x, y, c='red', label='Nodes', edgecolors='k')

        # 시작점을 그립니다.
        plt.scatter(start_gps[1], start_gps[0], c='blue', s=100, label='Start', edgecolors='k', zorder=5)

        # 도착점을 그립니다.
        plt.scatter(goal_gps[1], goal_gps[0], c='magenta', s=100, label='Goal', edgecolors='k', zorder=5)

        # 최단 경로를 그립니다.
        if shortest_path:
            path_x = [node_coordinates[node]['longitude'] if node not in ['robot start pose', 'goal pose'] else start_gps[1] if node == 'robot start pose' else goal_gps[1] for node in shortest_path]
            path_y = [node_coordinates[node]['latitude'] if node not in ['robot start pose', 'goal pose'] else start_gps[0] if node == 'robot start pose' else goal_gps[0] for node in shortest_path]
            plt.plot(path_x, path_y, 'g-', linewidth=3, label='Shortest Path')

        plt.xlabel('Longitude')
        plt.ylabel('Latitude')
        plt.title('Shortest Path with Obstacle')
        plt.legend()
        plt.grid(True)
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    gps_path_finder = GPSPathFinder()
    rclpy.spin(gps_path_finder)
    gps_path_finder.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()