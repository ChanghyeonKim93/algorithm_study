#include <iostream>
#include <queue>
#include <unordered_map>
#include <vector>

#include "opencv4/opencv2/highgui.hpp"
#include "opencv4/opencv2/opencv.hpp"

#define MAX_INT 1000000000

struct Position {
  int x{0};
  int y{0};
};

struct Connection {
  int index{-1};
  double dist{0.0};
};

struct Node {
  Position position;
  std::vector<Connection> connection_list;
};

std::unordered_map<int, Node> node_list;
std::vector<std::pair<int, int>> connection_list;

inline bool IsExistingNode(const int index) {
  return (node_list.find(index) != node_list.end());
}

void AddNode(const int index, const int x, const int y) {
  if (IsExistingNode(index)) throw std::runtime_error("duplicated insert!");
  node_list.insert({index, Node{{x, y}, {}}});
}

void ConnectNodes(const int start_index, const int end_index) {
  if (!IsExistingNode(start_index) || !IsExistingNode(end_index))
    throw std::runtime_error("not existing nodes!");

  auto& start_node = node_list.at(start_index);
  auto& end_node = node_list.at(end_index);

  const int dx = start_node.position.x - end_node.position.x;
  const int dy = start_node.position.y - end_node.position.y;
  const double dist = std::sqrt(dx * dx + dy * dy);
  start_node.connection_list.push_back({end_index, dist});
  end_node.connection_list.push_back({start_index, dist});
  connection_list.push_back({start_index, end_index});
}

int FindNearestNode(const int start_index) {
  const int num_nodes = static_cast<int>(node_list.size());
  std::vector<bool> visited(num_nodes, false);
  std::vector<int> prev_index(num_nodes, -1);

  std::vector<double> min_dist_list(num_nodes, MAX_INT);
  min_dist_list[start_index] = 0;
  struct IndexAndDist {
    int index{0};
    double dist{0.0};
    bool operator<(const IndexAndDist& rhs) const { return dist > rhs.dist; }
  };

  std::priority_queue<IndexAndDist> pq;
  pq.push({start_index, 0});
  while (!pq.empty()) {
    const auto index_and_dist = pq.top();
    pq.pop();
    std::cerr << "I D : " << index_and_dist.index << " " << index_and_dist.dist
              << std::endl;
    const int current_index = index_and_dist.index;
    if (visited[current_index]) continue;
    visited[current_index] = true;

    const auto current_dist = index_and_dist.dist;
    const auto& current_node = node_list.at(current_index);
    for (const auto& connection : current_node.connection_list) {
      const auto index = connection.index;
      const double new_dist = current_dist + connection.dist;
      if (new_dist < min_dist_list[index]) {
        min_dist_list[index] = new_dist;
        prev_index[index] = current_index;
        pq.push({index, new_dist});
      }
    }
  }

  for (int index = 0; index < num_nodes; ++index) {
    if (index == start_index) continue;
    pq.push({index, min_dist_list[index]});
  }
  return pq.top().index;
}

std::vector<int> FindShortestPath(const int start_index, const int end_index) {
  const int num_nodes = static_cast<int>(node_list.size());
  std::vector<bool> visited(num_nodes, false);
  std::vector<int> prev_index(num_nodes, -1);

  std::vector<double> min_dist_list(num_nodes, MAX_INT);
  min_dist_list[start_index] = 0;
  struct IndexAndDist {
    int index{0};
    double dist{0};
    bool operator<(const IndexAndDist& rhs) const { return dist > rhs.dist; }
  };

  std::priority_queue<IndexAndDist> pq;
  pq.push({start_index, 0});
  while (!pq.empty()) {
    const auto index_and_dist = pq.top();
    pq.pop();
    const int current_index = index_and_dist.index;
    if (visited[current_index]) continue;
    visited[current_index] = true;

    const auto current_dist = index_and_dist.dist;

    const auto& current_node = node_list.at(current_index);
    for (const auto& connection : current_node.connection_list) {
      const auto index = connection.index;
      const double new_dist = current_dist + connection.dist;
      if (new_dist < min_dist_list[index]) {
        min_dist_list[index] = new_dist;
        prev_index[index] = current_index;
        pq.push({index, new_dist});
      }
    }
  }

  std::vector<int> shortest_path;
  int i = end_index;
  shortest_path.push_back(i);
  while (i != start_index) {
    shortest_path.push_back(prev_index[i]);
    i = prev_index[i];
  }

  return shortest_path;
}

int main(int argc, char** argv) {
  try {
    constexpr int kWidth = 640;
    constexpr int kHeight = 480;
    cv::Mat map_image = cv::Mat::zeros(kHeight, kWidth, CV_8UC3);

    // Add nodes
    AddNode(0, 20, 145);
    AddNode(1, 400, 45);
    ConnectNodes(0, 1);
    AddNode(2, 462, 131);
    AddNode(3, 55, 225);
    ConnectNodes(1, 2);
    ConnectNodes(1, 3);
    ConnectNodes(2, 3);
    AddNode(4, 563, 443);
    AddNode(5, 477, 378);
    ConnectNodes(2, 4);
    ConnectNodes(3, 4);
    ConnectNodes(2, 5);
    ConnectNodes(3, 5);
    ConnectNodes(4, 5);
    AddNode(6, 222, 354);
    AddNode(7, 111, 332);
    ConnectNodes(3, 6);
    ConnectNodes(4, 6);
    ConnectNodes(6, 7);

    // Visualize nodes
    const cv::Scalar kNodeColor(255, 0, 0);
    const cv::Scalar kConnectionColor(0, 0, 255);
    for (const auto& connection : connection_list) {
      const auto& n0 = node_list.at(connection.first);
      const auto& n1 = node_list.at(connection.second);
      cv::line(map_image, cv::Point2i(n0.position.x, n0.position.y),
               cv::Point2i(n1.position.x, n1.position.y), kConnectionColor, 1);
    }
    for (const auto& [index, node] : node_list) {
      cv::circle(map_image, cv::Point2i(node.position.x, node.position.y), 3,
                 kNodeColor, 2);
      cv::putText(map_image, std::to_string(index),
                  cv::Point2i(node.position.x + 4, node.position.y),
                  cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(255, 255, 255), 1);
    }

    cv::namedWindow("map_image");
    cv::imshow("map_image", map_image);
    cv::waitKey(0);

    const int q_index = 2;
    const int n_index = FindNearestNode(q_index);
    const cv::Scalar kNearestColor(0, 255, 255);
    const auto& n0 = node_list.at(q_index);
    const auto& n1 = node_list.at(n_index);
    cv::line(map_image, cv::Point2i(n0.position.x, n0.position.y),
             cv::Point2i(n1.position.x, n1.position.y), kNearestColor, 6);

    auto shortest_path = FindShortestPath(0, 4);
    std::cerr << "shortest_path.size:" << shortest_path.size() << std::endl;
    for (int i = 0; i < shortest_path.size() - 1; ++i) {
      const auto& n0 = node_list.at(shortest_path[i]);
      const auto& n1 = node_list.at(shortest_path[i + 1]);
      cv::line(map_image, cv::Point2i(n0.position.x, n0.position.y),
               cv::Point2i(n1.position.x, n1.position.y),
               cv::Scalar(255, 0, 255), 2);
    }

    cv::imshow("map_image", map_image);
    cv::waitKey(0);
  } catch (std::exception& e) {
    std::cerr << " ERROR : " << e.what() << std::endl;
  }

  return 0;
}