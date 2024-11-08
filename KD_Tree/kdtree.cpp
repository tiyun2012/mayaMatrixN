#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <algorithm>

struct Point {
    double x, y, z;
    int index;

    // Calculate distance to another point
    double distance(const Point& other) const {
        return std::sqrt((x - other.x) * (x - other.x) +
            (y - other.y) * (y - other.y) +
            (z - other.z) * (z - other.z));
    }
};

struct KDNode {
    Point point;
    KDNode* left;
    KDNode* right;

    KDNode(const Point& p) : point(p), left(nullptr), right(nullptr) {}
};

// Custom comparator for the max-heap
struct Compare {
    bool operator()(const std::pair<double, int>& a, const std::pair<double, int>& b) {
        return a.first < b.first; // Max-heap: larger distance at the top
    }
};

class KDTree {
public:
    KDTree(std::vector<Point>& points) {
        root = buildTree(points.begin(), points.end(), 0);
    }

    std::vector<int> findKNearest(const Point& target, int k) {
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, Compare> maxHeap;
        findKNearest(root, target, k, 0, maxHeap);

        std::vector<int> indices;
        while (!maxHeap.empty()) {
            indices.push_back(maxHeap.top().second);
            maxHeap.pop();
        }
        std::reverse(indices.begin(), indices.end());
        return indices;
    }

private:
    KDNode* root;

    // Build the KD-Tree using iterators and nth_element for better performance
    KDNode* buildTree(std::vector<Point>::iterator start, std::vector<Point>::iterator end, int depth) {
        if (start >= end) return nullptr;

        int axis = depth % 3;
        auto comparator = [axis](const Point& a, const Point& b) {
            return (axis == 0) ? (a.x < b.x) : (axis == 1) ? (a.y < b.y) : (a.z < b.z);
            };

        auto medianIt = start + (end - start) / 2;
        std::nth_element(start, medianIt, end, comparator);

        KDNode* node = new KDNode(*medianIt);
        node->left = buildTree(start, medianIt, depth + 1);
        node->right = buildTree(medianIt + 1, end, depth + 1);

        return node;
    }

    // Optimized k-nearest neighbor search
    void findKNearest(KDNode* node, const Point& target, int k, int depth,
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, Compare>& maxHeap) {
        if (!node) return;

        double dist = target.distance(node->point);
        if (maxHeap.size() < k) {
            maxHeap.emplace(dist, node->point.index);
        }
        else if (dist < maxHeap.top().first) {
            maxHeap.pop();
            maxHeap.emplace(dist, node->point.index);
        }

        int axis = depth % 3;
        double diff = (axis == 0 ? target.x - node->point.x :
            (axis == 1 ? target.y - node->point.y :
                target.z - node->point.z));

        KDNode* first = (diff < 0) ? node->left : node->right;
        KDNode* second = (diff < 0) ? node->right : node->left;

        findKNearest(first, target, k, depth + 1, maxHeap);

        if (maxHeap.size() < k || std::abs(diff) < maxHeap.top().first) {
            findKNearest(second, target, k, depth + 1, maxHeap);
        }
    }
};

// Example usage
int main() {
    std::vector<Point> points = {
        {3.0, 6.0, 7.0, 0}, {17.0, 15.0, 13.0, 1}, {13.0, 15.0, 6.0, 2},
        {6.0, 12.0, 10.0, 3}, {9.0, 1.0, 2.0, 4}, {2.0, 7.0, 3.0, 5},
        {10.0, 19.0, 12.0, 6}
    };

    KDTree tree(points);

    Point target = { 9.0, 2.0, 7.0, -1 };
    int k = 3;
    std::vector<int> indices = tree.findKNearest(target, k);

    std::cout << "Indices of the " << k << " nearest neighbors:\n";
    for (int index : indices) {
        std::cout << index << " ";
    }
    std::cout << std::endl;

    return 0;
}
