#pragma once

#include <Eigen/Dense>

#include "BaseGraph.h"

namespace GraphCPP
{

    template <typename NodeType>
    class UndirectedGraph : public BaseGraph<NodeType, int>
    {
    public:
        void addNode(const NodeType &vertex) override
        {
            adjacencyList[vertex] = std::unordered_map<NodeType, int>();
        }

        void addEdge(const NodeType &from, const NodeType &to, const int &weight) override
        {
            adjacencyList[from][to] = weight;
            adjacencyList[to][from] = weight;
        }

        void removeNode(const NodeType &node) override
        {
            adjacencyList.erase(node);
            for (auto &pair : adjacencyList)
            {
                pair.second.erase(node);
            }
        }

        void removeEdge(const NodeType &from, const NodeType &to) override
        {
            adjacencyList[from].erase(to);
            adjacencyList[to].erase(from);
        }

        float getFiedlerValue()
        {
            // Construct Adjacency Matrix
            auto adjacencyMatrix = getAdjacencyMatrix();

            // Construct Degree Matrix
            auto degreeMatrix = getDegreeMatrix();

            // Construct Laplacian Matrix
            Eigen::MatrixXf laplacianMatrix = degreeMatrix - adjacencyMatrix;

            // Compute Eigenvalues
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(laplacianMatrix);
            if (eigensolver.info() != Eigen::Success)
                abort();
            Eigen::VectorXf eigenvalues = eigensolver.eigenvalues();

            // Sort Eigenvalues
            std::vector<std::pair<float, int>> eigenvalueIndexPairs;
            for (int i = 0; i < eigenvalues.size(); i++)
                eigenvalueIndexPairs.push_back({eigenvalues(i), i});

            // Return the second smallest eigenvalue
            std::sort(eigenvalueIndexPairs.begin(), eigenvalueIndexPairs.end());
            return eigenvalueIndexPairs[1].first;
        }

        float getSpectralGap()
        {
            auto adjacencyMatrix = getAdjacencyMatrix();

            // Compute Eigenvalues
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eigensolver(adjacencyMatrix);
            if (eigensolver.info() != Eigen::Success)
                abort();
            Eigen::VectorXf eigenvalues = eigensolver.eigenvalues();

            // Sort Eigenvalues
            std::vector<std::pair<float, int>> eigenvalueIndexPairs;
            for (int i = 0; i < eigenvalues.size(); i++)
                eigenvalueIndexPairs.push_back({eigenvalues(i), i});
            std::sort(eigenvalueIndexPairs.begin(), eigenvalueIndexPairs.end());

            // Return the difference between the largest and second smallest eigenvalue
            return eigenvalueIndexPairs[eigenvalueIndexPairs.size() - 1].first - eigenvalueIndexPairs[1].first;
        }

    private:
        Eigen::MatrixXf getAdjacencyMatrix()
        {
            Eigen::MatrixXf adjacencyMatrix(adjacencyList.size(), adjacencyList.size());
            adjacencyMatrix.setZero();
            std::unordered_map<NodeType, int> nodeToIndex;
            std::unordered_map<int, NodeType> indexToNode;

            int index = 0;
            for (auto &pair : adjacencyList)
            {
                nodeToIndex[pair.first] = index;
                indexToNode[index] = pair.first;
                index++;
            }

            for (auto &pair : adjacencyList)
            {
                NodeType node = pair.first;
                int i = nodeToIndex[node];
                for (auto &neighbor : pair.second)
                {
                    NodeType neighborNode = neighbor.first;
                    int j = nodeToIndex[neighborNode];
                    adjacencyMatrix(i, j) = neighbor.second;
                }
            }

            return adjacencyMatrix;
        }

        Eigen::MatrixXf getDegreeMatrix()
        {
            Eigen::MatrixXf degreeMatrix(adjacencyList.size(), adjacencyList.size());
            degreeMatrix.setZero();
            std::unordered_map<NodeType, int> nodeToIndex;
            std::unordered_map<int, NodeType> indexToNode;

            int index = 0;
            for (auto &pair : adjacencyList)
            {
                nodeToIndex[pair.first] = index;
                indexToNode[index] = pair.first;
                index++;
            }

            for (auto &pair : adjacencyList)
            {
                NodeType node = pair.first;
                int i = nodeToIndex[node];
                int degree = 0;
                for (auto &neighbor : pair.second)
                {
                    degree += neighbor.second;
                }
                degreeMatrix(i, i) = degree;
            }

            return degreeMatrix;
        }
    };

}