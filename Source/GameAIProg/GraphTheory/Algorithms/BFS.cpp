#include "BFS.h"

#include <map>
#include <queue>
#include <unordered_map>
#include <unordered_set>

#include "Shared/Graph/Graph.h"

using namespace GameAI;

BFS::BFS(Graph* const pGraph)
	: pGraph(pGraph)
{
}

// TODO Breath First Search Algorithm searches for a path from the startNode to the destinationNode
std::vector<Node*> BFS::FindPath(Node* const pStartNode, Node* const pDestinationNode) const
{
	std::vector<Node*> path;

	if (!pStartNode || !pDestinationNode)
		return path;

	std::queue<Node*> openQueue;
	std::unordered_map<Node*, Node*> cameFrom;
	std::unordered_set<Node*> visited;

	openQueue.push(pStartNode);
	visited.insert(pStartNode);

	while (!openQueue.empty())
	{
		Node* current = openQueue.front();
		openQueue.pop();

		if (current == pDestinationNode)
		{
			// reconstruct path
			Node* node = pDestinationNode;
			while (node)
			{
				path.push_back(node);
				node = cameFrom.count(node) ? cameFrom[node] : nullptr;
			}

			std::reverse(path.begin(), path.end());
			return path;
		}

		auto connections = pGraph->FindConnectionsFrom(current->GetId());

		for (Connection* conn : connections)
		{
			Node* neighbor = pGraph->GetNode(conn->GetToId()).get();

			if (visited.find(neighbor) != visited.end())
				continue;

			visited.insert(neighbor);
			cameFrom[neighbor] = current;
			openQueue.push(neighbor);
		}
	}

	return path;
}
