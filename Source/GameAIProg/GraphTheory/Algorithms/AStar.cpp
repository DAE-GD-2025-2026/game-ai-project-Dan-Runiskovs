#include "AStar.h"

using namespace GameAI;

AStar::AStar(Graph* const pGraph, HeuristicFunctions::Heuristic hFunction)
	: pGraph(pGraph)
	, HeuristicFunction(hFunction)
{
}

std::vector<Node*>AStar::FindPath(Node* const pStartNode, Node* const pGoalNode)
{
	std::vector<Node*> path{};

	if (!pGraph || !pStartNode || !pGoalNode)
	{
		return path;
	}

	std::vector<NodeRecord> openList{};
	std::vector<NodeRecord> closedList{};

	NodeRecord startRecord{};
	startRecord.pNode = pStartNode;
	startRecord.pConnection = nullptr;
	startRecord.costSoFar = 0.f;
	startRecord.estimatedTotalCost = GetHeuristicCost(pStartNode, pGoalNode);

	openList.push_back(startRecord);

	NodeRecord currentRecord{};
	bool bFoundGoal = false;

	while (!openList.empty())
	{
		auto currentIt = std::min_element(openList.begin(), openList.end());
		currentRecord = *currentIt;

		// Remove current now, before changing openList further
		openList.erase(currentIt);

		if (currentRecord.pNode == pGoalNode)
		{
			bFoundGoal = true;
			break;
		}

		auto connections = pGraph->FindConnectionsFrom(currentRecord.pNode->GetId());

		for (Connection* pConnection : connections)
		{
			if (!pConnection)
			{
				continue;
			}

			Node* pNextNode = pGraph->GetNode(pConnection->GetToId()).get();
			if (!pNextNode)
			{
				continue;
			}

			float newCostSoFar = currentRecord.costSoFar + pConnection->GetWeight();

			// Check closed list
			auto closedIt = std::find_if(closedList.begin(), closedList.end(),
				[&](const NodeRecord& record)
				{
					return record.pNode == pNextNode;
				});

			if (closedIt != closedList.end())
			{
				if (closedIt->costSoFar <= newCostSoFar)
				{
					continue;
				}

				closedList.erase(closedIt);
			}

			// Check open list
			auto openIt = std::find_if(openList.begin(), openList.end(),
				[&](const NodeRecord& record)
				{
					return record.pNode == pNextNode;
				});

			if (openIt != openList.end())
			{
				if (openIt->costSoFar <= newCostSoFar)
				{
					continue;
				}

				openList.erase(openIt);
			}

			NodeRecord nextRecord{};
			nextRecord.pNode = pNextNode;
			nextRecord.pConnection = pConnection;
			nextRecord.costSoFar = newCostSoFar;
			nextRecord.estimatedTotalCost = newCostSoFar + GetHeuristicCost(pNextNode, pGoalNode);

			openList.push_back(nextRecord);
		}

		closedList.push_back(currentRecord);
	}

	if (!bFoundGoal)
	{
		return path;
	}

	// Backtrack from goal to start
	while (currentRecord.pNode != pStartNode)
	{
		path.push_back(currentRecord.pNode);

		if (!currentRecord.pConnection)
		{
			path.clear();
			return path;
		}

		int fromId = currentRecord.pConnection->GetFromId();
		Node* pPreviousNode = pGraph->GetNode(fromId).get();

		auto previousIt = std::find_if(closedList.begin(), closedList.end(),
			[&](const NodeRecord& record)
			{
				return record.pNode == pPreviousNode;
			});

		if (previousIt == closedList.end())
		{
			path.clear();
			return path;
		}

		currentRecord = *previousIt;
	}

	path.push_back(pStartNode);
	std::reverse(path.begin(), path.end());

	return path;
}

float AStar::GetHeuristicCost(Node* const pStartNode, Node* const pEndNode) const
{
	FVector2D toDestination = pGraph->GetNode(pEndNode->GetId())->GetPosition() - pGraph->GetNode(pStartNode->GetId())->GetPosition();
	return HeuristicFunction(abs(toDestination.X), abs(toDestination.Y));
}