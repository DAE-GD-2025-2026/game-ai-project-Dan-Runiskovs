#include "NavGraph.h"

#include "NavGraphNode.h"

GameAI::NavGraph::NavGraph(std::unique_ptr<TriPolygon> && NavPoly)
	: Graph{false}
	, pNavPoly{std::move(NavPoly)}
{
	CreateNavigationGraph();
}

GameAI::NavGraph::NavGraph(const NavGraph& Other)
	: Graph(false)
{
	Nodes.reserve(Other.Nodes.size());
	for (std::unique_ptr<Node> const & OtherNode : Other.Nodes)
	{
		Nodes.push_back(std::make_unique<NavGraphNode>(*dynamic_cast<NavGraphNode*>(OtherNode.get())));
	}
        
	Connections.reserve(Other.Connections.size());
	for (std::unique_ptr<Connection> const & OtherConnection : Other.Connections)
	{
		Connections.push_back(std::make_unique<Connection>(*OtherConnection.get()));
	}
}

std::unique_ptr<GameAI::NavGraph> GameAI::NavGraph::Clone() const
{
	return std::make_unique<NavGraph>(*this);
}

int GameAI::NavGraph::GetNodeIdFromEdgeIndex(int EdgeIdx) const
{
	if (EdgeIdx >= 0)
	{
		for (auto const & pNode : Nodes)
		{
			if (reinterpret_cast<NavGraphNode*>(pNode.get())->GetEdgeIdx() == EdgeIdx)
			{
				return pNode->GetId();
			}
		}
	}
	
	return Graphs::InvalidNodeId;
}

void GameAI::NavGraph::CreateNavigationGraph()
{
	const auto& Edges = pNavPoly->GetEdges(); 
	const auto& Triangles = pNavPoly->GetTriangles();
	// --- Create Nodes ---
	for (int EdgeIdx = 0; EdgeIdx < Edges.size(); ++EdgeIdx) 
	{ 
		const auto& Edge = Edges[EdgeIdx]; 
		
		// --- Count how many triangles use this edge ---
		int TriangleCount{}; 
		for (const auto& Triangle : Triangles)
		{
			if (Triangle.HasEdge(Edge)) { TriangleCount++; }
		} 
		//  --- Only create node if edge is shared between TWO triangles ---
		if (TriangleCount == 2)
		{
			FVector p1 = Edge.GetP1(*pNavPoly); 
			FVector p2 = Edge.GetP2(*pNavPoly); 
			FVector2D midPoint{ (p1.X + p2.X) * 0.5f, (p1.Y + p2.Y) * 0.5f }; 
			auto node = std::make_unique<NavGraphNode>(midPoint, EdgeIdx); AddNode(std::move(node));
		} 
	} 
	
	// --- Create connections ---
	for (const auto& Triangle : Triangles) 
	{ 
		std::vector<int> NodeIds;
		
		// --- Find nodes belonging to this triangle ---
		for (const auto& Edge : Triangle.GetEdges())
		{
			auto EdgeIdxOpt = pNavPoly->FindEdgeIndex(Edge); 
			if (!EdgeIdxOpt.has_value()) continue; 
			int NodeId = GetNodeIdFromEdgeIndex(EdgeIdxOpt.value()); 
			if (NodeId == Graphs::InvalidNodeId)
			{
				UE_LOG(LogTemp, Warning, TEXT("No node for edge %d"), EdgeIdxOpt.value());
			} 
			if (NodeId != Graphs::InvalidNodeId)
			{
				NodeIds.push_back(NodeId);
			}
		} 
		// --- Connect all nodes inside this triangle ---
		for (int i{}; i < NodeIds.size(); ++i) 
		{ 
			for (int j{ i + 1}; j < NodeIds.size(); ++j) 
			{
				const auto* NodeA = GetNode(NodeIds[i]).get(); 
				const auto* NodeB = GetNode(NodeIds[j]).get(); 
				const float Weight = FVector2D::Distance( NodeA->GetPosition(), NodeB->GetPosition() ); 
				
				auto Connection0 = std::make_unique<Connection>(NodeIds[i], NodeIds[j]); 
				Connection0->SetWeight(Weight); 
				auto Connection1 = std::make_unique<Connection>(NodeIds[j], NodeIds[i]); 
				Connection1->SetWeight(Weight); 
				
				AddConnection(std::move(Connection0)); 
				AddConnection(std::move(Connection1)); // bi-directional 
			} 
		} 
	}
}
