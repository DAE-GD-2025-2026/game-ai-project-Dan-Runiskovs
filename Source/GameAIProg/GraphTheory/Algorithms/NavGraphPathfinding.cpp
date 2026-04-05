#include "NavGraphPathfinding.h"

#include "AStar.h"
#include "PathSmoothing.h"
#include "VectorTypes.h"
#include "Shared/Graph/NavGraph/NavGraph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

using namespace GameAI;

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos,
	NavGraph* const pNavGraph, std::vector<FVector2D>& debugNodePositions, std::vector<NavLine>& debugPortals) 
{
	std::vector<FVector2D> FinalPath{};

    const auto* NavPoly{ pNavGraph->GetNavPolygon()};

    // --- Find Triangles --- 
    FVector2D StartPosCorrected{};
    const auto* StartTriangle{ NavPoly->GetClosestTriangleToPosition(startPos, StartPosCorrected)};

    FVector2D EndPosCorrected{};
    const auto* EndTriangle{ NavPoly->GetClosestTriangleToPosition(endPos, EndPosCorrected)};

    if (!StartTriangle || !EndTriangle) return FinalPath;

    // --- Direct path in same triangle --- 
    if (StartTriangle == EndTriangle) return { StartPosCorrected, EndPosCorrected };

    // --- Clone graph ---
    const auto GraphCloned {pNavGraph->Clone()};

    // --- Helper : Connect Node to Triangle ---
    auto connectNodeToTriangle = [&](int NodeId, const FVector2D& Pos, const TriPolygon::Triangle* Triangle)
    {
        for (const auto& NodePtr : GraphCloned->GetNodes())
        {
            const auto* NavNode{ static_cast<NavGraphNode*>(NodePtr.get()) };

            const int EdgeIdx{ NavNode->GetEdgeIdx() };
            if (EdgeIdx < 0) continue;

            // --- No such edge ? skip ---
            if (const auto& Edge{ NavPoly->GetEdges()[EdgeIdx]}; !Triangle->HasEdge(Edge)) continue;

            const float Weight = FVector2D::Distance(Pos, NavNode->GetPosition());

            auto Connection0 = std::make_unique<Connection>(NodeId, NavNode->GetId());
            Connection0->SetWeight(Weight);

            auto Connection1 = std::make_unique<Connection>(NavNode->GetId(), NodeId);
            Connection1->SetWeight(Weight);

            GraphCloned->AddConnection(std::move(Connection0));
            GraphCloned->AddConnection(std::move(Connection1)); // bi-directional
        }
    };

    // --- Start ---
    const int StartNodeId{ GraphCloned->AddNode(
        std::make_unique<NavGraphNode>(StartPosCorrected, -1)
    )};
    connectNodeToTriangle(StartNodeId, StartPosCorrected, StartTriangle);

    // --- End ---
    const int EndNodeId{ GraphCloned->AddNode(
        std::make_unique<NavGraphNode>(EndPosCorrected, -1)
    )};
    connectNodeToTriangle(EndNodeId, EndPosCorrected, EndTriangle);

    // --- Fallback ---
    if (StartNodeId == Graphs::InvalidNodeId || EndNodeId == Graphs::InvalidNodeId) 
        return FinalPath;
    
    Node* StartNode{ GraphCloned->GetNode(StartNodeId).get() };
    Node* DestinationNode{ GraphCloned->GetNode(EndNodeId).get() };

    if (!StartNode || !DestinationNode) return FinalPath;

    // --- A* ---
    AStar Astar{ GraphCloned.get(), HeuristicFunctions::Chebyshev };
    const auto PathNodes{ Astar.FindPath(StartNode, DestinationNode) };

    if (PathNodes.empty()) return FinalPath;

    // --- Debug ---
    debugNodePositions.clear();
    for (const auto* Node : PathNodes)
    {
        if (Node) debugNodePositions.push_back(Node->GetPosition());
        // we dont add portals here
    }

    // --- Portals ---
    debugPortals = SSFA::FindPortals(PathNodes, *NavPoly);
    FinalPath = SSFA::OptimizePortals(debugPortals, *NavPoly);

    return FinalPath;
}

std::vector<FVector2D> NavMeshPathfinding::FindPath(const FVector2D& startPos, const FVector2D& endPos, NavGraph* const pNavGraph)
{
	std::vector<FVector2D> debugNodePositions{};
	std::vector<NavLine> debugPortals{};

	return FindPath(startPos, endPos, pNavGraph, debugNodePositions, debugPortals);
}