#pragma once
#include <vector>

#include "NavGraphPathfinding.h"
#include "VectorTypes.h"
#include "Movement/Pathfinding/Navmesh/TriPolygon.h"
#include "Shared/Graph/Graph.h"
#include "Shared/Graph/NavGraph/NavGraphNode.h"

namespace GameAI
{
	class SSFA final
{
public:
	//=== SSFA Functions ===
	//--- References ---
	//http://digestingduck.blogspot.be/2010/03/simple-stupid-funnel-algorithm.html
	//https://gamedev.stackexchange.com/questions/68302/how-does-the-simple-stupid-funnel-algorithm-work
	static std::vector<NavLine> FindPortals(std::vector<Node*> const & Path, TriPolygon const & NavPoly)
	{
		// --- Container ---
		std::vector<NavLine> Portals{};

		// --- Guard ---
		if (Path.empty()) return Portals;

		// --- Start ---
		NavLine StartPortal;
		StartPortal.P1 = Path.front()->GetPosition();
		StartPortal.P2 = Path.front()->GetPosition();
		Portals.emplace_back(StartPortal);

		// --- Start from 1, because start portal added ---
		for (size_t NodeIdx {1}; NodeIdx < Path.size(); ++NodeIdx)
		{
			const auto Node = Path[NodeIdx];
			const int EdgeIndex = static_cast<NavGraphNode*>(Node)->GetEdgeIdx();
			if (EdgeIndex == -1) continue;

			auto Edge = NavPoly.GetEdges()[EdgeIndex];

			FVector2D a{ Edge.GetP1(NavPoly)};
			FVector2D b{ Edge.GetP2(NavPoly)};

			// --- Path Direction --- 
			FVector2D Direction{ 
				(NodeIdx < Path.size() - 1) ? 
					(Path[NodeIdx + 1]->GetPosition() - Node->GetPosition()).GetSafeNormal()
					: FVector2D(1, 0)
			};
			
			const FVector2D EdgeVector{ b - a };
			const float CrossResult {Cross(EdgeVector, Direction)};
			
			NavLine Portal{};
			Portal.P1 = a; // Right
			Portal.P2 = b; // Left
			// --- Negative cross ? Swap ---
			if (CrossResult < 0) std::swap(Portal.P1, Portal.P2);
			
			Portals.emplace_back(Portal);
		}

		// --- End ---
		NavLine EndPortal;
		EndPortal.P1 = Path.back()->GetPosition();
		EndPortal.P2 = Path.back()->GetPosition();
		Portals.emplace_back(EndPortal);

		return Portals;
	}

	static std::vector<FVector2D> OptimizePortals( std::vector<NavLine> const & Portals, TriPolygon const & NavPoly)
	{
		// --- Container --- 
		std::vector<FVector2D> Path{};

		// --- Guard ---
		if (Portals.empty()) return Path;

		FVector2D Apex = Portals[0].P1;
		FVector2D LeftLeg  = Portals[0].P2 - Apex;
		FVector2D RightLeg = Portals[0].P1 - Apex;
		int ApexIndex = 0;
		int LeftLegIndex = 0;
		int RightLegIndex = 0;
		Path.push_back(Apex);

	    for (size_t PortalIdx { 1 }; PortalIdx < Portals.size(); ++PortalIdx)
	    {
	        const auto& [P1, P2] = Portals[PortalIdx];

	        // --- Right Check ---
	        const FVector2D Right = P1 - Apex;
	        if ( const float CrossRight{Cross(RightLeg, Right)}; CrossRight <= 0)
	        {
	            if (const float CrossLeft{Cross(LeftLeg, Right)}; CrossLeft < 0)
	            {
		            // --- Snap to left ---
	            	Apex = Apex + LeftLeg;
	            	Path.push_back(Apex);

	            	ApexIndex = LeftLegIndex;
	            	PortalIdx = ApexIndex + 1;

	            	LeftLegIndex = ApexIndex;
	            	RightLegIndex = ApexIndex;

	            	RightLeg = Portals[ApexIndex].P1 - Apex;
	            	LeftLeg  = Portals[ApexIndex].P2 - Apex;

	            	continue;
	            }
	            else
	            {
	            	// --- Tighten Right ---
	            	RightLeg = Right;
	            	RightLegIndex = PortalIdx;
	            }
	        }

	        // --- Left Check ---
	        const FVector2D Left{ P2 - Apex };
	        if (const float CrossLeft{ Cross (LeftLeg, Left) }; CrossLeft >= 0)
	        {
		        if (float CrossRight{Cross(RightLeg, Left)}; CrossRight > 0)
	            {
	                // --- Snap to right ---
	                Apex = Apex + RightLeg;
	                Path.push_back(Apex);

	                ApexIndex = RightLegIndex;
	                PortalIdx = ApexIndex + 1;

	                LeftLegIndex = ApexIndex;
	                RightLegIndex = ApexIndex;

	                RightLeg = Portals[RightLegIndex].P1 - Apex;
	                LeftLeg  = Portals[LeftLegIndex].P2 - Apex;

	                continue;
	            }
	            else
	            {
	            	// --- Tighten left ---
	                LeftLeg = Left;
	                LeftLegIndex = PortalIdx;
	            }
	        }
	    }

	    Path.push_back(Portals.back().P1);

	    return Path;
	}
	private:
		static float Cross(const FVector2D& a, const FVector2D& b)
		{
			return a.X * b.Y - a.Y * b.X;
		}
		SSFA() {};
		~SSFA() {};
	};
}
