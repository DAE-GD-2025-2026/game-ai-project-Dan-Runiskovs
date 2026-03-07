#include "SpacePartitioning.h"

// --- Cell ---
Cell::Cell(float Left, float Bottom, float Width, float Height)
{
	BoundingBox.Min = { Left, Bottom };
	BoundingBox.Max = { BoundingBox.Min.X + Width, BoundingBox.Min.Y + Height };
}

std::vector<FVector2D> Cell::GetRectPoints() const
{
	const float left = BoundingBox.Min.X;
	const float bottom = BoundingBox.Min.Y;
	const float width = BoundingBox.Max.X - BoundingBox.Min.X;
	const float height = BoundingBox.Max.Y - BoundingBox.Min.Y;

	std::vector<FVector2D> rectPoints =
	{
		{ left , bottom  },
		{ left , bottom + height  },
		{ left + width , bottom + height },
		{ left + width , bottom  },
	};

	return rectPoints;
}

// --- Partitioned Space ---
CellSpace::CellSpace(UWorld* pWorld, float Width, float Height, int Rows, int Cols, int MaxEntities)
	: pWorld{pWorld}
	, SpaceWidth{Width}
	, SpaceHeight{Height}
	, NrOfRows{Rows}
	, NrOfCols{Cols}
	, NrOfNeighbors{0}
{
	Neighbors.SetNum(MaxEntities);
	
	// --- Calculate Bounds ---
	CellOrigin = FVector2D(-Width * 0.5f, -Height * 0.5f);
	CellWidth = Width / Cols;
	CellHeight = Height / Rows;

	Cells.reserve(Rows * Cols);

	for (int row = 0; row < Rows; ++row)
	{
		for (int col = 0; col < Cols; ++col)
		{
			float left = CellOrigin.X + col * CellWidth;
			float bottom = CellOrigin.Y + row * CellHeight;

			Cells.emplace_back(left, bottom, CellWidth, CellHeight);
		}
	}
}

void CellSpace::AddAgent(ASteeringAgent& Agent)
{
	const int idx = PositionToIndex(Agent.GetPosition());
	
	if (std::find(Cells[idx].Agents.begin(), Cells[idx].Agents.end(), &Agent) == Cells[idx].Agents.end())
	{
		Cells[idx].Agents.push_back(&Agent);
	}
}

void CellSpace::UpdateAgentCell(ASteeringAgent& Agent, const FVector2D& OldPos)
{
	const int oldIdx = PositionToIndex(OldPos);
	const int newIdx = PositionToIndex(Agent.GetPosition());

	// If still in the same cell, nothing to do
	if (oldIdx == newIdx) return;

	// Remove from old cell
	Cells[oldIdx].Agents.remove(&Agent);

	// Add to new cell
	Cells[newIdx].Agents.push_back(&Agent);
}

void CellSpace::RegisterNeighbors(const ASteeringAgent& Agent, float QueryRadius)
{
	// --- Reset ---
	NrOfNeighbors = 0;

	const FVector2D agentPos = Agent.GetPosition();
	const float queryRadiusSq = QueryRadius * QueryRadius;

	// --- Build rectangle ---
	FRect queryRect;
	queryRect.Min = agentPos - FVector2D(QueryRadius, QueryRadius);
	queryRect.Max = agentPos + FVector2D(QueryRadius, QueryRadius);

	// Check overlapping cells
	for (Cell& cell : Cells)
	{
		if (!DoRectsOverlap(cell.BoundingBox, queryRect)) continue;

		// Check agents inside cell
		for (ASteeringAgent* other : cell.Agents)
		{
			if (&Agent == other) continue;

			const FVector2D toOther{ other->GetPosition() - agentPos };

			if (toOther.SizeSquared() <= queryRadiusSq)
			{
				if (NrOfNeighbors < Neighbors.Num())
				{
					Neighbors[NrOfNeighbors] = other;
					++NrOfNeighbors;
				}
				else return;
			}
		}
	}
}

void CellSpace::EmptyCells()
{
	for (Cell& c : Cells)
		c.Agents.clear();
}

void CellSpace::RenderCells() const
{
	if (!pWorld) return;

	constexpr float Z = 10.f;

	for (const Cell& cell : Cells)
	{
		// corners from bounding box
		FVector Bl(cell.BoundingBox.Min.X, cell.BoundingBox.Min.Y, Z); // bottom-left
		FVector Tl(cell.BoundingBox.Min.X, cell.BoundingBox.Max.Y, Z); // top-left
		FVector Tr(cell.BoundingBox.Max.X, cell.BoundingBox.Max.Y, Z); // top-right
		FVector Br(cell.BoundingBox.Max.X, cell.BoundingBox.Min.Y, Z); // bottom-right

		// draw box
		DrawDebugLine(pWorld, Bl, Tl, FColor::Red, false, 0.f, 0, 1.5f);
		DrawDebugLine(pWorld, Tl, Tr, FColor::Red, false, 0.f, 0, 1.5f);
		DrawDebugLine(pWorld, Tr, Br, FColor::Red, false, 0.f, 0, 1.5f);
		DrawDebugLine(pWorld, Br, Bl, FColor::Red, false, 0.f, 0, 1.5f);

		// draw number in the center
		FVector center(
			(cell.BoundingBox.Min + cell.BoundingBox.Max) * 0.5f,
			Z
		);

		DrawDebugString(
			pWorld,
			center,
			FString::FromInt(cell.Agents.size()),
			nullptr,
			FColor::White,
			0.f
		);
	}
}

int CellSpace::PositionToIndex(FVector2D const & Pos) const
{
	const float LocalX{ static_cast<float>(Pos.X - CellOrigin.X) };
	const float LocalY{ static_cast<float>(Pos.Y - CellOrigin.Y) };

	int Col{ static_cast<int>(LocalX / CellWidth)};
	int Row{ static_cast<int>(LocalY / CellHeight)};

	Col = FMath::Clamp(Col, 0, NrOfCols - 1);
	Row = FMath::Clamp(Row, 0, NrOfRows - 1);

	return Row * NrOfCols + Col;
}
bool CellSpace::DoRectsOverlap(FRect const & RectA, FRect const & RectB)
{
	// Check if the rectangles are separated on either axis
	if (RectA.Max.X < RectB.Min.X || RectA.Min.X > RectB.Max.X) return false;
	if (RectA.Max.Y < RectB.Min.Y || RectA.Min.Y > RectB.Max.Y) return false;
    
	// If they are not separated, they must overlap
	return true;
}