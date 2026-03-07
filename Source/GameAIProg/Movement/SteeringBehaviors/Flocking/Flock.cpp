#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
#include "GeometryCollection/GeometryCollectionComponent.h"
#include "Shared/ImGuiHelpers.h"


Flock::Flock(
	UWorld* pWorld,
	TSubclassOf<ASteeringAgent> AgentClass,
	int FlockSize,
	float WorldSize,
	ASteeringAgent* const pAgentToEvade,
	bool bTrimWorld)
	: pWorld{pWorld}
	, FlockSize{ FlockSize }
	, pAgentToEvade{pAgentToEvade}
{
	// --- Set Up Arrays ---
	Agents.SetNum(FlockSize);
	
	// --- Set Up Neighbors ---
#ifndef GAMEAI_USE_SPACE_PARTITIONING
	Neighbors.SetNum(FlockSize);
#endif
	NrOfNeighbors = 0;
	
	// --- Set Up Behaviours ---
	pCohesionBehavior = std::make_unique<Cohesion>(this);
	pSeparationBehavior = std::make_unique<Separation>(this);
	pVelMatchBehavior = std::make_unique<VelocityMatch>(this);
	pSeekBehavior = std::make_unique<Seek>();
	pWanderBehavior = std::make_unique<Wander>();
	pBlendedSteering = std::make_unique<BlendedSteering>(std::vector<BlendedSteering::WeightedBehavior>{
	{ pSeparationBehavior.get(),  0.5f },  
	{ pCohesionBehavior.get(),    0.2f },  
	{ pVelMatchBehavior.get(),    0.2f },  
	{ pWanderBehavior.get(),      0.1f }  
	});
	
	// --- Agent to evade ---
	if (pAgentToEvade)
	{
		pEvadeBehavior = std::make_unique<Evade>();
		pAgentToEvade->SetSteeringBehavior(pSeekBehavior.get());
		pPrioritySteering = std::make_unique<PrioritySteering>(
			std::vector<ISteeringBehavior*>{ pEvadeBehavior.get(), pBlendedSteering.get() }
		);
	}
	
	// --- Set Up Flock ---
	for (int agentIdx{ 0 }; agentIdx < FlockSize; ++agentIdx)
	{
		const auto x{ static_cast<float>(FMath::FRandRange(-WorldSize*.5, WorldSize*.5))};
		const auto y{ static_cast<float>(FMath::FRandRange(-WorldSize*.5, WorldSize*.5))};

		FActorSpawnParameters SpawnParameters{};
		SpawnParameters.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;

		Agents[agentIdx] = pWorld->SpawnActor<ASteeringAgent>(
			AgentClass, FVector{ x, y, 10.f },FRotator::ZeroRotator, SpawnParameters);
		
		if (IsValid(Agents[agentIdx]))
			Agents[agentIdx]->SetActorTickEnabled(false);
	}

#ifdef GAMEAI_USE_SPACE_PARTITIONING
	// --- Create Partitioned space
	pPartitionedSpace = std::make_unique<CellSpace>(
		pWorld,
		WorldSize*2,
		WorldSize*2,
		NrOfCellsX,
		NrOfCellsX,
		FlockSize
	);

	OldPositions.SetNum(FlockSize);
	
	// --- Add agents to partitioned space ---
	for (int AgentIdx{ 0 }; AgentIdx < Agents.Num(); ++AgentIdx)
	{
		if (!IsValid(Agents[AgentIdx])) continue;
		pPartitionedSpace->AddAgent(*Agents[AgentIdx]);
		OldPositions[AgentIdx] = Agents[AgentIdx]->GetPosition();
	}

#endif
	// --- Set Up Agent Behaviours ---
	for (auto* pAgent : Agents)
	{
		if (!IsValid(pAgent)) continue;

		if (pPrioritySteering) pAgent->SetSteeringBehavior(pPrioritySteering.get());
		else pAgent->SetSteeringBehavior(pBlendedSteering.get());
	}
}

Flock::~Flock()
{
}

void Flock::Tick(float DeltaTime)
{
	// --- Update Target To Evade ---
	if (pEvadeBehavior && IsValid(pAgentToEvade))
	{
		FTargetData TargetToEvade{};
		TargetToEvade.Position = pAgentToEvade->GetPosition();
		TargetToEvade.Orientation = pAgentToEvade->GetRotation();
		TargetToEvade.LinearVelocity = pAgentToEvade->GetLinearVelocity();
		TargetToEvade.AngularVelocity = pAgentToEvade->GetAngularVelocity();
		if (pPrioritySteering) pPrioritySteering->SetTarget(TargetToEvade);
	}
	
	// --- Per-Agent Update ---
	for (int AgentIdx{0}; AgentIdx < Agents.Num(); ++AgentIdx)
	{
		auto* pAgent{ Agents[AgentIdx] };

		if (!IsValid(pAgent)) continue;
#ifdef GAMEAI_USE_SPACE_PARTITIONING
		pPartitionedSpace->RegisterNeighbors(*pAgent, NeighborhoodRadius);
		NrOfNeighbors = pPartitionedSpace->GetNrOfNeighbors();
#else
		RegisterNeighbors(pAgent);
#endif
		pAgent->Tick(DeltaTime);
#ifdef GAMEAI_USE_SPACE_PARTITIONING
		pPartitionedSpace->UpdateAgentCell(
			*pAgent,
			OldPositions[AgentIdx]
		);
		OldPositions[AgentIdx] = pAgent->GetPosition();
#endif
	}
}

void Flock::RenderDebug()
{
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	if (DebugRenderPartitions && pPartitionedSpace)
	{
		pPartitionedSpace->RenderCells();
	}
#endif
}

void Flock::ImGuiRender(ImVec2 const& WindowPos, ImVec2 const& WindowSize)
{
#ifdef PLATFORM_WINDOWS
#pragma region UI
	//UI
	{
		//Setup
		bool bWindowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Gameplay Programming", &bWindowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

		//Elements
		ImGui::Text("CONTROLS");
		ImGui::Indent();
		ImGui::Text("LMB: place target");
		ImGui::Text("RMB: move cam.");
		ImGui::Text("Scrollwheel: zoom cam.");
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::Spacing();

		ImGui::Text("STATS");
		ImGui::Indent();
		ImGui::Text("%.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
		ImGui::Text("%.1f FPS", ImGui::GetIO().Framerate);
		ImGui::Unindent();

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();

		ImGui::Text("Flocking");
		ImGui::Spacing();

  // TODO: implement ImGUI checkboxes for debug rendering here

		ImGui::Text("Behavior Weights");
		ImGui::Spacing();

  // TODO: implement ImGUI sliders for steering behavior weights here
		if (pBlendedSteering)
		{			
			if (float* wSeparation  = pBlendedSteering->GetWeight(pSeparationBehavior.get()))
			{
				ImGui::SliderFloat("Separation", wSeparation, 0.f, 1.f, "%.2f");
			}
			
			if (float* wCohesion  = pBlendedSteering->GetWeight(pCohesionBehavior.get()))
			{
				ImGui::SliderFloat("Cohesion", wCohesion, 0.f, 1.f, "%.2f");
			}
			
			if (float* wAlignment  = pBlendedSteering->GetWeight(pVelMatchBehavior.get()))
			{
				ImGui::SliderFloat("Alignment", wAlignment, 0.f, 1.f, "%.2f");
			}
			
			if (float* wWander  = pBlendedSteering->GetWeight(pWanderBehavior.get()))
			{
				ImGui::SliderFloat("Wander", wWander, 0.f, 1.f, "%.2f");
			}
		}
		//End
		ImGui::End();
	}
#pragma endregion
#endif
}

void Flock::RenderNeighborhood()
{
	// --- Set Up ---
	const auto* first{ Agents[0] };
	if (!IsValid(first)) return;
	const FVector NeighborhoodPos(first->GetPosition(), 10.f);
	
	// --- Draw ---
	DrawDebugCircle( pWorld,NeighborhoodPos, NeighborhoodRadius,32, FColor::Black,
		false,-1.f,0,2.f,
		FVector(0, 1, 0),FVector(1, 0, 0),false);
}

#ifndef GAMEAI_USE_SPACE_PARTITIONING
void Flock::RegisterNeighbors(const ASteeringAgent* pAgent)
{
	// --- Reset ---
	NrOfNeighbors = 0;
	
	// --- Ref Position
	const auto& agentPos { pAgent->GetPosition()};
	
	// --- Compute Radius Squared ---
	const float rSquared{ NeighborhoodRadius * NeighborhoodRadius};
	
	// --- Per-Agent check ---
	for (ASteeringAgent* const pOther : Agents)
	{
		// --- Guards ---
		if (!IsValid(pOther) || pOther == pAgent) continue;

		// --- Compute Distance Squared --- 
		const FVector2D toOther = pOther->GetPosition() - agentPos;
		const float distanceSquared = toOther.SizeSquared();

		// --- Register if close ---
		if (distanceSquared <= rSquared)
		{
			if (NrOfNeighbors < Neighbors.Num())
			{
				Neighbors[NrOfNeighbors] = pOther;
				NrOfNeighbors++;
			}
		}
	}
}
#endif

FVector2D Flock::GetAverageNeighborPos() const
{
	// --- Reset ---
	FVector2D AveragePosition = FVector2D::ZeroVector;
	
	// --- Guard ---
	if (NrOfNeighbors == 0) return AveragePosition;
	
	// --- Per agent addition ---
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	for (auto AgentIdx{0}; AgentIdx < NrOfNeighbors; ++AgentIdx)
		AveragePosition += pPartitionedSpace->GetNeighbors()[AgentIdx]->GetPosition();
#else
	for (auto AgentIdx{0}; AgentIdx < NrOfNeighbors; ++AgentIdx)
		AveragePosition += Neighbors[AgentIdx]->GetPosition();
#endif
	// --- Divide and Return ---
	return AveragePosition / NrOfNeighbors;

}

FVector2D Flock::GetAverageNeighborVelocity() const
{
	// --- Reset ---
	FVector2D AverageVelocity = FVector2D::ZeroVector;
	
	// --- Guard ---
	if (NrOfNeighbors == 0) return AverageVelocity;
	
	// --- Sum-Up Velocity --- 
#ifdef GAMEAI_USE_SPACE_PARTITIONING
	for (auto AgentIdx{0}; AgentIdx < NrOfNeighbors; ++AgentIdx)
		AverageVelocity += pPartitionedSpace->GetNeighbors()[AgentIdx]->GetLinearVelocity();
#else
	for (auto AgentIdx{0}; AgentIdx < NrOfNeighbors; ++AgentIdx )
		AverageVelocity = AverageVelocity + Neighbors[AgentIdx]->GetLinearVelocity();
#endif
	// --- Divide and return ---
	return AverageVelocity / NrOfNeighbors;
}
