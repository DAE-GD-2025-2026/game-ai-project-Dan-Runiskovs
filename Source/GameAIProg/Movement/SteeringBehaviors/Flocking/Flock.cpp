#include "Flock.h"
#include "FlockingSteeringBehaviors.h"
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
	Neighbors.SetNum(FlockSize);
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
	for (int AgentIdx{ 0 }; AgentIdx < Agents.Num(); ++AgentIdx)
	{
		auto* pAgent{ Agents[AgentIdx] };
		
		// --- Guard ---
		if (!IsValid(pAgent)) continue;
		
		// --- Register neighbors for current agent ---
		RegisterNeighbors(pAgent);
		
		// --- Go-To update ---
		pAgent->Tick(DeltaTime);
	}
}

void Flock::RenderDebug()
{
 // TODO: Render all the agents in the flock
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
				float tmp = *wSeparation;
				if (ImGui::SliderFloat("Separation", &tmp, 0.f, 1.f, "%.2f"))
					*wSeparation = tmp;
			}
			
			if (float* wCohesion  = pBlendedSteering->GetWeight(pCohesionBehavior.get()))
			{
				float tmp = *wCohesion;
				if (ImGui::SliderFloat("Cohesion", &tmp, 0.f, 1.f, "%.2f"))
					*wCohesion = tmp;
			}
			
			if (float* wAlignment  = pBlendedSteering->GetWeight(pVelMatchBehavior.get()))
			{
				float tmp = *wAlignment;
				if (ImGui::SliderFloat("Alignment", &tmp, 0.f, 1.f, "%.2f"))
					*wAlignment = tmp;
			}
			
			if (float* wWander  = pBlendedSteering->GetWeight(pWanderBehavior.get()))
			{
				float tmp = *wWander;
				if (ImGui::SliderFloat("Wander", &tmp, 0.f, 1.f, "%.2f"))
					*wWander = tmp;
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
void Flock::RegisterNeighbors(ASteeringAgent* const pAgent)
{
	// --- Reset ---
	NrOfNeighbors = 0;
	
	// --- Ref Position
	const auto& agentPos { pAgent->GetPosition()};
	
	// --- Compute Radius Squared ---
	const float rSquared = NeighborhoodRadius * NeighborhoodRadius;
	
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
	for (auto AgentIdx{0}; AgentIdx < NrOfNeighbors; ++AgentIdx)
		AveragePosition += Neighbors[AgentIdx]->GetPosition();
	
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
	for (auto AgentIdx{0}; AgentIdx < NrOfNeighbors; ++AgentIdx )
		AverageVelocity = AverageVelocity + Neighbors[AgentIdx]->GetLinearVelocity();
	
	// --- Divide and return ---
	return AverageVelocity / NrOfNeighbors;
}
