#include "Level_CombinedSteering.h"
#include <format>
#include "imgui.h"

// Sets default values
ALevel_CombinedSteering::ALevel_CombinedSteering()
{
	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
}

// Called when the game starts or when spawned
void ALevel_CombinedSteering::BeginPlay()
{
	Super::BeginPlay();
	
	AddAgent(BehaviorTypes::BlendedSteering);
	AddAgent(BehaviorTypes::PrioritySteering);
	
	SteeringAgents[0].Agent->SetDebugRenderingEnabled(true);
	SteeringAgents[1].SelectedTarget = 0;
}

void ALevel_CombinedSteering::BeginDestroy()
{
	Super::BeginDestroy();
}

bool ALevel_CombinedSteering::AddAgent(BehaviorTypes BehaviorType, bool AutoOrient)
{
	ImGui_Agent ImGuiAgent = {};
	ImGuiAgent.Agent = GetWorld()->SpawnActor<ASteeringAgent>(SteeringAgentClass, FVector{0,0,90}, FRotator::ZeroRotator);
	if (IsValid(ImGuiAgent.Agent))
	{
		ImGuiAgent.SelectedBehavior = static_cast<int>(BehaviorType);
		ImGuiAgent.SelectedTarget = -1; // Mouse
		
		SetAgentBehavior(ImGuiAgent);

		SteeringAgents.push_back(std::move(ImGuiAgent));
		
		RefreshTargetLabels();

		return true;
	}

	return false;
}

void ALevel_CombinedSteering::RemoveAgent(unsigned int Index)
{
	SteeringAgents[Index].Agent->Destroy();
	SteeringAgents.erase(SteeringAgents.begin() + Index);

	RefreshTargetLabels();
	RefreshAgentTargets(Index);
}

void ALevel_CombinedSteering::SetAgentBehavior(ImGui_Agent& Agent)
{
	Agent.Behavior.reset();
	
	switch (static_cast<BehaviorTypes>(Agent.SelectedBehavior))
	{
	case BehaviorTypes::BlendedSteering:
		{
			// Create per-agent sub behaviors (owned by the level to avoid leaks / dangling).
			auto SeekPtr = std::make_unique<Seek>();
			auto WanderPtr = std::make_unique<Wander>();
			ISteeringBehavior* const pSeek = SeekPtr.get();
			ISteeringBehavior* const pWander = WanderPtr.get();
			OwnedBehaviors.emplace_back(std::move(SeekPtr));
			OwnedBehaviors.emplace_back(std::move(WanderPtr));

			std::vector<BlendedSteering::WeightedBehavior> DrunkBehaviours;
			DrunkBehaviours.emplace_back(pSeek, 0.5f);
			DrunkBehaviours.emplace_back(pWander, 0.5f);

			Agent.Behavior = std::make_unique<BlendedSteering>(DrunkBehaviours);
			break;
		}
	case BehaviorTypes::PrioritySteering:
		{
			auto EvadePtr = std::make_unique<Evade>();
			auto WanderPtrPrio = std::make_unique<Wander>();
			ISteeringBehavior* const pEvade = EvadePtr.get();
			ISteeringBehavior* const pWanderPrio = WanderPtrPrio.get();
			OwnedBehaviors.emplace_back(std::move(EvadePtr));
			OwnedBehaviors.emplace_back(std::move(WanderPtrPrio));

			Agent.Behavior = std::make_unique<PrioritySteering>(std::vector<ISteeringBehavior*>{pEvade, pWanderPrio});
			break;
		}
	default:
		assert(false);
	}

	UpdateTarget(Agent);
	Agent.Agent->ResetMaxSpeed();
	Agent.Agent->SetSteeringBehavior(Agent.Behavior.get());
}

void ALevel_CombinedSteering::RefreshTargetLabels()
{
	TargetLabels.clear();
	
	TargetLabels.push_back("Mouse");
	for (int i{0}; i < SteeringAgents.size(); ++i)
	{
		TargetLabels.push_back(std::format("Agent {}", i));
	}
}

void ALevel_CombinedSteering::UpdateTarget(ImGui_Agent& Agent)
{
	bool const bUseMouseAsTarget = Agent.SelectedTarget < 0;
	if (!bUseMouseAsTarget)
	{
		ASteeringAgent* const TargetAgent = SteeringAgents[Agent.SelectedTarget].Agent;

		FTargetData Target;
		Target.Position = TargetAgent->GetPosition();
		Target.Orientation = TargetAgent->GetRotation();
		Target.LinearVelocity = TargetAgent->GetLinearVelocity();
		Target.AngularVelocity = TargetAgent->GetAngularVelocity();

		Agent.Behavior->SetTarget(Target);
	}
	else
	{
		Agent.Behavior->SetTarget(MouseTarget);
	}
}

void ALevel_CombinedSteering::RefreshAgentTargets(unsigned int IndexRemoved)
{
	for (UINT i = 0; i < SteeringAgents.size(); ++i)
	{
		if (i >= IndexRemoved)
		{
			auto& Agent = SteeringAgents[i];
			if (Agent.SelectedTarget == IndexRemoved || i  == Agent.SelectedTarget)
			{
				--Agent.SelectedTarget;
			}
		}
	}
}

// Called every frame
void ALevel_CombinedSteering::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
	
#pragma region UI
	//UI
	{
		//Setup
		bool windowActive = true;
		ImGui::SetNextWindowPos(WindowPos);
		ImGui::SetNextWindowSize(WindowSize);
		ImGui::Begin("Game AI", &windowActive, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
	
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
		ImGui::Spacing();
	
		ImGui::Text("Flocking");
		ImGui::Spacing();
		ImGui::Spacing();
	
		if (ImGui::Checkbox("Debug Rendering", &CanDebugRender))
		{
			// TODO: Handle the debug rendering of your agents here :)
		}
		ImGui::Checkbox("Trim World", &TrimWorld->bShouldTrimWorld);
		if (TrimWorld->bShouldTrimWorld)
		{
			ImGuiHelpers::ImGuiSliderFloatWithSetter("Trim Size",
				TrimWorld->GetTrimWorldSize(), 1000.f, 3000.f,
				[this](float InVal) { TrimWorld->SetTrimWorldSize(InVal); });
		}
		ImGui::Spacing();

#pragma region PerAgentUI
		if (ImGui::Button("Add Agent"))
			AddAgent(BehaviorTypes::BlendedSteering);
		ImGui::Separator();

		for (int i{0}; i < SteeringAgents.size(); ++i)
		{
			ImGui::PushID(i);
			ImGui_Agent& a = SteeringAgents[i];
		
			std::string agentHeader{std::format("Agent {}:", i)};
			if (ImGui::CollapsingHeader(agentHeader.c_str()))
			{
				ImGui::Indent();
				//Actor Props
				if (ImGui::CollapsingHeader("Properties"))
				{
					float v = a.Agent->GetMaxLinearSpeed();
					if (ImGui::SliderFloat("Lin", &v, 0.f, 600.f, "%.2f"))
						a.Agent->SetMaxLinearSpeed(v);

					v = a.Agent->GetMaxAngularSpeed();
					if (ImGui::SliderFloat("Ang", &v, 0.f, 360.f, "%.2f"))
						a.Agent->SetMaxAngularSpeed(v);

					v = a.Agent->GetMass();
					if (ImGui::SliderFloat("Mass ", &v, 0.f, 100.f, "%.2f"))
						a.Agent->SetMass(v);
				}
			
				bool bBehaviourModified = false;

				ImGui::Spacing();
				ImGui::PushID(i + 50);
				ImGui::Text(" Behavior: ");
				ImGui::SameLine();
				ImGui::PushItemWidth(100);

				// Add the names of your steering behaviors
				if (ImGui::Combo("", &a.SelectedBehavior, "Blended Steering\0PrioritySteering", 4))
				{
					bBehaviourModified = true;
				}
				ImGui::PopItemWidth();
				ImGui::PopID();

			
				ImGui::Spacing();
				ImGui::PushID(i + 100);
				ImGui::Text(" Target: ");
				ImGui::SameLine();
				ImGui::PushItemWidth(100);
			
				int selectedTargetOffset = a.SelectedTarget + 1;
				std::string const Label{""};
				std::string Targets{};
				for (auto const & Target : TargetLabels)
				{
					Targets += Target;
					Targets += '\0';
				}
				if (ImGui::Combo(Label.c_str(), &selectedTargetOffset, Targets.c_str()))
				{
					a.SelectedTarget = selectedTargetOffset - 1;
					bBehaviourModified = true;
				}
			
				ImGui::PopItemWidth();
				ImGui::PopID();
				ImGui::Spacing();
				ImGui::Spacing();
			
			
				if (bBehaviourModified)
					SetAgentBehavior(a);

				if (ImGui::Button("x"))
				{
					AgentIndexToRemove = i;
				}

				ImGui::SameLine(0, 20);

				bool isChecked = a.Agent->GetDebugRenderingEnabled();
				if (ImGui::Checkbox("Debug Rendering", &isChecked))
				{
					a.Agent->SetDebugRenderingEnabled(isChecked);
				}

				ImGui::Unindent();
			}
#pragma endregion 
			
			ImGui::Spacing();
	
			ImGui::Text("Behavior Weights");
			ImGui::Spacing();
			
			// Should only be visible if agent is using BlendedSteering
			if (a.SelectedBehavior == static_cast<int>(BehaviorTypes::BlendedSteering) && a.Behavior)
			{
				ImGuiHelpers::ImGuiSliderFloatWithSetter("Seek",
				 static_cast<BlendedSteering*>(a.Behavior.get())->GetWeightedBehaviorsRef()[0].Weight, 0.f, 1.f,
				 [this, &a](const float InVal){static_cast<BlendedSteering*>(a.Behavior.get())->GetWeightedBehaviorsRef()[0].Weight = InVal; }, "%.2f");
		
				ImGuiHelpers::ImGuiSliderFloatWithSetter("Wander",
					static_cast<BlendedSteering*>(a.Behavior.get())->GetWeightedBehaviorsRef()[1].Weight, 0.f, 1.f,
					[this, &a](const float InVal){ static_cast<BlendedSteering*>(a.Behavior.get())->GetWeightedBehaviorsRef()[1].Weight = InVal; }, "%.2f");
			}
			ImGui::PopID();
		}
		
		if (AgentIndexToRemove >= 0)
		{
			RemoveAgent(AgentIndexToRemove);
			AgentIndexToRemove = -1;
		}
		//End
		ImGui::End();
	}
#pragma endregion 
		
	for (ImGui_Agent& a : SteeringAgents)
	{
		if (a.Agent)
		{
		UpdateTarget(a);
		}
	}
}
