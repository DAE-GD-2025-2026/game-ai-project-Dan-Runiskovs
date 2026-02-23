
#include "CombinedSteeringBehaviors.h"
#include <algorithm>
#include "../SteeringAgent.h"

BlendedSteering::BlendedSteering(const std::vector<WeightedBehavior>& WeightedBehaviors)
	:WeightedBehaviors(WeightedBehaviors)
{};

//****************
//BLENDED STEERING
SteeringOutput BlendedSteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput BlendedSteering = {};
	// TODO: Calculate the weighted average steering behavior
	for (auto& WeightedBehavior : WeightedBehaviors)
	{
		if (WeightedBehavior.Weight <= 0.f) continue;
		
		WeightedBehavior.pBehavior->SetTarget(Target);
		BlendedSteering.LinearVelocity += WeightedBehavior.pBehavior->CalculateSteering(DeltaT, Agent).LinearVelocity * WeightedBehavior.Weight;
		
		WeightedBehavior.pBehavior->DrawDebug(Agent);
	}
	
	return BlendedSteering;
}

float* BlendedSteering::GetWeight(ISteeringBehavior* const SteeringBehavior)
{
	const auto it = std::ranges::find_if(WeightedBehaviors.begin(),
		WeightedBehaviors.end(),
		[SteeringBehavior](const WeightedBehavior& Elem)
		{
			return Elem.pBehavior == SteeringBehavior;
		}
	);

	if(it!= WeightedBehaviors.end())
		return &it->Weight;
	
	return nullptr;
}

//*****************
//PRIORITY STEERING
SteeringOutput PrioritySteering::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering = {};

	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		Steering = pBehavior->CalculateSteering(DeltaT, Agent);

		if (Steering.IsValid)
			break;
	}

	//If none of the behavior return a valid output, last behavior is returned
	return Steering;
}