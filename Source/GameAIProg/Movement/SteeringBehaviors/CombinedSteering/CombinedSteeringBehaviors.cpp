
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
	float TotalWeight = 0.f;
	bool bHadAnyValid = false;

	for (auto& WeightedBehavior : WeightedBehaviors)
	{
		if (WeightedBehavior.Weight <= 0.f) continue;
		if (!WeightedBehavior.pBehavior) continue;
		
		WeightedBehavior.pBehavior->SetTarget(Target);
		//BlendedSteering.LinearVelocity += WeightedBehavior.pBehavior->CalculateSteering(DeltaT, Agent).LinearVelocity * WeightedBehavior.Weight;
		const SteeringOutput Out = WeightedBehavior.pBehavior->CalculateSteering(DeltaT, Agent);
		if (!Out.IsValid) continue;
		
		bHadAnyValid = true;
		TotalWeight += WeightedBehavior.Weight;

		BlendedSteering.LinearVelocity += Out.LinearVelocity * WeightedBehavior.Weight;
		BlendedSteering.AngularVelocity += Out.AngularVelocity * WeightedBehavior.Weight;

		WeightedBehavior.pBehavior->DrawDebug(Agent);
	}
	
	if (!bHadAnyValid || TotalWeight <= 0.f)
	{
		BlendedSteering.IsValid = false;
		return BlendedSteering;
	}
	
	BlendedSteering.LinearVelocity /= TotalWeight;
	BlendedSteering.AngularVelocity /= TotalWeight;

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
	constexpr float Epsilon{1e-3f};
	constexpr float EpsSq{ Epsilon * Epsilon };
	
	for (ISteeringBehavior* const pBehavior : m_PriorityBehaviors)
	{
		if (!pBehavior) continue;
		
		pBehavior->SetTarget(Target);
		SteeringOutput Steering = pBehavior->CalculateSteering(DeltaT, Agent);
		if (!Steering.IsValid) continue;

		const bool bHasLinear = Steering.LinearVelocity.SquaredLength() > EpsSq;
		const bool bHasAngular = FMath::Abs(Steering.AngularVelocity) > Epsilon;
		
		if (bHasLinear || bHasAngular)
			return Steering;
	}

	SteeringOutput None{};
	None.IsValid = false;
	return None;
}