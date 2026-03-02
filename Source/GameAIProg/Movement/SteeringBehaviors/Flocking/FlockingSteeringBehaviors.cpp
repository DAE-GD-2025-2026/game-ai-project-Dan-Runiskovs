#include "FlockingSteeringBehaviors.h"
#include "Flock.h"
#include "../SteeringAgent.h"
#include "../SteeringHelpers.h"


//*******************
//COHESION (FLOCKING)
SteeringOutput Cohesion::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	
	// --- Guards ---
	if (!pFlock) return Steering;
	if (pFlock->GetNrOfNeighbors() == 0)
	{
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}
	
	// --- Make Target -> avg Position ---
	const auto avgPosition{ pFlock->GetAverageNeighborPos() };
	FTargetData cohesionTarget{}; 
	cohesionTarget.Position = avgPosition;
	SetTarget(cohesionTarget);
	
	// --- Out ---
	return Seek::CalculateSteering(deltaT, pAgent);
}
//*********************
//SEPARATION (FLOCKING)
SteeringOutput Separation::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	
	// --- Guards ---
	if (!pFlock) return Steering;
	const auto nrNeighbors{ pFlock->GetNrOfNeighbors()};
	if (nrNeighbors == 0)
	{
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}
	
	// --- Set Up ---
	const auto& arrNeighbours = pFlock->GetNeighbors();
	const auto position{ pAgent.GetPosition()};
	FVector2D force = FVector2D::ZeroVector;
	
	// --- Calculate necessary force ---
	for (int i{ 0 }; i < nrNeighbors; ++i)
	{
		auto* pNeighbour = arrNeighbours[i];
		
		if (!IsValid(pNeighbour)) continue;
		
		const FVector2D toNeighbor{position - pNeighbour->GetPosition()};
		const auto dstToNeighbor{static_cast<float>(toNeighbor.Size())};

		force += (toNeighbor / dstToNeighbor) * (1.f / dstToNeighbor);
	}
	
	// --- Set correct speed ---
	const auto maxSpeed{ pAgent.GetMaxLinearSpeed()};
	if (!force.IsNearlyZero())
	{
		force *= maxSpeed;
		force = force.GetClampedToMaxSize(maxSpeed);
	}
	
	// --- Out ---
	Steering.LinearVelocity = force;
	return Steering;
}

//*************************
//VELOCITY MATCH (FLOCKING)
SteeringOutput VelocityMatch::CalculateSteering(float deltaT, ASteeringAgent& pAgent)
{
	SteeringOutput Steering{};
	
	// --- Guards ---
	if (!pFlock) return Steering;
	if (pFlock->GetNrOfNeighbors() == 0)
	{
		Steering.LinearVelocity = FVector2D::ZeroVector;
		return Steering;
	}
	
	// --- Get and Clamp max velocity ---
	auto avgVel = pFlock->GetAverageNeighborVelocity();
	avgVel = avgVel.GetClampedToMaxSize(pAgent.GetMaxLinearSpeed());
	
	// --- Out ---
	Steering.LinearVelocity = avgVel;
	return Steering;
}