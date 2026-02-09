#include "SteeringBehaviors.h"

#include <Programs/UnrealBuildAccelerator/Core/Public/UbaBase.h>

#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"
#include "VectorTypes.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	constexpr float CloseDstSquared{25.f};
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	if (Steering.LinearVelocity.SquaredLength() <= CloseDstSquared)
		Steering.LinearVelocity = FVector2D::ZeroVector;	
	
	//TODO: Add debug rendering for grades!
	
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = -(Target.Position - Agent.GetPosition());
	Steering.LinearVelocity.Normalize();
	
	return Steering; 
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};

	static constexpr float SlowRadius{300.f};
	static constexpr float TargetRadius{100.f};
	static constexpr float MaxSpeed{600.f};
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	const float DistanceToTarget{static_cast<float>(Steering.LinearVelocity.Length())};
	
	if (DistanceToTarget > SlowRadius)
	{
		Agent.ResetMaxSpeed();
	}
	else if (DistanceToTarget <= TargetRadius)
	{
		Agent.SetMaxLinearSpeed(0);
	}
	else if (DistanceToTarget <= SlowRadius)
	{
		const float SpeedMargin{(DistanceToTarget - TargetRadius)/(SlowRadius - TargetRadius)};
		Agent.SetMaxLinearSpeed(MaxSpeed*SpeedMargin);
	}
	
#pragma region DebugDrawing
	const FVector Position{ Agent.GetPosition().X, Agent.GetPosition().Y, 10};
	static const FColor SlowColor{FColor::Blue};
	static const FColor TargetColor{FColor::Red};
	
	DrawDebugCircle(Agent.GetWorld(), Position, SlowRadius, 20, SlowColor,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
	DrawDebugCircle(Agent.GetWorld(), Position, TargetRadius, 10, TargetColor,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
#pragma endregion
	
	return Steering;
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Agent.SetMaxLinearSpeed(0); // stop any movement :)
	
	return Steering;
}

SteeringOutput Pursuit::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	FVector2D DistanceVector = Target.Position - Agent.GetPosition();
	const float DistanceToTarget{static_cast<float>(DistanceVector.Length())};
	float Time{DistanceToTarget/Agent.GetMaxLinearSpeed()};
	
	// Optional clamp time
	constexpr float MaxPredictionTime{4};
	Time = uba::Min(Time, MaxPredictionTime);
	
	//Predict position
	const FVector2D PredictedPosition = Target.Position + Target.LinearVelocity * Time;
	
	Steering.LinearVelocity = PredictedPosition - Agent.GetPosition();
	
	//Don't move if stands in the predicted position
	constexpr float CloseDstSquared{25.f};
	if (Steering.LinearVelocity.SquaredLength() <= CloseDstSquared)
		Steering.LinearVelocity = FVector2D::ZeroVector;
	
	return Steering; 
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	constexpr float EvadeRadius{400.f};
	FVector2D ToThreat{Target.Position - Agent.GetPosition()};
	const float DistanceToThreat{static_cast<float>(ToThreat.Length())};
	
	// If outside Evade radius -> do nothing
	if (DistanceToThreat > EvadeRadius)
	{
		return Steering;
	}
	
	// Clamp prediction Time
	float Time{DistanceToThreat/Agent.GetMaxLinearSpeed()};
	Time = uba::Min(Time, 4.f);
	
	FVector2D PredictedThreatPosition{ Target.Position + Target.LinearVelocity * Time};
	FVector2D DesiredPosition{(Agent.GetPosition() - PredictedThreatPosition) * Agent.GetMaxLinearSpeed()}; 
	
	Steering.LinearVelocity = DesiredPosition;
	
#pragma region DebugDrawing
	const FVector Position{ Agent.GetPosition().X, Agent.GetPosition().Y, 10};
	static const FColor EvadeColor{FColor::Red};
	DrawDebugCircle(Agent.GetWorld(), Position, EvadeRadius, 20, EvadeColor,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
#pragma endregion
	
	return Steering;
}
