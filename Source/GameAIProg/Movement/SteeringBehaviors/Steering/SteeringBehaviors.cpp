#include "SteeringBehaviors.h"
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"

//SEEK
//*******
// TODO: Do the Week01 assignment :^)
SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	Steering.LinearVelocity.Normalize();
	
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
