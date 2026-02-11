#include "SteeringBehaviors.h"
#include <Programs/UnrealBuildAccelerator/Core/Public/UbaBase.h>
#include "GameAIProg/Movement/SteeringBehaviors/SteeringAgent.h"
#include "DrawDebugHelpers.h"
#include "VectorTypes.h"

void ISteeringBehavior::DrawDebug(const ASteeringAgent& Agent)
{
	// Colors
	const FColor Green { FColor::Green };
	const FColor Magenta { FColor::Magenta };
	const FColor Cyan { FColor::Cyan };
	// Data
	const FVector Position{ Agent.GetPosition().X, Agent.GetPosition().Y, 10 };
	const FVector Velocity{ Agent.GetVelocity().X, Agent.GetVelocity().Y, 0 };
	// Math
	const FVector Forward = Agent.GetActorForwardVector().GetSafeNormal();
	const FVector VelNorm = Velocity.GetSafeNormal();

	// Signed angle using atan2 (robust and clean)
	float SignedAngleRad =
		FMath::Atan2(
			FVector::CrossProduct(Forward, VelNorm).Z,
			FVector::DotProduct(Forward, VelNorm)
		);

	float SignedAngleDeg = FMath::RadiansToDegrees(SignedAngleRad);
	const FVector Right = Agent.GetActorRightVector();
	const FVector Direction = (SignedAngleDeg >= 0.f) ? Right : -Right;\
	const float AngleLength { FMath::Abs(SignedAngleDeg) * 20.f };
	
	// Target related stuff
	const FColor Red { FColor::Red }; 
	FVector TargetPosition {Target.Position.X, Target.Position.Y, 10};
	
	// Drawing
	constexpr float LengthFactor{ 0.5f };
	DrawDebugLine(Agent.GetWorld(), Position, Position + (Velocity) * LengthFactor, Green); //Velocity
	DrawDebugLine(Agent.GetWorld(), Position, Position + ((Agent.GetActorForwardVector() * Agent.GetLinearVelocity().Length()))*LengthFactor, Magenta); // Forward
	DrawDebugLine(Agent.GetWorld(), Position, Position + (Direction * AngleLength) * LengthFactor, Cyan); // Angle
	// Drawing Target
	DrawDebugCircle(Agent.GetWorld(), TargetPosition, 10.f, 8, Red,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
}

SteeringOutput Seek::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	constexpr float CloseDstSquared{25.f};
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	if (Steering.LinearVelocity.SquaredLength() <= CloseDstSquared)
		Steering.LinearVelocity = FVector2D::ZeroVector;	
	
	DrawDebug(Agent);
	return Steering;
}

SteeringOutput Flee::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	Steering.LinearVelocity = -(Target.Position - Agent.GetPosition());
	Steering.LinearVelocity.Normalize();
	
	DrawDebug(Agent);
	return Steering; 
}

SteeringOutput Arrive::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	static constexpr float MaxSpeed{600.f};
	
	Steering.LinearVelocity = Target.Position - Agent.GetPosition();
	
	const float DistanceToTarget{static_cast<float>(Steering.LinearVelocity.Length())};
	
	if (DistanceToTarget > m_SlowRadius)
	{
		Agent.ResetMaxSpeed();
	}
	else if (DistanceToTarget <= m_TargetRadius)
	{
		Agent.SetMaxLinearSpeed(0);
	}
	else if (DistanceToTarget <= m_SlowRadius)
	{
		const float SpeedMargin{(DistanceToTarget - m_TargetRadius)/(m_SlowRadius - m_TargetRadius)};
		Agent.SetMaxLinearSpeed(MaxSpeed*SpeedMargin);
	}

	DrawDebug(Agent);
	return Steering;
}

void Arrive::DrawDebug(const ASteeringAgent& Agent)
{
	ISteeringBehavior::DrawDebug(Agent);
	
	const FVector Position{ Agent.GetPosition().X, Agent.GetPosition().Y, 10};
	static const FColor SlowColor{FColor::Blue};
	static const FColor TargetColor{FColor::Red};
	DrawDebugCircle(Agent.GetWorld(), Position, m_SlowRadius, 20, SlowColor,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
	DrawDebugCircle(Agent.GetWorld(), Position, m_TargetRadius, 10, TargetColor,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
}

SteeringOutput Face::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	FVector2D ToTarget = Target.Position - Agent.GetPosition();

	if (ToTarget.IsNearlyZero())
		return Steering;

	FVector Forward = Agent.GetActorForwardVector();
	FVector2D Forward2D(Forward.X, Forward.Y);

	ToTarget.Normalize();
	Forward2D.Normalize();

	float SignedAngle =
		FMath::Atan2(
			Forward2D.X * ToTarget.Y - Forward2D.Y * ToTarget.X,
			FVector2D::DotProduct(Forward2D, ToTarget)
		);

	float RotationSpeed = 5.f; // tune this

	Steering.AngularVelocity = SignedAngle * RotationSpeed;
	Steering.LinearVelocity = FVector2D::ZeroVector;
	
	DrawDebug(Agent);
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
	
	DrawDebug(Agent);
	return Steering; 
}

SteeringOutput Evade::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	SteeringOutput Steering{};
	
	FVector2D ToThreat{Target.Position - Agent.GetPosition()};
	const float DistanceToThreat{static_cast<float>(ToThreat.Length())};
	
	// If outside Evade radius -> do nothing
	if (DistanceToThreat > m_EvadeRadius)
	{
		return Steering;
	}
	
	// Clamp prediction Time
	float Time{DistanceToThreat/Agent.GetMaxLinearSpeed()};
	Time = uba::Min(Time, 4.f);
	
	FVector2D PredictedThreatPosition{ Target.Position + Target.LinearVelocity * Time};
	FVector2D DesiredPosition{(Agent.GetPosition() - PredictedThreatPosition) * Agent.GetMaxLinearSpeed()}; 
	
	Steering.LinearVelocity = DesiredPosition;
	
	DrawDebug(Agent);
	return Steering;
}

void Evade::DrawDebug(const ASteeringAgent& Agent)
{
	ISteeringBehavior::DrawDebug(Agent);
	
	const FVector Position{ Agent.GetPosition().X, Agent.GetPosition().Y, 10};
	static const FColor EvadeColor{FColor::Red};
	DrawDebugCircle(Agent.GetWorld(), Position, m_EvadeRadius, 20, EvadeColor,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
}

SteeringOutput Wander::CalculateSteering(float DeltaT, ASteeringAgent& Agent)
{
	const FVector Forward{Agent.GetActorForwardVector()};
	const FVector Position { Agent.GetPosition().X, Agent.GetPosition().Y, 10 };
	// Calculate Circle Center (the one in front, duh)
	m_CircleCenter = Position + Forward * m_OffsetDistance;
	
	// Get a random angle change
	float deltaAngle{ FMath::FRandRange(-m_MaxAngleChange, m_MaxAngleChange) };
	m_WanderAngle += deltaAngle * DeltaT;
	
	// Local offset
	FVector2D LocalOffset{cos(m_WanderAngle), sin(m_WanderAngle)};
	LocalOffset *= m_Radius;

	// Convert to world
	FVector Right = FVector::CrossProduct(FVector(0,0,1),FVector(Forward.X, Forward.Y, 0));
	FVector WorldOffset = Forward * LocalOffset.X + Right * LocalOffset.Y;
	
	// Seek the target
	const FVector NewTargetPos{ m_CircleCenter + WorldOffset };
	Target.Position = FVector2D{NewTargetPos.X, NewTargetPos.Y};
	
	DrawDebug(Agent);
	return Seek::CalculateSteering(DeltaT, Agent);
}

void Wander::DrawDebug(const ASteeringAgent& Agent)
{
	ISteeringBehavior::DrawDebug(Agent);
	
	const FVector CirclePosition{m_CircleCenter.X, m_CircleCenter.Y, 10};
	DrawDebugCircle(Agent.GetWorld(), CirclePosition, m_Radius, 10, FColor::Blue,
					false, -1, 0, 0, 
					FVector(0,1,0), FVector(1,0,0), false);
}
