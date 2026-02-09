#pragma once

#include <Movement/SteeringBehaviors/SteeringHelpers.h>


class ASteeringAgent;

// SteeringBehavior base, all steering behaviors should derive from this.
class ISteeringBehavior
{
public:
	ISteeringBehavior() = default;
	virtual ~ISteeringBehavior() = default;

	// Override to implement your own behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) = 0;

	void SetTarget(const FTargetData& NewTarget) { Target = NewTarget; }
	
	template<class T, std::enable_if_t<std::is_base_of_v<ISteeringBehavior, T>>* = nullptr>
	T* As()
	{ return static_cast<T*>(this); }

protected:
	FTargetData Target;
};

// Your own SteeringBehaviors should follow here...
class  Seek : public ISteeringBehavior
{
public:
	Seek() = default;
	virtual  ~Seek() override = default;
	
	//Seek Behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
};

class Flee : public ISteeringBehavior
{
public:
	Flee() = default;
	virtual ~Flee() override = default;
	
	//Flee Behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
};

class Arrive : public ISteeringBehavior
{
public:
	Arrive() = default;
	virtual ~Arrive() override = default;
	
	//Arrive Behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
};

class Face : public ISteeringBehavior
{
public:
	Face() = default;
	virtual ~Face() override = default;
	
	//Face Behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
};

class Pursuit : public ISteeringBehavior
{
public:
	Pursuit() = default;
	virtual ~Pursuit() override = default;
	
	//Pursuit Behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
};

class Evade : public ISteeringBehavior
{
public:
	Evade() = default;
	virtual ~Evade() override = default;
	
	//Evade Behavior
	virtual SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
};

class Wander : public Seek
{
public:
	Wander() = default;
	virtual ~Wander() override = default;
	
	//Wander Behavior
	SteeringOutput CalculateSteering(float DeltaT, ASteeringAgent & Agent) override;
	
	void SetWanderOffset(float offset)	{m_OffsetDistance = offset;}
	void SetWanderRadius(float radius)	{m_Radius = radius;}
	void SetMaxAngleChange(float rad)	{m_MaxAngleChange = rad;}
	
protected:
	float m_OffsetDistance{120.f};					// Offset (Agent Direction)
	float m_Radius{80.f};							// WanderRadius
	float m_MaxAngleChange{ToRadians(45)};	// Max WanderAngle change per frame
	float m_WanderAngle{};							// Internal
};
