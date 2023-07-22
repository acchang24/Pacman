#include "MoveComponent.h"
#include "Actor.h"
#include "SDL2/SDL.h"

MoveComponent::MoveComponent(class Actor* owner)
:Component(owner, 50)
,mAngularSpeed(0.0f)
,mForwardSpeed(0.0f)
{
	
}

void MoveComponent::Update(float deltaTime)
{
	// Update owning actor's rotation by angular speed times delta time
	mOwner->SetRotation(mOwner->GetRotation() + mAngularSpeed * deltaTime);
	// Update owning actor's position based on forward vector, forward speed, and delta time
	mOwner->SetPosition(mOwner->GetPosition() + mOwner->GetForward() * mForwardSpeed * deltaTime);
}
