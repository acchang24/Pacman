#include "Actor.h"
#include "Game.h"
#include "Component.h"
#include <algorithm>

Actor::Actor(Game* game)
	:mGame(game)
	,mState(ActorState::Active)
	,mPosition(Vector2::Zero)
	,mScale(1.0f)
	,mRotation(0.0f)
{
	// Add actor to game with AddActor
	mGame->AddActor(this);
	spawnRoom = "";
}

Actor::~Actor()
{
	// Call RemoveActor
	mGame->RemoveActor(this);
	
	// delete each component
	for (auto comp : mComponents)
	{
		delete comp;
	}
}

void Actor::Update(float deltaTime)
{
	// If the actor's state is ActorState::Active call update on all components
	if (mState == ActorState::Active)
	{
		// Call update on all its components in the vector
		for (auto comp : mComponents)
		{
			comp->Update(deltaTime);
		}
		// Call OnUpdate
		OnUpdate(deltaTime);
	}
}

void Actor::OnUpdate(float deltaTime)
{
}

void Actor::ProcessInput(const Uint8* keyState)
{
	// If the Actor's state is Active, call ProcessInput on all components
	if (mState == ActorState::Active)
	{
		// call ProcessInput on all components
		for (auto comp : mComponents)
		{
			// call ProcessInput
			comp->ProcessInput(keyState);
		}
		// Call OnProcessInput
		OnProcessInput(keyState);
	}
}

void Actor::OnProcessInput(const Uint8* keyState)
{
}

void Actor::AddComponent(Component* c)
{
	mComponents.emplace_back(c);
	std::sort(mComponents.begin(), mComponents.end(), [](Component* a, Component* b) {
		return a->GetUpdateOrder() < b->GetUpdateOrder();
	});
}

Vector2 Actor::GetForward()
{
	return Vector2(Math::Cos(mRotation), -Math::Sin(mRotation));
}
