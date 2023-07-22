#include "CollisionComponent.h"
#include "MoveComponent.h"
#include "Actor.h"
#include "SDL2/SDL.h"
#include <algorithm>

CollisionComponent::CollisionComponent(class Actor* owner)
:Component(owner)
,mWidth(0.0f)
,mHeight(0.0f)
{
	
}

CollisionComponent::~CollisionComponent()
{
	
}

bool CollisionComponent::Intersect(const CollisionComponent* other)
{
	bool case1 = (other->GetMax().x < GetMin().x);
	bool case2 = (GetMax().x < other->GetMin().x);
	bool case3 = (other->GetMax().y < GetMin().y);
	bool case4 = (GetMax().y < other->GetMin().y);
	return !(case1 || case2 || case3 || case4);
}

Vector2 CollisionComponent::GetMin() const
{
	float minX = mOwner->GetPosition().x - (mWidth * mOwner->GetScale() / 2.0f);
	float minY = mOwner->GetPosition().y - (mHeight * mOwner->GetScale() / 2.0f);
	return Vector2(minX, minY);
}

Vector2 CollisionComponent::GetMax() const
{
	float maxX = mOwner->GetPosition().x + (mWidth * mOwner->GetScale() / 2.0f);
	float maxY = mOwner->GetPosition().y + (mHeight * mOwner->GetScale() / 2.0f);
	return Vector2(maxX, maxY);
}

const Vector2& CollisionComponent::GetCenter() const
{
	return mOwner->GetPosition();
}

CollSide CollisionComponent::GetMinOverlap(
	const CollisionComponent* other, Vector2& offset)
{
	offset = Vector2::Zero;
	
	// See if anything intersects
	if (Intersect(other))
	{
		// Owner's position
		Vector2 currPos = mOwner->GetPosition();

		// Calculate four other Min/Max X/y Diff variables and get the min absolute value
		float otherMinYDiff = (other->GetMin().y - GetMax().y);
		float otherMaxYDiff = (other->GetMax().y - GetMin().y);
		float otherMinXDiff = (other->GetMin().x - GetMax().x);
		float otherMaxXDiff = (other->GetMax().x - GetMin().x);

		// Find lowest absolute value for closest edge
		float min = std::min({ abs(otherMinXDiff), abs(otherMinYDiff), abs(otherMaxXDiff), abs(otherMaxYDiff) });

		//SDL_Log("left = %f, top = %f, right = %f, bottom = %f", otherMinXDiff, otherMinYDiff, otherMaxXDiff, otherMaxYDiff);

		if (min == abs(otherMinYDiff))
		{
			//SDL_Log("Top");
			// offset 
			//SDL_Log("%f", offset.y);
			//SDL_Log("%f", otherMinYDiff);
			offset.y += otherMinYDiff;
			// Set Position when collide
			mOwner->SetPosition(Vector2(currPos.x, currPos.y + offset.y));
			// return collision side
			return CollSide::Top;
		}
		else if (min == abs(otherMaxYDiff))
		{
			//SDL_Log("Bottom");
			//SDL_Log("%f", offset.y);
			//SDL_Log("%f", otherMaxYDiff);
			offset.y += otherMaxYDiff;
			// Set Position when collide
			mOwner->SetPosition(Vector2(currPos.x, currPos.y + offset.y));
			return CollSide::Bottom;
		}
		else if (min == abs(otherMinXDiff))
		{
			//SDL_Log("Left");
			//SDL_Log("%f", offset.x);
			//SDL_Log("%f", otherMinXDiff);
			offset.x += otherMinXDiff;
			// Set Position when collide
			mOwner->SetPosition(Vector2(offset.x + currPos.x, currPos.y));
			return CollSide::Left;
		}
		else if (min == abs(otherMaxXDiff))
		{
			//SDL_Log("Right");
			//SDL_Log("%f", offset.x);
			//SDL_Log("%f", otherMaxXDiff);
			offset.x += otherMaxXDiff;
			
			// Set Position when collide
			mOwner->SetPosition(Vector2(offset.x + currPos.x, currPos.y));
			
			return CollSide::Right;
		}
	}
	return CollSide::None;
}
