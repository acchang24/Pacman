#include "GhostAI.h"
#include "Actor.h"
#include "Game.h"
#include "CollisionComponent.h"
#include "Game.h"
#include "PathNode.h"
#include "AnimatedSprite.h"
#include <SDL2/SDL.h>
#include <unordered_map>
#include "Ghost.h"
#include "PacMan.h"
#include "Random.h"

GhostAI::GhostAI(class Actor* owner)
:Component(owner, 50)
{
	mGhost = static_cast<Ghost*>(owner);
}

void GhostAI::Update(float deltaTime)
{
	// 1. Update time
	UpdateTimer(deltaTime);

	// 2. Update Ghost Position based on direction
	// Get the owning Ghost's position
	Vector2 ghostPos = mGhost->GetPosition();

	// Get the direction from ghost to nextNode in path
	Vector2 nextNodePos = mNextNode->GetPosition();
	Vector2 direction = nextNodePos - ghostPos;
	direction.Normalize();

	// update position
	ghostPos += mSpeed * direction * deltaTime;
	// Set ghost's position
	mGhost->SetPosition(ghostPos);
	// 3. check for collisions with the nextNode of the path
	CollisionComponent* ghostBox = mGhost->GetComponent<CollisionComponent>();
	CollisionComponent* nodeBox = mNextNode->GetComponent <CollisionComponent>();
	// If there is collision with ghost and node
	if (ghostBox->Intersect(nodeBox))
	{
		// 4. Update Path based on state
		// Scatter State
		if (mState == State::Scatter)
		{
			// Follow path 
			if (!mPath.empty())
			{
				FollowPath();
			}
			else
			{
				mPath = GetShortestPath(mNextNode, mGhost->GetScatterNode());
			}
			
		}
		// Frightened State
		else if (mState == State::Frightened)
		{
			// Get path to random neighbor node
			GoToRandomNeighbor();
		}

		// Chase State
		else if (mState == State::Chase)
		{
			// clear path
			mPath.clear();

			// Get Different types of ghosts
			if (mGhost->GetType() == Ghost::Type::Blinky)
			{
				// Follow the player
				FollowPlayer();
			}
			else if (mGhost->GetType() == Ghost::Type::Pinky)
			{
				mPath = GetShortestPath(mNextNode, GetClosestNode(mGhost->GetGame()->mPlayer->GetPointInFrontOf(80.0f)));
			}
			else if (mGhost->GetType() == Ghost::Type::Inky)
			{
				// Get point 40 units in front of player
				Vector2 pointP = mGhost->GetGame()->mPlayer->GetPointInFrontOf(40.0f);
				// Vector v from Blinky to p
				Vector2 vectorV = pointP - (mGhost->GetGame()->mGhosts[0]->GetPosition());
				// Double the length of v and add to blinky's position to get q
				Vector2 pointQ = (vectorV * 2) + mGhost->GetGame()->mGhosts[0]->GetPosition();
				// Find path node that's near to this position
				mPath = GetShortestPath(mNextNode, GetClosestNode(pointQ));

			}
			else
			{
				// Get the distance between clyde and the player
				float distance = (mGhost->GetGame()->mPlayer->GetPosition() - mGhost->GetPosition()).Length();
				// If the distance is > 150.0f
				if (distance > 150.0f)
				{
					// Follow the player
					FollowPlayer();
				}
				else
				{
					// Scatter
					mPath = GetShortestPath(mNextNode, mGhost->GetScatterNode());
				}
			}
		}

		// Dead State
		else if (mState == State::Dead)
		{
			// Follow path 
			if (!mPath.empty())
			{
				FollowPath();
			}
			else 
			{
				// reached ghost pen
				if (mNextNode == mGhost->GetGame()->mGhostPen)
				{
					// Call start
					Start(mGhost->GetSpawnNode());
				}
				else
				{
					// Set new path from current node to ghostPen
					mPath = GetShortestPath(mNextNode, mGhost->GetGame()->mGhostPen);
				}
			}
		}
	}

	// 6. change the animations
	// Get new direction of prevNode to NextNode
	direction = Vector2::Normalize(mNextNode->GetPosition() - mPrevNode->GetPosition());
	UpdateAnims(direction);
}


void GhostAI::Frighten()
{
	// Reset scatter/chase timers
	timer = 0.0f;

	// If the ghost state is dead or ghost just spawned in pen while 
	// Pellet is eaten
	if (mState == State::Dead || mPrevNode->GetType() == PathNode::Type::Ghost)
	{
		// Do nothing
		return;
	}
	else if (mState == State::Frightened)
	{
		// reset timer in case another power pellet 
		// is collected while timer is still going
		timer = 0.0f;
	}
	// Set ghost state to frighten
	mState = State::Frightened;
	// Set the ghost's speed to 65
	mSpeed = 65.0f;
	// Switch directions
	std::swap(mNextNode, mPrevNode);
	// Clear mPath
	mPath.clear();
	// Set animation to frighten
	mOwner->GetComponent<AnimatedSprite>()->SetAnimation("scared0");

}

void GhostAI::Start(PathNode* startNode)
{
	timer = 0.0f;

	// Reset speed back to normal
	mSpeed = 90.0f;

	// Reset Any paths
	mPath.clear();

	// 1. Set position of the owner to the position of the startNode
	mGhost->SetPosition(startNode->GetPosition());

	// 2. Set mState to scatter
	mState = State::Scatter;

	// 3. Set mPrevNode, mNextNode, mTargetNode to nullptr
	mPrevNode = nullptr;
	mNextNode = nullptr; 
	mTargetNode = nullptr;

	// 4. Setup path from startNode to scatterNode and set to mPath
	mPath = GetShortestPath(startNode, mGhost->GetScatterNode());
}

void GhostAI::Die()
{
	// Change its state to dead
	mState = State::Dead;

	// Change its speed to 125.0f;
	mSpeed = 225.0f;
	
	// Clear Path
	mPath.clear();

	// reset the frightenTimer
	timer = 0.0f;
}

void GhostAI::DebugDrawPath(SDL_Renderer* render)
{
	// Draw a rectangle at the target node
	if (mTargetNode != nullptr)
	{
		const int SIZE = 16;
		SDL_Rect r;
		r.x = static_cast<int>(mTargetNode->GetPosition().x) - SIZE / 2;
		r.y = static_cast<int>(mTargetNode->GetPosition().y) - SIZE / 2;
		r.w = SIZE;
		r.h = SIZE;
		SDL_RenderDrawRect(render, &r);
	}

	// Line from ghost to next node
	if (mNextNode != nullptr)
	{
		SDL_RenderDrawLine(render,
			static_cast<int>(mOwner->GetPosition().x),
			static_cast<int>(mOwner->GetPosition().y),
			static_cast<int>(mNextNode->GetPosition().x),
			static_cast<int>(mNextNode->GetPosition().y));
	}

	// Exit if no path
	if (mPath.empty())
	{
		return;
	}

	if (mNextNode)
	{
		// Line from next node to subsequent on path
		SDL_RenderDrawLine(render,
			static_cast<int>(mNextNode->GetPosition().x),
			static_cast<int>(mNextNode->GetPosition().y),
			static_cast<int>(mPath.back()->GetPosition().x),
			static_cast<int>(mPath.back()->GetPosition().y));
	}

	// Lines for rest of path
	for (size_t i = 0; i < mPath.size() - 1; i++)
	{
		SDL_RenderDrawLine(render,
			static_cast<int>(mPath[i]->GetPosition().x),
			static_cast<int>(mPath[i]->GetPosition().y),
			static_cast<int>(mPath[i + 1]->GetPosition().x),
			static_cast<int>(mPath[i + 1]->GetPosition().y));
	}
}

std::vector<PathNode*> GhostAI::GetShortestPath(PathNode* start, PathNode* target) 
{
	//// Calculate path using A*

	// Map of PathNodes to NodeInfo
	std::unordered_map<class PathNode*, NodeInfo> info;

	// Get Current node
	PathNode* currentNode = start;

	// Set visited bool to true if its not the end node
	if (currentNode != target)
	{
		info[currentNode].IsClosed = true;
	}
	// Add mNext and mPrev nodes to end of currentNode's unsusable
	info[currentNode].Unusable.push_back(mNextNode);
	info[currentNode].Unusable.push_back(mPrevNode);

	// Vectors for open/closed sets
	std::vector<PathNode*> openSet;

	// Loop through nodes
	do {
		// loop through current node's adjacency list
		for (auto n : currentNode->mAdjacent)
		{
			// See if the adjacent nodes have not been visited
			// Only get nodes that are not tunnel nodes and
			// nodes that are not in the currentNode's unusable
			if(!info[n].IsClosed && n->GetType() != PathNode::Tunnel &&
				(std::find(info[currentNode].Unusable.begin(), info[currentNode].Unusable.end(), n) == info[currentNode].Unusable.end()))
			{
				// Check if the adjacent node is in the openSet vector (Check for adoption)
				if (std::find(openSet.begin(), openSet.end(), n) != openSet.end())
				{
					// Set it's actual cost for g(x)
					float new_g = info[currentNode].g + GetHeuristicDistance(n, currentNode);
					// See if new g is less than adjacent's g
					if (new_g < info[n].g)
					{
						// set adjacent's parent to the currentNode
						info[n].parent = currentNode;
						// Updated a node n's parent
						// set n's unusuable to parent's unusable
						info[n].Unusable = info[info[n].parent].Unusable;
						// if n's unusuable isn't empty pop back from it
						if (!info[n].Unusable.empty())
						{
							info[n].Unusable.pop_back();
						}

						// set adjacent's g(x) to new_g
						info[n].g = new_g;
						// set adjacent's f to g plus h
						info[n].f = info[n].g + info[n].h;
					}
				}
				else // adjacent node is not in the openSet vector
				{
					// set its parent to currentNode
					info[n].parent = currentNode;
					// set n's unusuable to parent's unusable
					info[n].Unusable = info[info[n].parent].Unusable;
					// if n's unusuable isn't empty pop back from it
					if (!info[n].Unusable.empty())
					{
						info[n].Unusable.pop_back();
					}

					// set its heuristic from start to target
					info[n].h = GetHeuristicDistance(n, target);
					// set its g
					info[n].g = info[currentNode].g + GetHeuristicDistance(currentNode, n);
					// set its f
					info[n].f = info[n].g + info[n].h;
					// add adjacent node to openSet
					openSet.push_back(n);
				}
			}
		}

		// if openSet is empty
		if (openSet.empty())
		{
			break;
		}
		// Node with lowest f in openSet
		PathNode* lowestFNode = *openSet.begin();
		for (auto node : openSet)
		{
			if (info[node].f < info[lowestFNode].f)
			{
				lowestFNode = node;
			}
		}
		// Set current node to that lowest f node
		currentNode = lowestFNode;

		// Remove currentNode from openSet
		openSet.erase(std::find(openSet.begin(), openSet.end(), currentNode));

		// Set IsClosed to true
		info[currentNode].IsClosed = true;

	} while (currentNode != target);

	// Setup the path
	// Loop while the current node's parent is not null
	while (info[currentNode].parent != nullptr)
	{
		// Push back to mPath vector
		mPath.push_back(currentNode);
		// Stop when going to loop past start
		if (info[currentNode].parent == target)
		{
			break;
		}
		// update currentNode
		currentNode = info[currentNode].parent;
	}

	// Setup the path member variables
	// Set the previous node to the start
	mPrevNode = start;
	// set next node to the path vector's last node
	mNextNode = mPath.back();
	// set targetnode to target
	mTargetNode = target;

	// Return the path
	return mPath;
}

float GhostAI::GetHeuristicDistance(class PathNode* start, class PathNode* target)
{
	return (target->GetPosition() - start->GetPosition()).Length();
}

void GhostAI::FollowPath()
{
	// If reached here, the ghost has arrived to a new node
	// Set mPrevNode to mNextNode
	mPrevNode = mNextNode;
	// Temp to hold the next node, pop off next node from mPath
	PathNode* newNextNode = mPath.back();
	mPath.pop_back();
	// Save in mNextNode
	mNextNode = newNextNode;

}

void GhostAI::UpdateAnims(Vector2 direction)
{
	// Get the ghost's animation component
	AnimatedSprite* ghostAnim = mGhost->GetComponent<AnimatedSprite>();
	if (mState == State::Scatter || mState == State::Chase)
	{
		if (direction.y < 0.0f)
		{
			ghostAnim->SetAnimation("up");
		}
		else if (direction.y > 0.0f)
		{
			ghostAnim->SetAnimation("down");
		}
		else if (direction.x < 0.0f)
		{
			ghostAnim->SetAnimation("left");
		}
		else if (direction.x > 0.0f)
		{
			ghostAnim->SetAnimation("right");
		}
	}
	else if (mState == State::Dead)
	{
		if (direction.y < 0.0f)
		{
			ghostAnim->SetAnimation("deadup");
		}
		else if (direction.y > 0.0f)
		{
			ghostAnim->SetAnimation("deaddown");
		}
		else if (direction.x < 0.0f)
		{
			ghostAnim->SetAnimation("deadleft");
		}
		else if (direction.x > 0.0f)
		{
			ghostAnim->SetAnimation("deadright");
		}
	}
}

void GhostAI::GoToRandomNeighbor()
{
	// Loop through to get all valid nodes
	std::vector<PathNode*> validNode;
	for (auto node : mNextNode->mAdjacent)
	{
		// Check to see if node type is not its previous, tunnel nodes, or the pen node
		if (node != mPrevNode && node->GetType() != PathNode::Type::Ghost &&
			node->GetType() != PathNode::Type::Tunnel) 
		{
			// If valid, add to vailidNode
			validNode.push_back(node);
		}
	}
	// pick random index from size of valid node vector with std::rand
	int randIndex = std::rand() % validNode.size();

	// set new temp node
	PathNode* newNeighbor = validNode[randIndex];
	// previous set current(next) node
	mPrevNode = mNextNode;
	// set next to new neighbor
	mNextNode = newNeighbor;

	// clear the vector
	validNode.clear();
}

PathNode* GhostAI::GetClosestNode(Vector2 pos)
{
	// Get vector of pathnodes
	std::vector<PathNode*> pathNodes = mGhost->GetGame()->mPathNodes;
	// Temp for closest node
	PathNode* closest = nullptr;
	// Temp for min distance and distance
	float minDistance = 100000.0f;
	float distance = 0.0f;
	// loop through nodes to find closest node from this position
	for (auto node : pathNodes)
	{
		// Check to see if it's not the prevNode
		if (node != mPrevNode)
		{
			distance = (node->GetPosition() - pos).Length();
			// Get distance from node to given position, see if less than current min
			if (distance < minDistance)
			{
				// set the min distance to distance
				minDistance = distance;
				// set closest to node
				closest = node;
			}
		}
	}
	return closest;
}

void GhostAI::FollowPlayer()
{
	// Get path from ghost's current to player's prev
	// If the playerNode is on tunnel, get closest default 
	if (mGhost->GetGame()->mPlayer->GetPrevNode()->GetType() == PathNode::Type::Tunnel)
	{
		// Temp to hold target
		PathNode* tunnelTarget = nullptr;
		// Loop through adj nodes to tunnel
		for (auto node : mGhost->GetGame()->mPlayer->GetPrevNode()->mAdjacent)
		{
			// If default set to this node
			if (node->GetType() == PathNode::Type::Default)
			{
				tunnelTarget = node;
			}
		}
		mPath = GetShortestPath(mNextNode, tunnelTarget);
	}
	else
	{
		mPath = GetShortestPath(mNextNode, mGhost->GetGame()->mPlayer->GetPrevNode());
	}
	
}

void GhostAI::UpdateTimer(float deltaTime)
{
	if (mState == State::Frightened)
	{
		// Update Frighten Timer with deltaTime
		timer += deltaTime;

		// Start flashing white by 5 seconds
		if (timer >= 5.0f)
		{
			mGhost->GetComponent<AnimatedSprite>()->SetAnimation("scared1");
			// Max frightened timer
			if (timer >= 7.0f)
			{
				// change state back to scatter
				mState = State::Scatter;
				// Reset timer
				timer = 0.0f;
				// set speed back to normal
				mSpeed = 90.0f;
			}
		}
	}
	else if (mState == State::Scatter)
	{
		// Speed
		mSpeed = 90.0f;

		// Update scatter timer with deltaTime
		timer += deltaTime;

		//  If scatter for 5 seconds
		if (timer >= 5.0f)
		{
			// Switch to chase state
			mState = State::Chase;

			// Reset timer
			timer = 0.0f;
		}
	}
	else if (mState == State::Chase)
	{
		// Update chase timer with deltaTime
		timer += deltaTime;

		// Update Speed
		mSpeed = 90.0f;

		// If chase for 20 seconds
		if (timer >= 20.0f)
		{
			// Switch to Scatter state
			mState = State::Scatter;

			// Reset timer
			timer = 0.0f;
		}
	}
}