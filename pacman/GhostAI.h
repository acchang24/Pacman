#pragma once
#include "Component.h"
#include "Math.h"
#include <vector>
#include <unordered_map>
#include <algorithm>

// Struct for node info to store info during search
struct NodeInfo 
{
	// Parent node/node before current node
	class PathNode* parent = nullptr;

	// f(x) = g(x) + h(x)
	float f = 0.0f;

	// g(x) true cost from start to this node
	float g = 0.0f;

	// h(x) heuristic estimate from this node to goal
	float h = 0.0f;

	// bool to check if node has been visited
	bool IsClosed = false; 

	// Unusable vector
	std::vector<PathNode*> Unusable;
};

class GhostAI : public Component
{
public:
	// Used to track the four different GhostAI states
	enum State
	{
		Scatter,
		Chase,
		Frightened,
		Dead
	};
	
	GhostAI(class Actor* owner);

	void Update(float deltaTime) override;
	
	// Called when the Ghost starts at the beginning
	// (or when the ghosts should respawn)
	void Start(class PathNode* startNode);
	
	// Get the current state
	State GetState() const { return mState; }
	
	// Called when the ghost should switch to the "Frightened" state
	void Frighten();
	
	// Called when the ghost should switch to the "Dead" state
	void Die();

	//  Helper function to draw GhostAI's current path
	void DebugDrawPath(struct SDL_Renderer* render);

private:
	// Member data for pathfinding

	// TargetNode is our current goal node
	class PathNode* mTargetNode = nullptr;
	// PrevNode is the last node we intersected
	// with prior to the current position
	class PathNode* mPrevNode = nullptr;
	// NextNode is the next node we're trying
	// to get to
	class PathNode* mNextNode = nullptr;

	// This vector always contains the path
	// from "next node" to "target node"
	// (if there is still such a path)
	std::vector<class PathNode*> mPath;

	// Current state of the Ghost AI
	State mState = State::Scatter;

	// Save the owning actor (cast to a Ghost*)
	class Ghost* mGhost;

	// Velocity of the ghosts
	float mSpeed = 90.0f;

	float timer = 0.0f;

	// Function to find shortest path between start and target nodes. Returns vector of PathNodes
	std::vector<class PathNode*> GetShortestPath(class PathNode* start, class PathNode* target);

	// Get Heuristic (Euclidean) Distance
	float GetHeuristicDistance(class PathNode* start, class PathNode* target);

	// Function for Ghosts to follow/update the path
	void FollowPath();

	// Update Animations Function
	void UpdateAnims(Vector2 direction);

	// Get random neighbor Node
	void GoToRandomNeighbor();

	// Get the closest Node with a given position
	class PathNode* GetClosestNode(Vector2 pos);

	// Function to follow player with prevNode
	void FollowPlayer();

	// Function to update timer
	void UpdateTimer(float deltaTime);
};
