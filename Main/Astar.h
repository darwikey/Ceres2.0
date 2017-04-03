#ifndef _ASTAR_H_
#define _ASTAR_H_

#include <vector>

struct AStarCoord
{
	AStarCoord() : x(-1), y(-1) {}
	AStarCoord(uint16_t _x, uint16_t _y) : x(_x), y(_y) {}
	bool operator==(const AStarCoord& other) const {
		return x == other.x && y == other.y;
	}
	bool operator!=(const AStarCoord& other) const {
		return x != other.x || y != other.y;
	}
	void ToWordPosition(float &_x, float &_y) const;
	void FromWordPosition(float _x, float _y);
	int16_t x;
	int16_t y;
};

void astar_test(AStarCoord _c);

struct ClosedList;
struct OpenList;
class Graph;

struct AStar {
public:
	struct Node {
		AStarCoord _pos;
		AStarCoord _parent;
		double _cost;

		Node(AStarCoord c)
			: _pos(c), _cost(0) {
		}

		Node(void)
			: _cost(0) {
		}

		Node(const Node& o)
			: _pos(o._pos), _parent(o._parent), _cost(o._cost) {
		}

		int cost(void) const {
			return _cost;
		}

		void SetParent(const Node& parent);

		bool operator==(const Node& other) const {
			return _pos == other._pos;
		}
	};

	struct Path : std::vector<Node> {
		void Flush(void) {
			this->resize(0);
		}

		bool IsFull(void) {
			return false;
		}

		void Push(const Node& e) {
			this->insert(this->begin(), e);
		}
	};

	static AStar Instance;

	AStar(Graph& g)
		: m_Graph(g) {
	}

	enum ReturnStatus {
		SUCCESS,
		ERROR_NOT_FOUND,
		ERROR_OUT_OF_MEMORY
	};

	ReturnStatus BuildPath(const Node& last, Path& out_path);
	ReturnStatus FindPath(const Node& source, const Node& destination, Path& out_path);

private:
	Graph& m_Graph;

	bool FindBetter(const ClosedList& closed, const OpenList& open, const Node& node);

	template<class List>
	bool TryInsert(List& list, const Node& node);
};

class Graph {
public:
	static Graph Instance;
	enum class Value : char {
		EMPTY,
		OBSTACLE,
		CLOSED,
		OPEN,
		PATH
	};

	struct InternalNode {
		AStarCoord parent;
		Value v;
	};

	static const int WIDTH = 60;
	static const int HEIGHT = 40;

private:
	InternalNode m_Data[WIDTH][HEIGHT];

public:
	Graph();
	void Init();

	InternalNode & operator [](const AStarCoord &c);
	const InternalNode & operator [](const AStarCoord &c) const;

	void Print(bool _debug=true) const;

	static int myAbs(int val) {
		return val < 0 ? -val : val;
	}

	int GetDistance(const AStar::Node& node1, const AStar::Node& node2) const {
		return myAbs(node1._pos.x - node2._pos.x) + myAbs(node1._pos.y - node2._pos.y);
	}

	bool IsNode(const AStarCoord &c) const;
	std::vector<AStar::Node> GetNeighbors(const AStar::Node& node) const;
	std::vector<AStar::Node> GetParents(const AStar::Node& node) const;

	void SetParent(AStar::Node& child, const AStar::Node& parent) {
		child.SetParent(parent);
		(*this)[child._pos].parent = parent._pos;
	}

	int GetCost(const AStar::Node& node) {
		return node.cost();
	}

	void PutElement(unsigned x, unsigned y, Value v) {
		if (x >= WIDTH) return;
		if (y >= HEIGHT) return;
		m_Data[x][y].v = v;
	}

	void PutElement(const AStarCoord &c, Value v) {
		if ((unsigned)c.x >= WIDTH) return;
		if ((unsigned)c.y >= HEIGHT) return;
		m_Data[c.x][c.y].v = v;
	}

	void PutObstacle(unsigned x, unsigned y) {
		PutElement(x, y, Value::OBSTACLE);
	}
	void PutObstacle(unsigned x0, unsigned y0, unsigned x1, unsigned y1);
};

#endif
