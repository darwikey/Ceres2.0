#ifndef _ASTAR_H_
#define _ASTAR_H_

#include <vector>

void astar_test(int x, int y);

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
	int16_t x;
	int16_t y;
};


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

		void setParent(const Node& parent);

		bool operator==(const Node& other) const {
			return _pos == other._pos;
		}
	};

	struct Path : std::vector<Node> {
		void flush(void) {
			this->resize(0);
		}

		bool isFull(void) {
			return false;
		}

		void push(const Node& e) {
			this->insert(this->begin(), e);
		}
	};

	static AStar Instance;

	AStar(Graph& g)
		: _graph(g) {
	}

	enum ReturnStatus {
		SUCCESS,
		ERROR_NOT_FOUND,
		ERROR_OUT_OF_MEMORY
	};

	ReturnStatus buildPath(const Node& last, Path& out_path);
	ReturnStatus findPath(const Node& source, const Node& destination, Path& out_path);

private:
	Graph& _graph;

	bool findBetter(const ClosedList& closed, const OpenList& open, const Node& node);

	template<class List>
	bool tryInsert(List& list, const Node& node);
};

class Graph {
public:
	static Graph Instance;
	enum class Value : char {
		OBSTACLE,
		EMPTY,
		CLOSED,
		OPEN,
		PATH
	};

	struct InternalNode {
		AStarCoord parent;
		Value v;
	};

	static const int _width = 20;
	static const int _height = 15;

private:
	InternalNode _data[_width][_height];

public:
	Graph();

	InternalNode & operator [](const AStarCoord &c);
	const InternalNode & operator [](const AStarCoord &c) const;

	void print(void) const;

	static int myAbs(int val) {
		return val < 0 ? -val : val;
	}

	int getDistance(const AStar::Node& node1, const AStar::Node& node2) const {
		return myAbs(node1._pos.x - node2._pos.x) + myAbs(node1._pos.y - node2._pos.y);
	}

	bool isNode(const AStarCoord &c) const;
	std::vector<AStar::Node> getNeighbors(const AStar::Node& node) const;
	std::vector<AStar::Node> getParents(const AStar::Node& node) const;

	void setParent(AStar::Node& child, const AStar::Node& parent) {
		child.setParent(parent);
		(*this)[child._pos].parent = parent._pos;
	}

	int getCost(const AStar::Node& node) {
		return node.cost();
	}

	void putElement(unsigned x, unsigned y, Value v) {
		if (x >= _width) return;
		if (y >= _height) return;
		_data[x][y].v = v;
	}

	void putElement(const AStarCoord &c, Value v) {
		if ((unsigned)c.x >= _width) return;
		if ((unsigned)c.y >= _height) return;
		_data[c.x][c.y].v = v;
	}

	void putObstacle(unsigned x, unsigned y) {
		putElement(x, y, Value::OBSTACLE);
	}
};

#endif
