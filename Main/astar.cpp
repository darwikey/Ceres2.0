#include <WProgram.h>

class DummyCout {};
class DummyEndl {};

template<typename T> DummyCout& operator<<(DummyCout& cout, T val) {
	Serial.print(val);
	return cout;
}

DummyCout& operator<<(DummyCout& cout, const char* str) {
	Serial.print(str);
	return cout;
}

DummyCout& operator<<(DummyCout& cout, const DummyEndl&) {
	Serial.println();
	return cout;
}

static DummyCout cout;
static DummyEndl endl;

#include "astar.hpp"
#include <vector>
#include <cmath>

void std::__throw_length_error(char const*) {
	while(1)
		cout << "ERROR : __throw_length_error" << endl;
}

struct MyNode {
	AStarCoord _pos;
	AStarCoord _parent;
	double _cost;

	MyNode(AStarCoord c)
		: _pos(c), _cost(0) {
	}

	MyNode(void)
		: _cost(0) {
	}

	MyNode(const MyNode& o)
		: _pos(o._pos), _parent(o._parent), _cost(o._cost) {
	}

	//MyNode& operator=(const MyNode& o) {
	//	_x = o.;
	//	_y = o._y;
	//	_parent_x = o._parent_x;
	//	_parent_y = o._parent_y;
	//	_cost = o._cost;
	//	return *this;
	//}

	int cost(void) const {
		return _cost;
	}

	void setParent(const MyNode& parent) {
		_parent = parent._pos;
		const double xx = _pos.x - _parent.x;
		const double yy = _pos.y - _parent.y;
		const double dist = sqrt(xx*xx + yy*yy);
		double angle = 0;
		if (parent._parent.x != -1 && parent._parent.y != -1) {
			const double pxx = parent._pos.x - parent._parent.x;
			const double pyy = parent._pos.y - parent._parent.y;
			const double pdist = sqrt(pxx*pxx + pyy*pyy);
			const double scal = xx*pxx + yy*pyy;
			const double cosa = scal / (dist*pdist);
			angle = fmod(acos(cosa), 3.15 / 2);
		}
		_cost = parent._cost + dist + (100 * angle);
	}

	bool operator==(const MyNode& other) const {
		return _pos == other._pos;
	}
};

class MyGraph {
public:
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
	MyGraph(void) {
		for (int y = 0; y < _height; y++) {
			for (int x = 0; x < _width; x++) {
				_data[x][y].v = Value::EMPTY;
			}
		}
	}

	InternalNode & operator [](const AStarCoord &c){
		if ((unsigned)c.x >= _width || (unsigned)c.y >= _height) {
			while (1)
				cout << "ERROR : MyGraph::operator []" << endl;
		}
		return _data[c.x][c.y];
	}
	const InternalNode & operator [](const AStarCoord &c) const {
		if ((unsigned)c.x >= _width || (unsigned)c.y >= _height) {
			while (1)
				cout << "ERROR : MyGraph::operator [] const" << endl;
		}
		return _data[c.x][c.y];
	}

	void print(void) const {
		for (int y = 0; y < _height; y++) {
			for (int x = 0; x < _width; x++) {
				auto v = _data[x][y].v;
				if (v == Value::EMPTY) {
					cout << " ";
				}
				if (v == Value::OBSTACLE) {
					cout << "O";
				}
				if (v == Value::CLOSED) {
					cout << "c";
				}
				if (v == Value::OPEN) {
					cout << ".";
				}
				if (v == Value::PATH) {
					cout << "X";
				}
			}
			cout << endl;
		}
	}

	static int myAbs(int val) {
		return val < 0 ? -val : val;
	}

	int getDistance(const MyNode& node1, const MyNode& node2) const {
		return myAbs(node1._pos.x - node2._pos.x) + myAbs(node1._pos.y - node2._pos.y);
	}

	bool isNode(const AStarCoord &c) const {
		if ((unsigned)c.x >= _width) return false;
		if ((unsigned)c.y >= _height) return false;
		return _data[c.x][c.y].v != Value::OBSTACLE;
	}

	std::vector<MyNode> getNeighbors(const MyNode& node) const {
		std::vector<MyNode> ret;
		AStarCoord c;
		for (c.y = node._pos.y - 1; c.y <= node._pos.y + 1; c.y++) {
			for (c.x = node._pos.x - 1; c.x <= node._pos.x + 1; c.x++) {
				if (isNode(c)) {
					MyNode n(c);
					if (!(node == n)) {
						n.setParent(node);
						ret.push_back(n);
					}
				}
			}
		}
		//cout << endl;
		//print();
		return ret;
	}

	std::vector<MyNode> getParents(const MyNode& node) const {
		std::vector<MyNode> ret;
		AStarCoord cur = node._pos;
		while (cur != AStarCoord()) {
			ret.push_back(MyNode(cur));
			cur = (*this)[cur].parent;
		}
		return ret;
	}

	void setParent(MyNode& child, const MyNode& parent) {
		child.setParent(parent);
		(*this)[child._pos].parent = parent._pos;
	}

	int getCost(const MyNode& node) {
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

struct MyClosedList {
	std::vector<MyNode> _data;
	MyGraph& _graph;

	MyClosedList(MyGraph& graph)
		: _graph(graph) {

	}

	void insert(MyNode e) {
		for (auto it = _data.begin(); it != _data.end(); ++it) {
			if (*it == e) {
				_graph.putElement(e._pos, MyGraph::Value::CLOSED);
				*it = e;
				return;
			}
		}
		_graph.putElement(e._pos, MyGraph::Value::CLOSED);
		_data.push_back(e);
	}

	std::vector<MyNode>::const_iterator begin(void) const {
		return _data.begin();
	}

	std::vector<MyNode>::const_iterator end(void) const {
		return _data.end();
	}

	bool isEmpty(void) const {
		return _data.size() == 0;
	}

	bool isFull(void) const {
		return false;
	}

	void flush(void) {
		_data.resize(0);
	}

};

struct MyOpenList {
	std::vector<MyNode> _data;
	MyGraph& _graph;
	MyNode _dest;

	MyOpenList(MyGraph& graph, MyNode dest)
		: _graph(graph), _dest(dest) {
	}

	void insert(MyNode e) {
		for (auto it = _data.begin(); it != _data.end(); ++it) {
			if (*it == e) {
				_graph.putElement(e._pos, MyGraph::Value::OPEN);
				*it = e;
				return;
			}
		}
		_graph.putElement(e._pos, MyGraph::Value::OPEN);
		_data.push_back(e);
	}

	void pop(void) {
		auto rm = _data.begin();
		for (auto it = _data.begin(); it != _data.end(); ++it) {
			if (heuristic(*it) < heuristic(*rm)) {
				rm = it;
			}
		}
		_graph.putElement(rm->_pos, MyGraph::Value::EMPTY);
		_data.erase(rm);
	}

	int heuristic(const MyNode& n) const {
		return n.cost() + _graph.getDistance(n, _dest);
	}

	MyNode head(void) const {
		auto min = _data.begin();
		for (auto it = _data.begin(); it != _data.end(); ++it) {
			if (heuristic(*it) < heuristic(*min)) {
				min = it;
			}
		}
		return *min;
	}

	std::vector<MyNode>::const_iterator begin(void) const {
		return _data.begin();
	}

	std::vector<MyNode>::const_iterator end(void) const {
		return _data.end();
	}

	bool isEmpty(void) const {
		return _data.size() == 0;
	}

	bool isFull(void) const {
		return false;
	}

	void flush(void) {
		_data.resize(0);
	}
};

struct MyPath : std::vector<MyNode> {
	void flush(void) {
		this->resize(0);
	}

	bool isFull(void) {
		return false;
	}

	void push(const MyNode& e) {
		this->insert(this->begin(), e);
	}
};

using MyAStar = AStar<MyGraph, MyNode, MyClosedList, MyOpenList>;

void astar_test(int x, int y)
{
	Serial.printf("Test A*: (0,0) to (%d, %d)\r\n",x,y);
	MyGraph g;
	MyAStar astar(g);
	MyPath path;

	MyNode source(AStarCoord(0, 0));
	MyNode dest(AStarCoord(x, y));

	g.putObstacle(5, 5);
	g.putObstacle(4, 4);
	g.putObstacle(5, 4);
	g.putObstacle(4, 5);

	g.putObstacle(5, 3);
	g.putObstacle(4, 3);

	g.putObstacle(3, 4);
	g.putObstacle(3, 5);
	g.putObstacle(2, 4);
	g.putObstacle(2, 5);

	auto ret = astar.findPath(source, dest, path);

	if (ret == MyAStar::SUCCESS) cout << "SUCCESS" << endl;
	else if (ret == MyAStar::ERROR_NOT_FOUND) cout << "ERROR_NOT_FOUND" << endl;
	else if (ret == MyAStar::ERROR_OUT_OF_MEMORY) cout << "ERROR_OUT_OF_MEMORY" << endl;

	cout << path.size() << endl;

	for (auto it = path.begin(); it != path.end(); ++it) {
		g.putElement(it->_pos, MyGraph::Value::PATH);
	}

	cout << endl;
	g.print();
}
