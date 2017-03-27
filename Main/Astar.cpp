#include <WProgram.h>
#include "Astar.h"
#include <cmath>

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

void std::__throw_length_error(char const*) {
	while(1)
		cout << "ERROR : __throw_length_error" << endl;
}

Graph Graph::Instance;
AStar AStar::Instance(Graph::Instance);

void AStar::Node::setParent(const Node & parent)
{
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

void astar_test(int x, int y)
{
	Serial.printf("Test A*: (0,0) to (%d, %d)\r\n",x,y);
	AStar::Path path;

	AStar::Node source(AStarCoord(0, 0));
	AStar::Node dest(AStarCoord(x, y));

	Graph &g = Graph::Instance;
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

	auto ret = AStar::Instance.findPath(source, dest, path);

	if (ret == AStar::SUCCESS) cout << "SUCCESS" << endl;
	else if (ret == AStar::ERROR_NOT_FOUND) cout << "ERROR_NOT_FOUND" << endl;
	else if (ret == AStar::ERROR_OUT_OF_MEMORY) cout << "ERROR_OUT_OF_MEMORY" << endl;

	cout << path.size() << endl;

	for (auto it = path.begin(); it != path.end(); ++it) {
		g.putElement(it->_pos, Graph::Value::PATH);
	}

	cout << endl;
	g.print();
}

Graph::Graph()
{
	for (int y = 0; y < _height; y++) {
		for (int x = 0; x < _width; x++) {
			_data[x][y].v = Value::EMPTY;
		}
	}
}

Graph::InternalNode & Graph::operator[](const AStarCoord & c)
{
	if ((unsigned)c.x >= _width || (unsigned)c.y >= _height) {
		while (1)
			cout << "ERROR : Graph::operator []" << endl;
	}
	return _data[c.x][c.y];
}

const Graph::InternalNode & Graph::operator[](const AStarCoord & c) const
{
	if ((unsigned)c.x >= _width || (unsigned)c.y >= _height) {
		while (1)
			cout << "ERROR : Graph::operator [] const" << endl;
	}
	return _data[c.x][c.y];
}

void Graph::print(void) const
{
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

bool Graph::isNode(const AStarCoord & c) const
{
	if ((unsigned)c.x >= _width) return false;
	if ((unsigned)c.y >= _height) return false;
	return _data[c.x][c.y].v != Value::OBSTACLE;
}

std::vector<AStar::Node> Graph::getNeighbors(const AStar::Node & node) const
{
	std::vector<AStar::Node> ret;
	AStarCoord c;
	for (c.y = node._pos.y - 1; c.y <= node._pos.y + 1; c.y++) {
		for (c.x = node._pos.x - 1; c.x <= node._pos.x + 1; c.x++) {
			if (isNode(c)) {
				AStar::Node n(c);
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

std::vector<AStar::Node> Graph::getParents(const AStar::Node & node) const
{
	std::vector<AStar::Node> ret;
	AStarCoord cur = node._pos;
	while (cur != AStarCoord()) {
		ret.push_back(AStar::Node(cur));
		cur = (*this)[cur].parent;
	}
	return ret;
}

struct ClosedList {
	std::vector<AStar::Node> _data;
	Graph& _graph;

	ClosedList(Graph& graph)
		: _graph(graph) {

	}

	void insert(AStar::Node e) {
		for (auto it = _data.begin(); it != _data.end(); ++it) {
			if (*it == e) {
				_graph.putElement(e._pos, Graph::Value::CLOSED);
				*it = e;
				return;
			}
		}
		_graph.putElement(e._pos, Graph::Value::CLOSED);
		_data.push_back(e);
	}

	std::vector<AStar::Node>::const_iterator begin(void) const {
		return _data.begin();
	}

	std::vector<AStar::Node>::const_iterator end(void) const {
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

struct OpenList {
	std::vector<AStar::Node> _data;
	Graph& _graph;
	AStar::Node _dest;

	OpenList(Graph& graph, AStar::Node dest)
		: _graph(graph), _dest(dest) {
	}

	void insert(AStar::Node e) {
		for (auto it = _data.begin(); it != _data.end(); ++it) {
			if (*it == e) {
				_graph.putElement(e._pos, Graph::Value::OPEN);
				*it = e;
				return;
			}
		}
		_graph.putElement(e._pos, Graph::Value::OPEN);
		_data.push_back(e);
	}

	void pop(void) {
		auto rm = _data.begin();
		for (auto it = _data.begin(); it != _data.end(); ++it) {
			if (heuristic(*it) < heuristic(*rm)) {
				rm = it;
			}
		}
		_graph.putElement(rm->_pos, Graph::Value::EMPTY);
		_data.erase(rm);
	}

	int heuristic(const AStar::Node& n) const {
		return n.cost() + _graph.getDistance(n, _dest);
	}

	AStar::Node head(void) const {
		auto min = _data.begin();
		for (auto it = _data.begin(); it != _data.end(); ++it) {
			if (heuristic(*it) < heuristic(*min)) {
				min = it;
			}
		}
		return *min;
	}

	std::vector<AStar::Node>::const_iterator begin(void) const {
		return _data.begin();
	}

	std::vector<AStar::Node>::const_iterator end(void) const {
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

template<class List>
bool  AStar::tryInsert(List& list, const Node& node)
{
	bool ret = !list.isFull();
	if (ret) {
		list.insert(node);
	}
	return ret;
}

bool AStar::findBetter(const ClosedList & closed, const OpenList & open, const Node & node)
{
	for (auto it = closed.begin(); it != closed.end(); ++it) {
		if ((*it) == node) {
			return _graph.getCost(*it) <= _graph.getCost(node);
		}
	}

	for (auto it = open.begin(); it != open.end(); ++it) {
		if ((*it) == node) {
			return _graph.getCost(*it) <= _graph.getCost(node);
		}
	}

	return false;
}

AStar::ReturnStatus AStar::buildPath(const Node & last, Path & out_path)
{
	auto parents = _graph.getParents(last);
	for (auto it = parents.begin(); it != parents.end(); ++it) {
		const Node& n = *it;

		if (out_path.isFull()) {
			return ReturnStatus::ERROR_OUT_OF_MEMORY;
		}
		out_path.push(n);
	}

	return ReturnStatus::SUCCESS;
}

AStar::ReturnStatus AStar::findPath(const Node & source, const Node & destination, Path & out_path)
{
	OpenList open(_graph, destination);
	ClosedList closed(_graph);
	out_path.flush();

	if (!tryInsert(open, source)) {
		return ReturnStatus::ERROR_OUT_OF_MEMORY;
	}

	while (!open.isEmpty()) {
		const Node& current = open.head();
		open.pop();

		if (!tryInsert(closed, current)) {
			return ReturnStatus::ERROR_OUT_OF_MEMORY;
		}

		if (current == destination) {
			return buildPath(current, out_path);
		}

		auto neighbors = _graph.getNeighbors(current);
		for (auto it = neighbors.begin(); it != neighbors.end(); ++it) {
			Node& neighbor = *it;

			if (!findBetter(closed, open, neighbor)) {
				_graph.setParent(neighbor, current);
				if (!tryInsert(open, neighbor)) {
					return ReturnStatus::ERROR_OUT_OF_MEMORY;
				}
			}
		}
	}

	return ReturnStatus::ERROR_NOT_FOUND;
}
