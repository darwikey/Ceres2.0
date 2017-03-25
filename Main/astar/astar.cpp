#include <WProgram.h>

class DummyCout {};
class DummyEndl {};

template<typename T> DummyCout& operator<<(DummyCout& cout, T val) {
	Serial.print(val);
}

DummyCout& operator<<(DummyCout& cout, const char* str) {
	Serial.print(str);
}

DummyCout& operator<<(DummyCout& cout, const DummyEndl&) {
	Serial.println();
}

static DummyCout cout;
static DummyEndl endl;

using namespace std;
#include "astar.hpp"
#include <vector>
#include <cmath>

void std::__throw_length_error(char const*) {
	cout << "ERROR : __throw_length_error" << endl;
}

struct MyNode {
  int _x;
  int _y;
  int _parent_x;
  int _parent_y;
  double _cost;

  MyNode(int x, int y)
    : _x(x), _y(y), _parent_x(-1), _parent_y(-1), _cost(0) {
  }

  MyNode(void)
    : _x(-1), _y(-1), _parent_x(-1), _parent_y(-1), _cost(0) {
  }

  MyNode(const MyNode& o)
    : _x(o._x), _y(o._y), _parent_x(o._parent_x), _parent_y(o._parent_y), _cost(o._cost) {
  }

  MyNode& operator=(const MyNode& o) {
    _x = o._x;
    _y = o._y;
    _parent_x = o._parent_x;
    _parent_y = o._parent_y;
    _cost = o._cost;
    return *this;
  }

  int cost(void) const {
    return _cost;
  }
  
  void setParent(const MyNode& parent) {
    _parent_x = parent._x;
    _parent_y = parent._y;
    const double xx = _x-_parent_x;
    const double yy = _y-_parent_y;
    const double dist = sqrt(xx*xx+yy*yy);
    double angle = 0;
    if(parent._parent_x != -1 && parent._parent_y != -1) {
      const double pxx = parent._x - parent._parent_x;
      const double pyy = parent._y - parent._parent_y;
      const double pdist = sqrt(pxx*pxx+pyy*pyy);
      const double scal = xx*pxx + yy*pyy;
      const double cosa = scal / (dist*pdist);
      angle = fmod(acos(cosa), 3.15/2);
      /*
      cerr << "xx = " << xx << endl;
      cerr << "yy = " << yy << endl;
      cerr << "dist = " << dist << endl;
      cerr << "pxx = " << pxx << endl;
      cerr << "pyy = " << pyy << endl;
      cerr << "pdist = " << pdist << endl;
      cerr << "scal = " << scal << endl;
      cerr << "cosa = " << cosa << endl;
      cerr << "angle = " << angle << endl;
      cerr << "angle (deg) = " << (angle*180/3.1415) << endl;
      */
    }
    _cost = parent._cost+dist+(100*angle);
  }
  
  bool operator==(const MyNode& other) const {
    return _x == other._x && _y == other._y;
  }
};

struct MyGraph {
  enum Value {
    OBSTACLE,
    EMPTY,
    CLOSED,
    OPEN,
    PATH
  };

  struct InternalNode {
    int p;
    Value v;
  };

  vector<InternalNode> _data;
  int _width = 20;
  int _height = 15;
  
  void print(void) const {
    for(int y = 0 ; y < _height ; y++) {
      for(int x = 0 ; x < _width ; x++) {
	auto v = _data[x*_height+y].v;
	if(v == EMPTY) {
	  cout << " ";
	}
	if(v == OBSTACLE) {
	  cout << "O";
	}
	if(v == CLOSED) {
	  cout << "C";
	}
	if(v == OPEN) {
	  cout << ".";
	}
	if(v == PATH) {
	  cout << "X";
	}
      }
      cout << endl;
    }
  }

  static int abs(int val) {
    return val < 0 ? -val : val;
  }
  
  int getDistance(const MyNode& node1, const MyNode& node2) const {
    return abs(node1._x - node2._x) + abs(node1._y - node2._y);
  }

  bool isNode(int x, int y) const {
    if(x < 0 || _width <= x) return false;
    if(y < 0 || _height <= y) return false;
    return !(_data[x*_height+y].v == OBSTACLE);
  }
  
  vector<MyNode> getNeighbors(const MyNode& node) const {
    vector<MyNode> ret;
    for(int y = node._y-1 ; y <= node._y+1 ; y++) {
      for(int x = node._x-1 ; x <= node._x+1 ; x++) {
	if(isNode(x,y)) {
	  MyNode n(x,y);
	  if(!(node == n)) {
	    n.setParent(node);
	    ret.push_back(n);
	  }
	}
      }
    }
    print();
    return ret;
  }

  vector<MyNode> getParents(const MyNode& node) const {
    vector<MyNode> ret;
    int cur = node._x*_height+node._y;
    while(cur != -1) {
      ret.push_back(MyNode(cur/_height, cur%_height));
      cur = _data[cur].p;
    }
    return ret;
  }

  void setParent(MyNode& child, const MyNode& parent) {
    child.setParent(parent);
    _data[child._x*_height+child._y].p = parent._x*_height+parent._y;
  }

  int getCost(const MyNode& node) {
    return node.cost();
  }

  MyGraph(void) {
    _data.resize(_width*_height, InternalNode{-1,EMPTY});
  }
  
  void putObstacle(int x, int y, Value v = OBSTACLE) {
    _data[x*_height+y].v = v;
  }
  
};

struct MyClosedList {
  vector<MyNode> _data;
  MyGraph& _graph;

  MyClosedList(MyGraph& graph)
    : _graph(graph) {

  }
  
  void insert(MyNode e) {
    for(auto it = _data.begin() ; it != _data.end() ; it++) {
      if(*it == e) {
	_graph.putObstacle(e._x, e._y, MyGraph::CLOSED);
	*it = e;
	return;
      }
    }
    _graph.putObstacle(e._x, e._y, MyGraph::CLOSED);
    _data.push_back(e);
  }

  vector<MyNode>::const_iterator begin(void) const {
    return _data.begin();
  }

  vector<MyNode>::const_iterator end(void) const {
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
  vector<MyNode> _data;
  MyGraph& _graph;
  MyNode _dest;

  MyOpenList(MyGraph& graph, MyNode dest)
    : _graph(graph), _dest(dest) {
  }
  
  void insert(MyNode e) {
    for(auto it = _data.begin() ; it != _data.end() ; it++) {
      if(*it == e) {
	_graph.putObstacle(e._x, e._y, MyGraph::OPEN);
	*it = e;
	return;
      }
    }
    _graph.putObstacle(e._x, e._y, MyGraph::OPEN);
    _data.push_back(e);
  }
  
  void pop(void) {
    auto rm = _data.begin();
    for(auto it = _data.begin() ; it != _data.end() ; it++) {
      if(heuristic(*it) < heuristic(*rm)) {
	rm = it;
      }
    }
    _graph.putObstacle(rm->_x, rm->_y, MyGraph::EMPTY);
    _data.erase(rm);
  }

  int heuristic(const MyNode& n) const {
    return n.cost() + _graph.getDistance(n, _dest);
  }
  
  MyNode head(void) const {
    auto min = _data.begin();
    for(auto it = _data.begin() ; it != _data.end() ; it++) {
      if(heuristic(*it) < heuristic(*min)) {
	min = it;
      }
    }
    return *min;
  }
  
  vector<MyNode>::const_iterator begin(void) const {
    return _data.begin();
  }

  vector<MyNode>::const_iterator end(void) const {
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

struct MyPath : vector<MyNode> {
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
using TestAStar = AStar<Graph, Node, ClosedList, OpenList>;

void astar_test(void) {
  cout << "Hello!" << endl;
  MyGraph g;
  MyAStar astar(g);
  MyPath path;

  MyNode source(0,0);
  MyNode dest(19,14);

  g.putObstacle(5,5);
  g.putObstacle(4,4);
  g.putObstacle(5,4);
  g.putObstacle(4,5);

  g.putObstacle(5,3);
  g.putObstacle(4,3);

  g.putObstacle(3,4);
  g.putObstacle(3,5);
  g.putObstacle(2,4);
  g.putObstacle(2,5);

  auto ret = astar.findPath(source, dest, path);

  if(ret == MyAStar::SUCCESS) cout << "SUCCESS" << endl;
  else if(ret == MyAStar::ERROR_NOT_FOUND) cout << "ERROR_NOT_FOUND" << endl;
  else if(ret == MyAStar::ERROR_OUT_OF_MEMORY) cout << "ERROR_OUT_OF_MEMORY" << endl;

  cout << path.size() << endl;

  for(auto it = path.begin() ; it != path.end() ; it++) {
    g.putObstacle(it->_x, it->_y, MyGraph::PATH);
    g.print();
  }
}
