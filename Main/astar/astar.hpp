
struct Node {
  bool operator==(const Node& other) const;
};

template<typename T>
struct Iterator {
  Iterator& operator++(int);
  T& operator*(void);
  bool operator!=(const Iterator& other);
};

template<typename T>
struct IterableList {
  Iterator<T>& begin(void) const;
  Iterator<T>& end(void) const;
};

struct Graph {
  enum ListType {
    NEIGHBOR,
    PARENT
  };

  template<ListType TYPE>
  struct List : IterableList<Node> {};

  using NeighborsList = List<NEIGHBOR>;
  using ParentsList = List<PARENT>;

  int getDistance(const Node& node1, const Node& node2) const;
  int getCost(const Node& node) const;
  NeighborsList& getNeighbors(const Node& node) const;
  ParentsList& getParents(const Node& node) const;
  void setParent(Node& child, const Node& parent);
};

struct ClosedList : IterableList<Node> {
  ClosedList(Graph& g);
  void insert(Node e);
  bool isEmpty(void) const;
  bool isFull(void) const;
  void flush(void);
};

struct OpenList : IterableList<Node> {
  OpenList(Graph& g, const Node& destination);
  void insert(Node e);
  void pop(void);
  Node head(void) const;
  bool isEmpty(void) const;
  bool isFull(void) const;
  void flush(void);
};

struct Path {
  void push(Node e);
  bool isFull(void);
  void flush();
};

template<class Graph, class Node, class ClosedList, class OpenList>
struct AStar {
private:
  bool findBetter(const ClosedList& closed, const OpenList& open, const Node& node) {
    for(auto it = closed.begin() ; it != closed.end() ; it++) {
      if((*it) == node) {
        return _graph.getCost(*it) <= _graph.getCost(node);
      }
    }

    for(auto it = open.begin() ; it != open.end() ; it++) {
      if((*it) == node) {
        return _graph.getCost(*it) <= _graph.getCost(node);
      }
    }

    return false;
  }

  template<class List>
  bool tryInsert(List& list, const Node& node) {
    bool ret = !list.isFull();
    if(ret) {
      list.insert(node);
    }
    return ret;
  }

private:
  Graph& _graph;
  
public:
  AStar(Graph& g)
    : _graph(g) {
  }
  
  enum ReturnStatus {
    SUCCESS,
    ERROR_NOT_FOUND,
    ERROR_OUT_OF_MEMORY
  };

  template<typename Path>
  ReturnStatus buildPath(const Node& last, Path& out_path) {
    auto parents = _graph.getParents(last);
    for(auto it = parents.begin() ; it != parents.end() ; it++) {
      const Node& n = *it;

      if(out_path.isFull()) {
	return ReturnStatus::ERROR_OUT_OF_MEMORY;
      }
      out_path.push(n);
    }
    
    return ReturnStatus::SUCCESS;
  }

  template<typename Path>
  ReturnStatus findPath(const Node& source, const Node& destination, Path& out_path) {
    OpenList open(_graph, destination);
    ClosedList closed(_graph);
    out_path.flush();

    if(!tryInsert(open, source)) {
      return ReturnStatus::ERROR_OUT_OF_MEMORY;
    }

    while(!open.isEmpty()) {
      const Node& current = open.head();
      open.pop();
      
      if(!tryInsert(closed, current)) {
	return ReturnStatus::ERROR_OUT_OF_MEMORY;
      }

      if(current == destination) {
	return buildPath(current, out_path);
      }
      
      auto neighbors = _graph.getNeighbors(current);
      for(auto it = neighbors.begin() ; it != neighbors.end() ; it++) {
	Node& neighbor = *it;
	
	if(!findBetter(closed, open, neighbor)) {
	  _graph.setParent(neighbor, current);
	  if(!tryInsert(open, neighbor)) {
	    return ReturnStatus::ERROR_OUT_OF_MEMORY;
	  }
	}
      }
    }

    return ReturnStatus::ERROR_NOT_FOUND;
  }
};
