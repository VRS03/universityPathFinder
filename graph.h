// Filename: graph.h 
// Author: Viraj Saudagar
//
// This class enables for the creation of a graph with weighted edges of templated types for
// both the vertex and weights. The graph uses an adjacency list which is implemented as a linked list of
// edge structs. 
//

#pragma once

#include <iostream>
#include <stdexcept>
#include <map>
#include <set> 

using namespace std;

// Templated class allows for custom types for vertex and weights. 
template<typename VertexT, typename WeightT>
class graph {
 private:
  // edge struct which is used to create the graphs adjacency list implemented as a linked list of edges. 
  struct edge{
    VertexT edgeTo;
    WeightT weight;
    edge* next;
  };

  // graph implementation as a map of vertex type to linked list of edge structs. 
  map<VertexT, edge*> myGraph;

  // Helper function -> creates a deep copy of a linked list of edges given a pointer to the head of linked list.
  edge* copyLinkedList(edge* head) const{

    if(head == nullptr){
        return nullptr;
    }

    edge* origHead = head;
    edge* newHead = new edge;
    newHead->edgeTo = origHead->edgeTo;
    newHead->weight = origHead->weight;
    newHead->next = nullptr;
    edge* newHeadPos = newHead;
        
    origHead = origHead->next;

    while(origHead != nullptr){

        edge* newNode = new edge;
        newNode->edgeTo = origHead->edgeTo;
        newNode->weight = origHead->weight;
        newNode->next = nullptr;
        newHeadPos->next = newNode;
            
        newHeadPos = newHeadPos->next;
        origHead = origHead->next;
    }

    return newHead;
  }

  // Helper Function -> frees the memory allocated within the class
  void freeMem(){

    for(auto& i: this->myGraph){

      edge* curr = i.second;

      while(curr != nullptr){

        edge* temp = curr;
        curr = curr->next;
        delete temp;

      }

    }

    this->myGraph.clear();

  }

 public:

  // default constructor 
  graph(){

  }

  // clears the memory within the class. 
  void clear(){
    freeMem();
    this->myGraph.clear();
  }

  // copy constructor -> makes deep copy of object.  
  graph(const graph& origObject){

    for(auto i : origObject.myGraph){

      edge* origHead = i.second;
      edge* copyHead = copyLinkedList(origHead);

      this->myGraph.emplace(i.first, nullptr);
      this->myGraph.at(i.first) = copyHead;

    }

  }

  // copy asignment overload -> allows created object to be assigned another object of the class creating a deep copy. 
  graph& operator=(const graph& objToCopy){

    if(this != &objToCopy){

      freeMem();

      for(auto i : objToCopy.myGraph){

        edge* origHead = i.second;
        edge* copyHead = copyLinkedList(origHead);

        this->myGraph.emplace(i.first, nullptr);
        this->myGraph.at(i.first) = copyHead;

      }

    }

    return *this;

  }

  // Destructor -> frees memory allocated when object goes out of scope. 
  ~graph(){

    freeMem();
    
  }

  //
  // NumVertices
  //
  // Returns the # of vertices currently in the graph.
  //
  int NumVertices() const {
    return this->myGraph.size();
  }

  //
  // NumEdges
  //
  // Returns the # of edges currently in the graph.
  //
  int NumEdges() const {
    
    int sum = 0; 

    for(auto i : this->myGraph){
      
      edge* curr = i.second;

      while(curr != nullptr){
        sum += 1;
        curr = curr->next;
      }

    }

    return sum;

  }

  //
  // addVertex
  //
  // Adds the vertex v to the graph if there's room, and if so
  // returns true.  If the graph is full, or the vertex already
  // exists in the graph, then false is returned.
  //
  bool addVertex(VertexT v) {
    
    if(this->myGraph.count(v) == 1){
      return false;
    }

    this->myGraph.emplace(v, nullptr);
    return true;

  }

  //
  // addEdge
  //
  // Adds the edge (from, to, weight) to the graph, and returns
  // true.  If the vertices do not exist or for some reason the
  // graph is full, false is returned.
  //
  // NOTE: if the edge already exists, the existing edge weight
  // is overwritten with the new edge weight.
  //
  bool addEdge(VertexT from, VertexT to, WeightT weight) {
    
    if(this->myGraph.count(from) == 0 || this->myGraph.count(to) == 0){
      return false;
    }

    edge* curr = this->myGraph.at(from);

    while(curr != nullptr){

      if(curr->edgeTo == to){
        curr->weight = weight;
        return true;
      }

      curr = curr->next;

    }


    edge* newEdge = new edge;
    newEdge->edgeTo = to;
    newEdge->weight = weight;
    newEdge->next = nullptr;

    edge* head = this->myGraph.at(from);
    edge* headCopy = head;

    if(head == nullptr){
      head = newEdge;
      this->myGraph.at(from) = head;
      return true;
    }

    
    while(head->next != nullptr){
      head = head->next;
    }

    head->next = newEdge;
    this->myGraph.at(from) = headCopy;
    return true;
    
  }

  //
  // getWeight
  //
  // Returns the weight associated with a given edge.  If
  // the edge exists, the weight is returned via the reference
  // parameter and true is returned.  If the edge does not
  // exist, the weight parameter is unchanged and false is
  // returned.
  //
  bool getWeight(VertexT from, VertexT to, WeightT& weight) const {
    //
    // we need to search the Vertices and find the position
    // of each vertex; this will denote the row and col to
    // access in the adjacency matrix:
    //

    if(this->myGraph.count(from) == 0 || this->myGraph.count(to) == 0){
      return false;
    }

    edge* curr = this->myGraph.at(from);

    while(curr != nullptr){

      if(curr->edgeTo == to){

        weight = curr->weight;
        return true;

      }

      curr = curr->next;

    }

    return false;
    
  }

  //
  // neighbors
  //
  // Returns a set containing the neighbors of v, i.e. all
  // vertices that can be reached from v along one edge.
  // Since a set is returned, the neighbors are returned in
  // sorted order; use foreach to iterate through the set.
  //
  set<VertexT> neighbors(VertexT v) const {

    set<VertexT>  S;

    if(this->myGraph.count(v) == 0){
      return S;
    }

    edge* curr = this->myGraph.at(v);

    while(curr != nullptr){
      S.insert(curr->edgeTo);
      curr = curr->next;
    }
    
    return S;

  }

  //
  // getVertices
  //
  // Returns a vector containing all the vertices currently in
  // the graph.
  //
  vector<VertexT> getVertices() const {

    vector<VertexT> ans;

    for(auto i : this->myGraph){

      ans.push_back(i.first); 

    }

    return ans;
    
  }

  //
  // dump
  //
  // Dumps the internal state of the graph for debugging purposes.
  //
  // Example:
  //    graph<string,int>  G(26);
  //    ...
  //    G.dump(cout);  // dump to console
  //
  void dump(ostream& output) const {
    output << "***************************************************" << endl;
    output << "********************* GRAPH ***********************" << endl;

    output << "**Num vertices: " << this->NumVertices() << endl;
    output << "**Num edges: " << this->NumEdges() << endl;

    output << endl;
    output << "**Vertices:" << endl;

    int count = 0;
    for(auto i : this->myGraph){
      output << " " << count << ". " << i.first << endl;
      count += 1;
    }

    output << endl;
    output << "**Edges:" << endl;

    for(auto i : this->myGraph){

      edge* curr = i.second;
      
      while(curr != nullptr){
        output << "(" << i.first << ", " << curr->edgeTo << ", " << curr->weight << ")" << endl;
        curr = curr->next;
      }

    }

    
    output << "**************************************************" << endl;
  }
};
