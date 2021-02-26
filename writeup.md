# ReND_Cpp_Route-Planning_project

> AUTHOR: SungwookLE(joker1251@naver.com)  
> DATE: '21.2/26  

### BUILD SCRIPT
|||  
|:---|---:|
|**COMPILE:**||
|1)|`mkdir build`|
|2)|`cd build`|
|3)|`cmake .. & make`|
|**RUN:**||
|1) default map:|<span style="color:red">./OSM_A_star_search|
|2) specific map:|<span style="color:blue"> ./OSM_A_star_search -f ../<your_osm_file.osm>|
|**TEST:**|<span style="color:green">./test|

## 1. INTRODUCTION
Route-Planning project was implemented with `io2d` package. The key algorithm to find route is *A star* [*algorithm*.](hhttps://github.com/SungwookLE/ReND_Cpp_Astar)  
The code structure is as below.
[![video](https://video.udacity-data.com/topher/2019/August/5d4373ed_lesson-diagrams/lesson-diagrams.png)](https://youtu.be/-orQxTCLOuw)
With RouteModel and RoutePlanner class, I implemented the route planner using heuristic value.

## 2. MAIN
The `main()` function was implemented below.
```c++
int main(int argc, const char **argv)
{ 
    ...
    // Build Model.
    RouteModel model(osm_data);

    // Create RoutePlanner object and perform A* search.
    RoutePlanner route_planner(model, start_x, start_y, end_x, end_y);
    route_planner.AStarSearch();

    std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

    // Render results of search.
    Render render{model};
    ...
}
```
[*OSM data*](https://www.openstreetmap.org/) is open source data. This data is composed of specific elements known as `node`, `way`, `relation`. Each node has relations with other node through way. For interpreting this data, I declared `RouteModel` class as model variable.

`RouteModel` <span style="color:red"> class header is as below.  
```c++
class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        Node * parent = nullptr;
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        float f_value = h_value+ g_value;
        std::vector<Node *> neighbors;

        void FindNeighbors();
        float distance(Node other) const {
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        int index;
        Node * FindNeighbor(std::vector<int> node_indices);
        RouteModel * parent_model = nullptr;
    };

    RouteModel(const std::vector<std::byte> &xml);
    Node &FindClosestNode(float x, float y);
    auto &SNodes() { return m_Nodes; }
    std::vector<Node> path;
    
  private:
    void CreateNodeToRoadHashmap();
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
    std::vector<Node> m_Nodes;

};
```
`RouteModel` class has Node class and `Node` class has `h_value`, `g_value`, `visited`, `f_value`. This values are essential things to implement the *A star*. Also this `class` has members `FindNeighbors()` and `FindClosestNode()`. `FindNeighbors()` is implemented for find nodes that have not been visited before, near here. `FindClosestNode()` is for finding the closestNode from a coordinate `x` and `y`.

Next, Let's look at the `route_planner` class. I added code annotation below code line. Check it while reading the source.
```c++
    RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
        // Convert inputs to percentage:
        start_x *= 0.01;
        start_y *= 0.01;
        end_x *= 0.01;
        end_y *= 0.01;
        /* TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
        Store the nodes you find in the RoutePlanner's start_node and end_node attributes.*/
        start_node = &m_Model.FindClosestNode(start_x, start_y);
        end_node =  &m_Model.FindClosestNode(end_x, end_y);
    }
```
Using `FindClosestNode`, start_node and end_node were set.

```c++
    // TODO 3: Implement the CalculateHValue method.
    // Tips:
    // - You can use the distance to the end_node for the h value.
    // - Node objects have a distance method to determine the distance to another node.
    float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
        float h_value = node->distance(*end_node);
        return h_value;
    }
```
Using `distance`, h_value was acquired. h_value is the distance from end_node to Here.

```c++
    // TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
    // Tips:
    // - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
    // - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
    // - Use CalculateHValue below to implement the h-Value calculation.
    // - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
    void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
        current_node->FindNeighbors();

        for(auto one : current_node->neighbors){
            one->parent = current_node;
            one->h_value = this->CalculateHValue(one);
            one->g_value = current_node->g_value  + current_node->distance(*one);
            one->visited = true;
            one->f_value = one->h_value + one->g_value;
            
            this->open_list.emplace_back(one);
        }
    }
```
This `AddNeighbors` is important process. When I arrived node, find the possible Neighbors can be reached. And then, Calculate the h_value and f_value that is tatal cost function. When choosing the next node, Node that has the smallest f_value would be selected.    
Also, set neighbor's parent as current_node, the final path will be printed. It is called linked list data structure.  

```c++
    // TODO 5: Complete the NextNode method to sort the open list and return the next node.
    // Tips:
    // - Sort the open_list according to the sum of the h value and g value.
    // - Create a pointer to the node in the list with the lowest sum.
    // - Remove that node from the open_list.
    // - Return the pointer.
    RouteModel::Node *RoutePlanner::NextNode() {
        if (!open_list.empty()){
            std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node *a, RouteModel::Node * b)
            {
            // descending order
            return (a->f_value) > (b->f_value);
            });

            RouteModel::Node *Next = this->open_list.back();
            this->open_list.pop_back();
            return Next;
        }
        else{
            return nullptr;
        }
    }
```
This `NextNode` is important process too. Using sorting function the node that has the smallest f_value would be choosen. After picking the next, then this node should be poped not to make this node choose again.  
And, also if the open_List was empty, then it returns nullptr. It means there are no exit or route to get the end_node.  
```c++
    // TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
    // Tips:
    // - This method should take the current (final) node as an argument and iteratively follow the 
    //   chain of parents of nodes until the starting node is found.
    // - For each node in the chain, add the distance from the node to its parent to the distance variable.
    // - The returned vector should be in the correct order: the start node should be the first element
    //   of the vector, the end node should be the last element.
    std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
        // Create path_found vector
        distance = 0.0f;
        std::vector<RouteModel::Node> path_found;

        // TODO: Implement your solution here.
        path_found.emplace_back(*current_node);
        RouteModel::Node* path_temp = current_node;

        // Pointer data handling (2/25)
        while ( (path_temp->parent) != start_node  ){
            
            distance += path_temp->distance( *path_temp->parent );
            path_temp = path_temp->parent;
            path_found.emplace_back(*path_temp);
        }
        path_found.emplace_back(*start_node);
        distance += path_temp->distance( *path_temp->parent );
        
        std::reverse(path_found.begin(), path_found.end());
        
        distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
        return path_found;
    }
```
This `ConstructFinalPath' is for constructing final path that this algorithm has found.   Using linked list(->parent), the total route was push_back in path_found vector.
```c++
    // TODO 7: Write the A* Search algorithm here.
    // Tips:
    // - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
    // - Use the NextNode() method to sort the open_list and return the next node.
    // - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
    // - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
    void RoutePlanner::AStarSearch() {
        RouteModel::Node *current_node = nullptr;
        // TODO: Implement your solution here.
        start_node->visited=true;
        current_node = start_node;
        open_list.emplace_back(current_node);

        while (current_node != end_node && current_node != nullptr){
            
            if (current_node != nullptr)
                this->AddNeighbors(current_node);
            
            if (!open_list.empty())
                current_node = this->NextNode();
            else
                break;
        }
        m_Model.path = this->ConstructFinalPath(current_node);


    }
```
Using class memeber, the A_star function was implemented here.  
In main() function, only this member was called to execute A_star algorithm.
From start_node to end_node, this while loop will be executed until finding the end_node. And then, final path will be printed on the map.  

## 3. CONCLUSION
Results is as below. The start node is (20, 50) and end node is (50, 150).
![image[NOTE]](./image.png)
Orang Line was the `FinalPath` that was returned this code.  
The total distance was 248.5 meters.  

## DISCUSSION  
It would be fun to make this program as navigator with `Get_Latitude,long...`,