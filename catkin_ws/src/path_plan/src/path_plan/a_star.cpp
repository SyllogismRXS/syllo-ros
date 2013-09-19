#include <iostream>
#include <math.h>

#include "path_plan/a_star.h"
#include "path_plan/types.h"

using std::cout;
using std::endl;

namespace syllo
{
     ///----------------------------------------------------------------
     /// AStar Class functions
     ///----------------------------------------------------------------
     AStar::AStar()
     {
          directions_.push_back(Direction(Direction::N,Straight,Point(0,1)));
          directions_.push_back(Direction(Direction::NE,Diagonal,Point(1,1)));
          directions_.push_back(Direction(Direction::E,Straight,Point(1,0)));
          directions_.push_back(Direction(Direction::SE,Diagonal,Point(1,-1)));
          directions_.push_back(Direction(Direction::S,Straight,Point(0,-1)));
          directions_.push_back(Direction(Direction::SW,Diagonal,Point(-1,-1)));
          directions_.push_back(Direction(Direction::W,Straight,Point(-1,0)));
          directions_.push_back(Direction(Direction::NW,Diagonal,Point(-1,1)));
     }

     void AStar::reset()
     {
          open_.clear();
          closed_.clear();
          map_ = NULL;
          path_.clear();
     }

     int AStar::set_map(Map *map)
     {          
          map_ = map;
          
          // Create a Node for each element in the map
          node_map_.resize(boost::extents[map_->x_width()][map_->y_height()]);
          for (int x = 0 ; x < map_->x_width(); x++) {
               for (int y = 0; y < map_->y_height(); y++) {
                    if (node_map_[x][y] == NULL) {
                         //cout << "creating" << endl;
                         node_map_[x][y] = new Node(x,y);
                    } else {
                         node_map_[x][y]->reset();
                    }                   
               }
          }
     }

     int AStar::generate_path(Node start, Node goal)
     {
          start_ = start;
          goal_ = goal;

          // Ensure that start node is in map
          if (!map_->inMap(start_.point())) {
               return -3;
          }

          // Ensure that goal node is in map
          if (!map_->inMap(goal_.point())) {
               return -4;
          }

          //// Get the pointer to the starting node
          Node *start_ptr;
          start_ptr = node_map_[start_.point().x][start_.point().y];
          
          // Add starting node to the open list
          start_ptr->set_list(Node::Open);
          open_.push_back(start_ptr);

          bool goal_reached = false;

          do {
               // If the open list is empty, we didn't find the goal node
               if (open_.empty()) {
                    goal_reached = false;
                    break;
               }
          
               // Grab the first item off the list (list is sorted by F cost)
               Node *cur_node = open_.front();
               open_.pop_front();
          
               // Switch the lowest cost F node from the open list
               // to the closed list;
               cur_node->set_list(Node::Closed);
               closed_.push_back(cur_node);
                         
               // Was the last node added to the closed list the goal node?
               if (goal_.point() == cur_node->point()) {
                    goal_reached = true;
                    break;
               }
          
               // Loop through all directions
               std::vector<Direction>::iterator dir_it;
               for (dir_it = directions_.begin(); dir_it != directions_.end(); dir_it++) {
                    Point point;
                    Node *adj_node;
          
                    point = cur_node->point() + dir_it->point();
          
                    // Check to see if point is within the map boundary
                    if (!map_->inMap(point)) {
                         continue;
                    }
          
                    adj_node = node_map_[point.x][point.y];
                         
                    if (map_->at(point) > 50 || adj_node->list() == Node::Closed) {
                         // ignore the node (not walkable, in the closed list)
                         continue;
                    }
                    
                    if (adj_node->list() != Node::Open) {
                         adj_node->set_list(Node::Open);
                         adj_node->set_parent(cur_node, dir_it->dir(), dir_it->cost());
                         adj_node->compute_costs(cur_node->point());
          
                         // Add the node into the sorted open list
                         bool node_inserted = false;
                         std::list<Node*>::iterator it;
                         for (it = open_.begin(); it != open_.end(); it++) {
                              if (*adj_node < **it) {
                                   open_.insert(it, adj_node);
                                   node_inserted = true;
                                   break;
                              }
                         }
                         // If the node wasn't inserted in the middle of the
                         // list, add it to the back
                         if (!node_inserted) {
                              open_.push_back(adj_node);
                         }
                         
                    } else {
                         // Node is already on open list, check to see if
                         // this path is lower cost, resort
                         if ((cur_node->g() + dir_it->cost()) < adj_node->g()) {
                              adj_node->set_parent(cur_node, dir_it->dir(), dir_it->cost());
                              adj_node->compute_costs(cur_node->point());
                              open_.sort();
                         }
                    }
               }
          } while(true);                   

          if (goal_reached) {
               // Generate the path by walking backwards from the goal node
               // to the start node
               Node * node = node_map_[goal_.point().x][goal_.point().y];
               do {
                    path_.push_front(node);
                    node = node->parent();
               } while(node->point() != start_.point());

               return 0; 
          } else {
               return -1;
          }
     }

     std::list<Node*> & AStar::path()
     {
          return path_;
     }
}