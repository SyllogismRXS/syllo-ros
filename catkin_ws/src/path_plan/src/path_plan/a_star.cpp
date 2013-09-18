#include <iostream>
#include <math.h>

#include "a_star.h"

using std::cout;
using std::endl;

namespace syllo
{

     ///----------------------------------------------------------------
     /// Point class functions
     ///----------------------------------------------------------------
     double Point::euclidean_distance(Point p)
     {
          double result;
          result = sqrt( pow((x-p.x),2) + pow(y-p.y,2) );
          return result;
     }

     Point operator+(const Point &p1, const Point &p2)
     {
          return Point(p1.x + p2.x, p1.y + p2.y);
     }

     bool Point::operator==(const Point &other) const
     {
          return (this->x == other.x) && (this->y == other.y);
     }
     
     bool Point::operator!=(const Point &other) const
     {
          return !(*this == other);
     }

     bool operator<(const Point &p1, const Point &p2)
     {
          return (p1.x < p2.x) || (p1.x == p2.x && p1.y < p2.y);
     }


     ///----------------------------------------------------------------
     /// Node class functions
     ///----------------------------------------------------------------
     Node::Node(Point point) : point_(point) { }
     
     Node::Node(int x, int y)
     {
          Node(Point(x,y));
     }

     bool Node::operator==(const Node &other) const
     {
          return (this->point_.x == other.point_.x) && 
               (this->point_.y == other.point_.y);
     }

     bool Node::operator!=(const Node &other) const
     {
          return !(*this == other);
     }

     bool operator<(const Node &n1, const Node &n2)
     {
          return false;
     }

     void Node::compute_costs(Point goal)
     {
          h_ = point_.euclidean_distance(goal);
     }

     ///----------------------------------------------------------------
     /// Map Class functions
     ///----------------------------------------------------------------
     void Map::set_origin(int x, int y)
     {
          origin_.x = x;
          origin_.y = y;
     }

     int Map::fill_map(int *map, int rows, int cols)
     {
          map_ = Eigen::MatrixXi(rows, cols);
          for (int r = 0; r < rows; r++) {
               for (int c = 0; c < cols; c++) {
                    map_(r,c) = map[r*cols + c];
               }
          }
     }

     int Map::at(int x, int y)
     {
          return map_(x-origin_.x, y-origin_.y);
     }

     void Map::set_at(int value, int x, int y)
     {
          map_(x-origin_.x, y-origin_.y) = value;
     }

     int Map::at(Point point)
     {
          return this->at(point.x, point.y);
     }

     void Map::set_at(int value, Point point)
     {
          this->set_at(value, point.x, point.y);
     }

     ///----------------------------------------------------------------
     /// AStar Class functions
     ///----------------------------------------------------------------
     int AStar::set_map(Map *map)
     {
          map_ = map;
     }

     int AStar::generate_path(Node start, Node goal)
     {
          start_ = start;
          goal_ = goal;

          // Add starting node to open list
          start_.set_list(Node::open);
          open_.push_back(&start_);
     
          // Find node in the open list with the lowest F
          // TODO: Make this a sorted list
          double champ = 99999;
          std::list<Node*>::iterator champ_it;
          std::list<Node*>::iterator it;
          Node * node;
          for (it = open_.begin(); it != open_.end(); it++) {
               if ((*it)->f() < champ) {
                    champ = (*it)->f();
                    champ_it = it;
                    node = *it;
               }
          }

          // Switch the lowest cost F node from the open list
          // to the closed list;
          node->set_list(Node::closed);
          closed_.push_back(node);
          open_.erase(champ_it);
          
          for (int i = 0 ; i < 8; i++) {
               Point point;

               switch (i) {
               case Map::N:
                    point = node->point() + Point(0,1);
                    if (map_->at(point) > 50 || node->list() == Node::closed) {
                         // ignore the node (not walkable, in the closed list)
                         continue;
                    }
                    
                    if (node->list() != Node::open) {
                         
                    }                    

                    break;
               case Map::NE:
                    break;
               case Map::E:
                    break;
               case Map::SE:
                    break;
               case Map::S:
                    break;
               case Map::SW:
                    break;
               case Map::W:
                    break;
               case Map::NW:
                    break;
               default:
                    cout << "Invalid heading" << endl;
                    break;
               }
          }
          return 0;
     }
}
