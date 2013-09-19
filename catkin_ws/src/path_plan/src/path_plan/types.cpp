#include <iostream>
#include <math.h>

#include "path_plan/types.h"

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
     Node::Node() {}
     void Node::set_parent(Node *parent, Direction::Compass dir, Cost_t cost)
     {
          parent_ = parent;
          direction_from_parent_ = dir;
          cost_from_parent_ = cost;
     }

     Node::Node(Point point) : point_(point), list_(None), parent_(NULL) { }
     
     Node::Node(int x, int y)
     {          
          point_.x = x;
          point_.y = y;
     }

     void Node::reset()
     {
          g_ = 0;
          h_ = 0;
          f_ = 0;
          parent_ = NULL;
          direction_from_parent_ = Direction::N;
          cost_from_parent_ = Zero;
          list_ = None;
     }

     //bool Node::operator==(const Node &other) const
     //{
     //     return (this->point_.x == other.point_.x) && 
     //          (this->point_.y == other.point_.y);
     //}
     //
     //bool Node::operator!=(const Node &other) const
     //{
     //     return !(*this == other);
     //}
     //
     //bool operator<(const Node &n1, const Node &n2)
     //{
     //     return false;
     //}
     
     bool Node::operator<(const Node &other) 
     {
          return this->f_ < other.f_;
     }

     void Node::compute_costs(Point goal)
     {
          h_ = point_.euclidean_distance(goal);
          g_ = parent_->g() + cost_from_parent_;
          f_ = h_ + g_;
     }     

     ///----------------------------------------------------------------
     /// Map Class functions
     ///----------------------------------------------------------------
     // Sets the pose of the bottom left pixel
     void Map::set_origin(int x, int y)
     {
          origin_.x = x;
          origin_.y = y;
     }

     bool Map::inMap(const Point &point)
     {
          if (point.x < 0 || point.y < 0) {
               return false;
          }
          
          if (point.x >= x_width_ || point.y >= y_height_) {
               return false;
          }

          return true;
     }

     int Map::fill_map(const std::vector<int> &map, int x_width, int y_height)
     {
          this->x_width_ = x_width;
          this->y_height_ = y_height;

          map_.resize(boost::extents[x_width_][y_height_]);
          for (int x = 0; x < x_width_; x++) {
               for (int y = 0; y < y_height_; y++) {
                    map_[x][y] = map[y*x_width_ + x];
               }
          }
          return 0;
     }

     int Map::at(int x, int y)
     {
          return map_[x-origin_.x][y-origin_.y];
     }

     void Map::set_at(int value, int x, int y)
     {
          map_[x-origin_.x][y-origin_.y] = value;
     }

     int Map::at(Point point)
     {
          return this->at(point.x, point.y);
     }

     void Map::set_at(int value, Point point)
     {
          this->set_at(value, point.x, point.y);
     }

}
