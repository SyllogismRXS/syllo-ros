#include <iostream>
#include <math.h>

#include "path_plan/types.h"

#include <syllo_common/Filter.h>

using std::cout;
using std::endl;

namespace syllo
{

     ///----------------------------------------------------------------
     /// Point class functions
     ///----------------------------------------------------------------
     //template <class T>
     //double Point<T>::euclidean_distance(Point<T> p)
     //{
     //     double result;
     //     result = sqrt( pow((x-p.x),2) + pow(y-p.y,2) );
     //     return result;
     //}

     //template <class T>
     //Point<T> operator+(const Point<T> &p1, const Point<T> &p2)
     //{
     //     return Point<T>(p1.x + p2.x, p1.y + p2.y);
     //}
     //
     //template <class T>
     //Point<T> operator-(const Point<T> &p1, const Point<T> &p2)
     //{
     //     return Point<T>(p1.x - p2.x, p1.y - p2.y);
     //}
     
     
     //template <class T>
     //bool Point<T>::operator==(const Point<T> &other) const
     //{
     //     return (this->x == other.x) && (this->y == other.y);
     //}
     //
     //template <class T>
     //bool Point<T>::operator!=(const Point<T> &other) const
     //{
     //     return !(*this == other);
     //}
     //
     //template <class T>
     //bool Point<T>::operator<(const Point<T> &rhs)
     //{
     //     return (this->p1.x < rhs.x) || (this->p1.x == rhs.x && this->p1.y < rhs.y);
     //}

     Point<double> add_points(const Point<double> &p1, const Point<int> &p2)
     {
          Point<double> result;
          result.x = p1.x + (double)p2.x;
          result.y = p1.y + (double)p2.y;
          return result;
     }

     Point<double> sub_points(const Point<int> &p1, const Point<double> &p2)
     {
          Point<double> result;
          result.x = (double)p1.x - p2.x;
          result.y = (double)p1.y - p2.y;
          return result;
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

     Node::Node(Point<int> point) : point_(point), list_(None), parent_(NULL) { }
     
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

     void Node::compute_costs(Point<int> goal)
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

     bool Map::inMap(const Point<int> &point)
     {
          if (point.x < 0 || point.y < 0) {
               return false;
          }
          
          if (point.x >= x_width_ || point.y >= y_height_) {
               return false;
          }

          return true;
     }

     int Map::set_map(const std::vector<int> &map, int x_width, int y_height)
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

     int Map::get_map(std::vector<int> &map, int &x_width, int &y_height)
     {
          x_width = this->x_width_;
          y_height = this->y_height_;

          for (int x = 0; x < x_width_; x++) {
               for (int y = 0; y < y_height_; y++) {
                    map.push_back(map_[x][y]);
               }
          }

          return 0;
     }

     int Map::set_map(const cv::Mat &map)
     {
          this->x_width_ = map.cols;
          this->y_height_ = map.rows;

          map_.resize(boost::extents[x_width_][y_height_]);
          for (int x = 0; x < x_width_; x++) {
               for (int y = 0; y < y_height_; y++) {
                    map_[x][y] = 255 - map.at<uchar>(y_height_-1-y,x);
               }
          }

          return 0;
     }

     int Map::set_negated_map(const cv::Mat &map)
     {
          this->x_width_ = map.cols;
          this->y_height_ = map.rows;

          map_.resize(boost::extents[x_width_][y_height_]);
          for (int x = 0; x < x_width_; x++) {
               for (int y = 0; y < y_height_; y++) {
                    map_[x][y] = map.at<uchar>(y_height_-1-y,x);
               }
          }

          return 0;
     }

     int Map::get_map(cv::Mat &map)
     {
          map = cv::Mat(y_height_, x_width_, CV_8UC1);     
          for (int y = 0; y < y_height_; y++) {
               for (int x = 0; x < x_width_; x++) {
                    double value = this->at(x,y);
                    value = normalize(value, 0, 100, 0, 255);
                    map.at<uchar>(y_height_-1-y,x) = 255 - value;
               }
          }
          return 0;
     }

     int Map::get_negated_map(cv::Mat &map)
     {
          map = cv::Mat(y_height_, x_width_, CV_8UC1);     
          for (int y = 0; y < y_height_; y++) {
               for (int x = 0; x < x_width_; x++) {
                    double value = this->at(x,y);
                    value = normalize(value, 0, 100, 0, 255);
                    map.at<uchar>(y_height_-1-y,x) = value;
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

     int Map::at(Point<int> point)
     {
          return this->at(point.x, point.y);
     }

     void Map::set_at(int value, Point<int> point)
     {
          this->set_at(value, point.x, point.y);
     }

}
